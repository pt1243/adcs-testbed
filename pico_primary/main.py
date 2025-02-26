from micropython import const
from machine import ADC, I2C, Pin, PWM, UART
import rp2
from time import sleep, sleep_ms, sleep_us, ticks_ms, ticks_us, ticks_diff
from collections import deque
import gc

led = Pin("LED", Pin.OUT)
led.value(0)


# this is not technically the correct type annotation - it should be typing.NoReturn, but installing type stubs is
# nontrivial
def flash_error_message(num_flashes: int) -> None:
    while True:
        for _ in range(num_flashes):
            led.value(1)
            sleep_ms(100)
            led.value(0)
            sleep_ms(100)
        sleep_ms(1000 - 2 * num_flashes * 100)


light_sensor_1 = ADC(Pin(28))
light_sensor_2 = ADC(Pin(27))
light_sensor_3 = ADC(Pin(26))


uart = UART(0)
uart.init(tx=Pin(12), rx=Pin(13))

pwm = PWM(Pin(16, Pin.OUT, pull=None), freq=50)


def set_speed(speed: float) -> None:
    speed = min(1, max(0, speed))  # clip to [0, 1]
    MAX_SAFE_THROTTLE = 0.40  # just in case
    speed = min(speed, MAX_SAFE_THROTTLE)
    pwm.duty_u16(65535 - int(3260 * (1 - speed) + 6540 * speed))


set_speed(0)

# IMU
IMU = const(0x69)  # I2C bus address for the IMU

# set up I2C
i2c = I2C(0, sda=Pin(4), scl=Pin(5))
if IMU not in i2c.scan():
    flash_error_message(1)


def setup_IMU() -> None:
    # Registers that only need to be written to once at startup
    REG_CONFIG = const(0x1A)
    REG_GYRO_CONFIG = const(0x1B)
    REG_ACCEL_CONFIG = const(0x1C)
    REG_ACCEL_CONFIG_2 = const(0x1D)
    REG_LP_MODE_CFG = const(0x1E)
    REG_PWR_MGMT_1 = const(0x6B)

    # disable sleep mode
    i2c.writeto_mem(IMU, REG_PWR_MGMT_1, b"\x01")

    # set sensitivity values
    # TODO: document all these values!

    # whatever configuration this is seems to work
    i2c.writeto_mem(IMU, REG_CONFIG, bytes([0b0_0_000_110]))
    #                                                 +++----------- DLPF_CFG
    i2c.writeto_mem(IMU, REG_GYRO_CONFIG, bytes([0b000_00_0_00]))
    #                                                  ||   ++------ FCHOICE_B
    #                                                  ++----------- FS_SEL
    i2c.writeto_mem(IMU, REG_ACCEL_CONFIG, bytes([0b000_00_000]))
    #                                                   ++---------- ACCEL_FS_SEL
    i2c.writeto_mem(IMU, REG_ACCEL_CONFIG_2, bytes([0b00_11_1_000]))
    #                                                    || | +++--- A_DLPF_CFG
    #                                                    || +------- ACCEL_FCHOICE_B
    #                                                    ++--------- DEC2_CFG
    i2c.writeto_mem(IMU, REG_LP_MODE_CFG, bytes([0b1_000_0000]))
    #                                              | +++------------ G_AVGCFG
    #                                              +---------------- GYRO_CYCLE


setup_IMU()

# these are floats so unfortunately no const() optimisations
GYRO_FACTOR = 250 / 32768  # LSB to degrees per second
ACCEL_FACTOR = 2000 / 32768  # LSB to milli-g

def read_16_bit_signed(value: bytes) -> int:
    number = 256 * value[0] + value[1]
    if number >= 32768:
        return number - 65536
    return number


# Registers needed for reading the IMU values
REG_ACCEL_XOUT = const(0x3B)
REG_ACCEL_YOUT = const(0x3D)
REG_ACCEL_ZOUT = const(0x3F)
REG_GYRO_XOUT = const(0x43)
REG_GYRO_YOUT = const(0x45)
REG_GYRO_ZOUT = const(0x47)

def read_imu() -> tuple[float, float, float, float, float, float]:
    # TODO: remove unnecessary reads (eg. accelerometer) to speed things up
    x_accel = i2c.readfrom_mem(IMU, REG_ACCEL_XOUT, 2)
    y_accel = i2c.readfrom_mem(IMU, REG_ACCEL_YOUT, 2)
    z_accel = i2c.readfrom_mem(IMU, REG_ACCEL_ZOUT, 2)

    x_gyro = i2c.readfrom_mem(IMU, REG_GYRO_XOUT, 2)
    y_gyro = i2c.readfrom_mem(IMU, REG_GYRO_YOUT, 2)
    z_gyro = i2c.readfrom_mem(IMU, REG_GYRO_ZOUT, 2)

    return (
        read_16_bit_signed(x_accel) * ACCEL_FACTOR,
        read_16_bit_signed(y_accel) * ACCEL_FACTOR,
        read_16_bit_signed(z_accel) * ACCEL_FACTOR,
        read_16_bit_signed(x_gyro) * GYRO_FACTOR,
        read_16_bit_signed(y_gyro) * GYRO_FACTOR,
        read_16_bit_signed(z_gyro) * GYRO_FACTOR,
    )


def read_imu_gyro_z() -> float:
    # REG_GYRO_ZOUT = 0x47
    z_gyro = i2c.readfrom_mem(IMU, REG_GYRO_ZOUT, 2)
    return read_16_bit_signed(z_gyro) * GYRO_FACTOR


times_rates = []
rates = []
times_outputs = []
outputs = []
p_outputs = []
i_outputs = []
d_outputs = []

class PID:
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float, max_delta: float = 0.10) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.max_delta = max_delta
        self.use_integral_buffer = False  # by default, use the infinite integral
        self.integral_buffer = deque([0 for _ in range(25)], 25)
        self.accumulated_integral_error = 0.
        self.last_time: int | None = None
        self.last_error: float | None = None
    
    def get_control_output(self, rotation_rate: float) -> float:
        # NOTE: this assumes that a positive error is associated with an increased control output
        # input: rotation rate in degrees per second
        # output: difference from nominal throttle level

        error = self.setpoint - rotation_rate
        current_time = ticks_us()
        if self.last_time is None:
            dt_integral = 0
            dt_derivative = None
        else:
            dt = ticks_diff(current_time, self.last_time) * 1e-6
            dt_integral = max(0, dt)  # ticks_diff should ensure that it is always positive, but just in case
            dt_derivative = max(1e-9, dt)  # prevent division by very small values
        
        # proportional term
        proportional_term = self.Kp * error
        
        # integral term: we need to keep track of the error over time
        if self.last_error is None:
            integral_error = error * dt_integral
        else:
            integral_error = (error + self.last_error) / 2 * dt_integral
        
        if self.use_integral_buffer:
            self.integral_buffer.append(integral_error)
            integral_term = self.Ki * sum(self.integral_buffer)
        else:
            self.accumulated_integral_error += integral_error
            integral_term = self.Ki * self.accumulated_integral_error

        # TODO: limit integral windup
        integral_term = min(0.2, max(-0.2, integral_term))

        if dt_derivative is not None and self.last_error is not None:
            d_error = error - self.last_error
            derivative_term = -self.Kd * d_error / dt_derivative
        else:
            derivative_term = 0

        output = proportional_term + integral_term + derivative_term
        print(f"P:      {proportional_term:+.6f}")
        print(f"I:      {integral_term:+.6f}")
        print(f"D:      {derivative_term:+.6f}")
        print(f"Output: {output:+.6f}")

        try:
            times_outputs.append(current_time)
            p_outputs.append(proportional_term)
            i_outputs.append(integral_term)
            d_outputs.append(derivative_term)
            outputs.append(output)
        except MemoryError:
            set_speed(0)
            flash_error_message(2)

        self.last_error = error
        self.last_time = current_time

        # if output <= self.max_delta or output >= self.max_delta:  # indicate PID output over limits
        #     led.value(1)
        # else:
        #     led.value(0)

        return min(max(-self.max_delta, output), self.max_delta)

# pid = PID(4e-3, 4e-4, 0, 0., max_delta=0.10)
pid = PID(1e-3, 3e-4, 5e-5, 0., max_delta=0.15)
nominal_speed = 0.23

BUFFER_LENGTH = 10
initial_reading = read_imu_gyro_z()
z_gyro_buffer = deque([initial_reading for _ in range(BUFFER_LENGTH)], BUFFER_LENGTH)

print("waiting for ready: primary button for detumbling, or secondary button for light following")
DETUMBLING_MODE = True

light_sensor_buffer = bytearray(6)

def read_light_sensors_from_secondary() -> tuple[int, int, int]:
    uart.write(b"\xff")
    # TODO: figure out how long to wait
    while not uart.any():
        sleep_ms(1)
    sleep_ms(10)
    uart.readinto(light_sensor_buffer)
    return (
        (light_sensor_buffer[0] << 8) + light_sensor_buffer[1],
        (light_sensor_buffer[2] << 8) + light_sensor_buffer[3],
        (light_sensor_buffer[4] << 8) + light_sensor_buffer[5],
    )

while True:
    if rp2.bootsel_button():
        uart.write(b"\x00")  # indicate detumbling mode
        while not uart.txdone():
            sleep_us(200)
        print("DETUMBLING MODE SELECTED")
        pid.setpoint = 0
        pid.Kp *= 1
        pid.Ki *= 1
        break
    elif uart.any() and uart.read() == b"\xff":  # light-following mode
        DETUMBLING_MODE = False
        print("LIGHT-FOLLOWING MODE SELECTED")
        pid.Kp *= 1
        pid.Ki *= 1
        pid.use_integral_buffer = False
        break
    sleep_ms(5)
uart.read()  # ensure that everything is cleared

print("about to start spinning...")
sleep_ms(1000)

print("spinning up to nominal speed")
set_speed(nominal_speed)

while True:
    if rp2.bootsel_button():
        break
    sleep_ms(10)

print("waiting to begin active control...")
sleep_ms(1000)
print("beginning active control")

current_speed = nominal_speed

LIGHT_SENSOR_BUFFER_LENGTH = 50
light_sensor_1_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_2_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_3_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_4_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_5_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_6_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)


FAST_ROTATION_RATE = 30
SLOW_ROTATION_RATE = 20
FINE_ROTATION_RATE = 20

# def get_desired_setpoint(light_sensor_readings: list[int]) -> float:
#     highest_light_sensor = 1
#     highest_light_sensor_value = light_sensor_readings[0]
#     for i, value in enumerate(light_sensor_readings[1:], start=2):
#         if value > highest_light_sensor_value:
#             highest_light_sensor_value = value
#             highest_light_sensor = i
#     if highest_light_sensor == 3:
#         print(f"Max light sensor is 3, commanding {FAST_ROTATION_RATE} dps")
#         return FAST_ROTATION_RATE
#     elif highest_light_sensor == 4:
#         print(f"Max light sensor is 4, commanding {-FAST_ROTATION_RATE} dps")
#         return -FAST_ROTATION_RATE
#     elif highest_light_sensor == 2:
#         print(f"Max light sensor is 2, commanding {SLOW_ROTATION_RATE} dps")
#         return SLOW_ROTATION_RATE
#     elif highest_light_sensor == 5:
#         print(f"Max light sensor is 5, commanding {-SLOW_ROTATION_RATE} dps")
#         return -SLOW_ROTATION_RATE
#     else:
#         if highest_light_sensor == 1:
#             print(f"Max light sensor is 1, commanding {FINE_ROTATION_RATE} dps")
#             return FINE_ROTATION_RATE
#         else:
#             print(f"Max light sensor is 6, commanding {-FINE_ROTATION_RATE} dps")
#             return -FINE_ROTATION_RATE


def get_desired_setpoint(light_sensor_readings: list[int]) -> float:
    highest_light_sensor = 1
    highest_light_sensor_value = light_sensor_readings[0]
    for i, value in enumerate(light_sensor_readings[1:], start=2):
        if value > highest_light_sensor_value:
            highest_light_sensor_value = value
            highest_light_sensor = i
    if highest_light_sensor == 3:
        print(f"Max light sensor is 3, commanding {FAST_ROTATION_RATE} dps")
        return FAST_ROTATION_RATE
    elif highest_light_sensor == 4:
        print(f"Max light sensor is 4, commanding {-FAST_ROTATION_RATE} dps")
        return -FAST_ROTATION_RATE
    elif highest_light_sensor == 2:
        print(f"Max light sensor is 2, commanding {SLOW_ROTATION_RATE} dps")
        return SLOW_ROTATION_RATE
    elif highest_light_sensor == 5:
        print(f"Max light sensor is 5, commanding {-SLOW_ROTATION_RATE} dps")
        return -SLOW_ROTATION_RATE
    else:
        fraction = light_sensor_readings[5] / (light_sensor_readings[0] + light_sensor_readings[5])
        print(f"Fraction = {fraction}, commanded rotation rate is {FINE_ROTATION_RATE - fraction * 2 * FINE_ROTATION_RATE} dps")
        return FINE_ROTATION_RATE - fraction * 2 * FINE_ROTATION_RATE


last_pid_call = ticks_us()
start = last_pid_call
last_recording = last_pid_call
pid.setpoint = 0
control = True
# main control loop
while True:
    if rp2.bootsel_button():
        print("exiting")
        set_speed(0)
        led.value(1)
        with open("recording.txt", "w+") as f:
            for (time, rate) in zip(times_rates, rates):
                f.write(f"0 {time} {rate} 0 0 0\n")
            for (time, output, p, i, d) in zip(times_outputs, outputs, p_outputs, i_outputs, d_outputs):
                f.write(f"1 {time} {output} {p} {i} {d}\n")
        led.value(0)
        while True:
            sleep_ms(1000)


    current_rotation_rate = read_imu_gyro_z()
    if abs(pid.setpoint - current_rotation_rate) <= 0.5:
        led.value(1)
    else:
        led.value(0)
    z_gyro_buffer.append(current_rotation_rate)

    current_time = ticks_us()
    
    if ticks_diff(current_time, last_recording) >= 100_000:
        last_recording = current_time
        try:
            times_rates.append(current_time)
            rates.append(sum(z_gyro_buffer) / BUFFER_LENGTH)
        except MemoryError:
            set_speed(0)
            flash_error_message(2)

    if ticks_diff(current_time, start) >= 5_000_000 and control:
        control = False
        pid.setpoint = 0

    time_since_last_pid_call_ms = ticks_diff(current_time, last_pid_call) / 1000
    if DETUMBLING_MODE:
        if time_since_last_pid_call_ms >= 100:
            last_pid_call = current_time
            averaged_input = sum(z_gyro_buffer) / BUFFER_LENGTH
            print(f"\nCalling PID with rotation rate of {averaged_input:+.4f} dps")
            diff = pid.get_control_output(averaged_input)
            current_speed = nominal_speed + diff
            print(f"Commanded current speed is now {current_speed:.6f}")
            set_speed(current_speed)
    else:
        light_sensor_4_value, light_sensor_5_value, light_sensor_6_value = read_light_sensors_from_secondary()
        light_sensor_1_buffer.append(light_sensor_1.read_u16())
        light_sensor_2_buffer.append(light_sensor_2.read_u16())
        light_sensor_3_buffer.append(light_sensor_3.read_u16())
        light_sensor_4_buffer.append(light_sensor_4_value)
        light_sensor_5_buffer.append(light_sensor_5_value)
        light_sensor_6_buffer.append(light_sensor_6_value)
        if time_since_last_pid_call_ms >= 100:
            last_pid_call = current_time
            averaged_input = sum(z_gyro_buffer) / BUFFER_LENGTH
            pid.setpoint = get_desired_setpoint([
                sum(light_sensor_1_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                sum(light_sensor_2_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                sum(light_sensor_3_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                sum(light_sensor_4_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                sum(light_sensor_5_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                sum(light_sensor_6_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
            ])
            diff = pid.get_control_output(averaged_input)
            current_speed = nominal_speed + diff
            set_speed(current_speed)



# Notes on PID:
# integral: just add dt * Ki * current system value to an integral term (limit this to prevent integral windup)
# derivative: unsure if value is the last control input or the system value, but it is Kd * (current - last) / dt
# (note: may need a minus sign; in general, check all signs with gyro +ve direction, RW direction, etc).
