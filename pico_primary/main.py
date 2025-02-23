from micropython import const
from machine import ADC, I2C, Pin, PWM, UART
import rp2
from time import sleep, sleep_ms, ticks_ms, ticks_us, ticks_diff
from collections import deque

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


light_sensor_0 = ADC(Pin(26))
light_sensor_1 = ADC(Pin(27))
light_sensor_2 = ADC(Pin(28))

uart = UART(0)
uart.init(tx=Pin(12), rx=Pin(13))

pwm = PWM(Pin(16, Pin.OUT, pull=None), freq=50)


def set_speed(speed: float) -> None:
    speed = min(1, max(0, speed))  # clip to [0, 1]
    MAX_SAFE_THROTTLE = 0.40  # just in case
    speed = min(speed, MAX_SAFE_THROTTLE)
    pwm.duty_u16(65535 - int(3260 * (1 - speed) + 6540 * speed))


current_speed = 0
set_speed(current_speed)

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

    # DLPF_CFG = 6  # 0 through 7
    # print((DLPF_CFG << 0) == 0b0_0_000_110)

    # FCHOICE_B = 0  # 0 through 3
    # FS_SEL = 0  # 0 through 3
    # print((FS_SEL << 3 + FCHOICE_B << 0) == 0b000_00_0_00)

    # ACCEL_FS_SEL = 0  # 0 through 3
    # print((ACCEL_FS_SEL << 3) == 0b000_00_000)

    # A_DLPF_CFG = 0  # 0 through 7
    # ACCEL_FCHOICE_B = 1  # 0 or 1
    # DEC2_CFG = 3  # 0 through 3
    # print((A_DLPF_CFG << 0 + ACCEL_FCHOICE_B << 3 + DEC2_CFG << 4) == 0b00_11_1_000)

    # G_AVGCFG = 0  # 0 through 7
    # GYRO_CYCLE = 1  # 0 or 1
    # print((G_AVGCFG << 4 + GYRO_CYCLE << 7) == 0b1_000_0000)

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


# x_gyro_buffer = deque([0 for _ in range(BUFFER_LENGTH)], BUFFER_LENGTH)
# y_gyro_buffer = deque([0 for _ in range(BUFFER_LENGTH)], BUFFER_LENGTH)

# while True:
#     if rp2.bootsel_button():
#         break
#     sleep_ms(10)

# sleep_ms(1000)
# set_speed(0.10)

# while True:
#     if rp2.bootsel_button():
#         break
#     sleep_ms(10)

# set_speed(0)

# while True:
#     sleep_ms(100)

# while True:
    # start = ticks_us()
    # imu_value = read_imu()
    # x_gyro_buffer.append(imu_value[3])
    # y_gyro_buffer.append(imu_value[4])
    # z_gyro_buffer.append(imu_value[5])
    # readings += 1
    # end = ticks_us()
    # print(f"Reading values into buffer took {ticks_diff(end, start) / 1000} ms")

    # current_time = ticks_us()
    # time_since_last_report = ticks_diff(current_time, last_reported_measurement) / 1000
    # # print(f"Time since last report: {time_since_last_report} ms")

    # if time_since_last_report >= 100:  # 100 ms
    #     print(
    #         f"Current buffer values, from {readings} readings: {sum(x_gyro_buffer) / BUFFER_LENGTH}, {sum(y_gyro_buffer) / BUFFER_LENGTH}, {sum(z_gyro_buffer) / BUFFER_LENGTH}"
    #     )
    #     last_reported_measurement = ticks_us()
    #     readings = 0

    # end = ticks_us()
    # print(f"IMU value: {imu_value[3:]}")
    # print(f"Time to read IMU: {ticks_diff(end, start) / 1000} ms")

    # start = ticks_ms()
    # for _ in range(1000):
    #     read_imu()
    # end = ticks_ms()
    # print(f"Time in ms to read IMU, averaged over 1000 readings: {ticks_diff(end, start) / 1000}")
    # light_sensor_0_value = light_sensor_0.read_u16()
    # light_sensor_1_value = light_sensor_1.read_u16()
    # light_sensor_2_value = light_sensor_2.read_u16()

    # print(f"Light sensor 0: {light_sensor_0_value}")
    # print(f"Light sensor 1: {light_sensor_1_value}")
    # print(f"Light sensor 2: {light_sensor_2_value}")
    # print()

    # sleep_ms(200)


class PID:
    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral_buffer = deque([0 for _ in range(10)], 10)
        self.accumulated_integral_error = 0.
        self.last_time: int | None = None
        self.last_error: float | None = None
    
    def get_control_output(self, rotation_rate: float) -> float:
        # NOTE: this assumes that a positive error is associated with an increased control output
        # input: rotation rate in degrees per second
        # output: throttle level

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
            # integral_error = error * dt_integral
            self.accumulated_integral_error += error * dt_integral
        else:
            self.accumulated_integral_error += (error + self.last_error) / 2 * dt_integral
            # integral_error = (error + self.last_error) / 2 * dt_integral
        # self.integral_buffer.append(integral_error)

        # integral_term = self.Ki * sum(self.integral_buffer)
        integral_term = self.Ki * self.accumulated_integral_error
        # print(f"Accumulated integral: {self.accumulated_integral_error:+.6f}")
        # TODO: limit integral windup
        integral_term = min(0.08, max(-0.08, integral_term))

        if dt_derivative is not None and self.last_error is not None:
            d_error = error - self.last_error
            derivative_term = -self.Kd * d_error / dt_derivative * 1e-6
        else:
            derivative_term = 0

        output = proportional_term + integral_term + derivative_term
        print(f"P:      {proportional_term:+.6f}")
        print(f"I:      {integral_term:+.6f}")
        print(f"D:      {derivative_term:+.6f}")
        print(f"Output: {output:+.6f}")
        print()

        self.last_error = error
        self.last_time = current_time

        if output <= -0.10 or output >= 0.10:
            led.value(1)
        else:
            led.value(0)

        return min(max(-0.10, output), 0.10)

pid = PID(6e-4, 3e-4, 1, 0.)
nominal_speed = 0.18

BUFFER_LENGTH = 10
initial_reading = read_imu_gyro_z()
z_gyro_buffer = deque([initial_reading for _ in range(BUFFER_LENGTH)], BUFFER_LENGTH)

print("waiting for ready")
while True:
    if rp2.bootsel_button():
        break
    sleep_ms(10)

print("about to start spinning...")
sleep_ms(1000)

print("spinning up to nominal speed")
set_speed(nominal_speed)

while True:
    if rp2.bootsel_button():
        break
    sleep_ms(10)

print("waiting to begin PID control...")
sleep_ms(1000)
print("beginning PID control")

last_pid_call = ticks_us()
# main control loop
while True:
    if rp2.bootsel_button():
        print("exiting")
        set_speed(0)
        led.value(0)
        while True:
            sleep_ms(1000)

    current_rotation_rate = read_imu_gyro_z()
    if abs(current_rotation_rate) <= 1:
        led.value(1)
    else:
        led.value(0)
    z_gyro_buffer.append(current_rotation_rate)


    current_time = ticks_us()
    time_since_last_pid_call_ms = ticks_diff(current_time, last_pid_call) / 1000
    if time_since_last_pid_call_ms >= 100:
        last_pid_call = current_time

        averaged_input = sum(z_gyro_buffer) / BUFFER_LENGTH
        print(f"Calling PID with rotation rate of {averaged_input:+.4f} dps")
        diff = pid.get_control_output(averaged_input)
        
        set_speed(nominal_speed + diff)




# Notes on PID:
# integral: just add dt * Ki * current system value to an integral term (limit this to prevent integral windup)
# derivative: unsure if value is the last control input or the system value, but it is Kd * (current - last) / dt
# (note: may need a minus sign; in general, check all signs with gyro +ve direction, RW direction, etc).
