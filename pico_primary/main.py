from micropython import const
from machine import ADC, I2C, Pin, PWM, UART
import rp2
from time import sleep_ms, sleep_us, ticks_us, ticks_diff
from collections import deque

# initialise LED
led = Pin("LED", Pin.OUT)
led.value(0)


# note: this is not technically the correct type annotation - it should be typing.NoReturn, but installing type stubs is
# nontrivial
def flash_error_message(num_flashes: int) -> None:
    """Indicate an error code and halt execution of other code."""
    while True:
        for _ in range(num_flashes):
            led.value(1)
            sleep_ms(100)
            led.value(0)
            sleep_ms(100)
        sleep_ms(1000 - 2 * num_flashes * 100)


# configure light sensor ADC
light_sensor_1 = ADC(Pin(28))
light_sensor_2 = ADC(Pin(27))
light_sensor_3 = ADC(Pin(26))

# initialise UART controller
uart = UART(0)
uart.init(tx=Pin(12), rx=Pin(13))

# initialise PWM control for the ESC
pwm = PWM(Pin(16, Pin.OUT, pull=None), freq=50)


def set_speed(speed: float) -> None:
    """Set the ESC speed as a value from 0 to 1."""
    speed = min(1, max(0, speed))  # clip to [0, 1]
    MAX_SAFE_THROTTLE = 0.40  # just in case
    speed = min(speed, MAX_SAFE_THROTTLE)
    pwm.duty_u16(65535 - int(3260 * (1 - speed) + 6540 * speed))  # convert to a pulse ranging from 1 to 2 ms


# we need to start the PWM signal so the ESC does not complain, but we don't want the wheel to immediately start
# spinning right when the battery is connected
set_speed(0)

# IMU
IMU = const(0x69)  # I2C bus address for the IMU

# set up I2C for the IMU
i2c = I2C(0, sda=Pin(4), scl=Pin(5))
if IMU not in i2c.scan():  # IMU is needed for all modes of operation, so if it is not detected, halt execution
    flash_error_message(1)


def setup_IMU() -> None:
    """Write values to the IMU registers to configure it appropriately."""
    # Registers that only need to be written to once at startup
    REG_CONFIG = const(0x1A)
    REG_GYRO_CONFIG = const(0x1B)
    REG_ACCEL_CONFIG = const(0x1C)
    REG_ACCEL_CONFIG_2 = const(0x1D)
    REG_LP_MODE_CFG = const(0x1E)
    REG_PWR_MGMT_1 = const(0x6B)

    # disable sleep mode
    i2c.writeto_mem(IMU, REG_PWR_MGMT_1, b"\x01")

    # set values; the comments under each bit indicate the relevant parameter in the IMU datasheet
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

# factors to convert sensor readings to physical values
# these are floats so unfortunately no const() optimisations
GYRO_FACTOR = 250 / 32768  # LSB to degrees per second
ACCEL_FACTOR = 2000 / 32768  # LSB to milli-g


def read_16_bit_signed(value: bytes) -> int:
    """Helper function to read a 2-byte signed 16-bit integer."""
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
    """Read all accelerometer and gyroscope values from the IMU."""
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
    """Read only the z-axis rotation from the IMU. If not all values are needed this is faster."""
    z_gyro = i2c.readfrom_mem(IMU, REG_GYRO_ZOUT, 2)
    return read_16_bit_signed(z_gyro) * GYRO_FACTOR


class PID:
    """Simple class to manage the PID controller."""

    def __init__(self, Kp: float, Ki: float, Kd: float, setpoint: float, max_delta: float = 0.10) -> None:
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.max_delta = max_delta
        self.use_integral_buffer = False  # by default, use the infinite integral
        self.integral_buffer = deque([0 for _ in range(25)], 25)
        self.accumulated_integral_error = 0.0
        self.last_time: int | None = None
        self.last_error: float | None = None

    def get_control_output(self, rotation_rate: float) -> float:
        """Calculate the desired throttle output as an offset from the nominal throttle for the given rotation rate."""
        # NOTE: this assumes that a positive error is associated with an increased control output
        # input: rotation rate in degrees per second
        # output: difference from nominal throttle level

        error = self.setpoint - rotation_rate
        current_time = ticks_us()
        # we need the time between calls to calculate the integral and derivative terms; special handling is therefore
        # needed for the first call
        if self.last_time is None:
            dt_integral = 0
            dt_derivative = None
        else:
            dt = ticks_diff(current_time, self.last_time) * 1e-6
            dt_integral = max(0, dt)  # ticks_diff should ensure that it is always positive, but just in case
            dt_derivative = max(1e-9, dt)  # prevent division by very small values causing large derivatives

        # proportional term
        proportional_term = self.Kp * error

        # integral term: we need to keep track of the error over time
        if self.last_error is None:
            integral_error = error * dt_integral
        else:  # average over last two error readings for hopefully a more accurate value
            integral_error = (error + self.last_error) / 2 * dt_integral

        # either use a fixed-length integral buffer, or an unbounded integral
        if self.use_integral_buffer:
            self.integral_buffer.append(integral_error)
            integral_term = self.Ki * sum(self.integral_buffer)
        else:
            self.accumulated_integral_error += integral_error
            integral_term = self.Ki * self.accumulated_integral_error

        # there are probably other, better ways to limit integral windup, but this seems to work
        integral_term = min(0.2, max(-0.2, integral_term))

        if dt_derivative is not None and self.last_error is not None:
            d_error = error - self.last_error
            derivative_term = -self.Kd * d_error / dt_derivative
        else:
            derivative_term = 0

        output = proportional_term + integral_term + derivative_term

        # for debugging when connected to a computer
        print(f"P:      {proportional_term:+.6f}")
        print(f"I:      {integral_term:+.6f}")
        print(f"D:      {derivative_term:+.6f}")
        print(f"Output: {output:+.6f}")

        self.last_error = error
        self.last_time = current_time

        return min(max(-self.max_delta, output), self.max_delta)


pid = PID(1e-3, 3e-4, 5e-5, 0.0, max_delta=0.15)
nominal_speed = 0.23

BUFFER_LENGTH = 10  # gyro readings buffer size
initial_reading = read_imu_gyro_z()
z_gyro_buffer = deque([initial_reading for _ in range(BUFFER_LENGTH)], BUFFER_LENGTH)  # fixed-length FIFO queue

print("Waiting for ready: primary button for detumbling, or secondary button for light following")
DETUMBLING_MODE = True

# create a values buffer for the light sensor values that are being read from the other chip. reusing the buffer
# reduces the latency with the cross-chip communication as the values can be read directly to the buffer without having
# to instantiate a new object each time.
light_sensor_buffer = bytearray(6)


def read_light_sensors_from_secondary() -> tuple[int, int, int]:
    """Read and decode light sensor values from the secondary pico."""
    uart.write(b"\xff")
    while not uart.any():
        sleep_us(500)
    sleep_ms(5)  # ensure that the transmission is finished
    uart.readinto(light_sensor_buffer)
    return (
        (light_sensor_buffer[0] << 8) + light_sensor_buffer[1],
        (light_sensor_buffer[2] << 8) + light_sensor_buffer[3],
        (light_sensor_buffer[4] << 8) + light_sensor_buffer[5],
    )


while True:  # determine selected mode based on which button is pressed
    if rp2.bootsel_button():
        uart.write(b"\x00")  # indicate detumbling mode
        while not uart.txdone():
            sleep_us(200)
        print("DETUMBLING MODE SELECTED")
        pid.setpoint = 0
        # set gains that perform best for detumbling
        pid.Kp = 1e-3
        pid.Ki = 3e-4
        break
    elif uart.any() and uart.read() == b"\xff":  # light-following mode
        DETUMBLING_MODE = False
        print("LIGHT-FOLLOWING MODE SELECTED")
        # set gains that work best for light following
        pid.Kp = 3e-3
        pid.Ki = 5e-4
        pid.use_integral_buffer = False
        break
    sleep_ms(5)
uart.read()  # ensure that everything is cleared

print("about to start spinning...")
sleep_ms(1000)  # slight delay after mode selection to allow the platform to be secured

print("spinning up to nominal speed")
set_speed(nominal_speed)

while True:  # wait until button is pressed to begin active control
    if rp2.bootsel_button():
        break
    sleep_ms(10)

print("waiting to begin active control...")
sleep_ms(1000)
print("beginning active control")

current_speed = nominal_speed

LIGHT_SENSOR_BUFFER_LENGTH = 50  # average the light sensor values to reduce noice
light_sensor_1_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_2_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_3_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_4_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_5_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_6_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)


# values used for light-following mode
FAST_ROTATION_RATE = 30
SLOW_ROTATION_RATE = 20
FINE_ROTATION_RATE = 20


def get_desired_setpoint(light_sensor_readings: list[int]) -> float:
    """Determing the PID setpoint given the light sensor readings, in order to point the direction between LS1 and 6
    towards the light."""
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
        print(
            f"Fraction = {fraction}, commanded rotation rate is {FINE_ROTATION_RATE - fraction * 2 * FINE_ROTATION_RATE} dps"
        )
        return FINE_ROTATION_RATE - fraction * 2 * FINE_ROTATION_RATE


last_pid_call = ticks_us()
start = last_pid_call
last_recording = last_pid_call
pid.setpoint = 0
while True:  # main control loop
    if rp2.bootsel_button():
        print("exiting")
        set_speed(0)
        while True:
            sleep_ms(1000)

    current_rotation_rate = read_imu_gyro_z()
    if abs(pid.setpoint - current_rotation_rate) <= 0.5:  # indicate if the setpoint has been reached or not
        led.value(1)
    else:
        led.value(0)
    z_gyro_buffer.append(current_rotation_rate)

    current_time = ticks_us()

    time_since_last_pid_call_ms = ticks_diff(current_time, last_pid_call)
    if DETUMBLING_MODE:
        if time_since_last_pid_call_ms >= 100_000:  # run PID controller at 10 Hz
            last_pid_call = current_time
            averaged_input = sum(z_gyro_buffer) / BUFFER_LENGTH  # average the gyro measurements to reduce noise
            print(f"\nCalling PID with rotation rate of {averaged_input:+.4f} dps")
            diff = pid.get_control_output(averaged_input)  # call PID
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
        if time_since_last_pid_call_ms >= 100_000:  # run PID controller at 10 Hz
            last_pid_call = current_time
            averaged_input = sum(z_gyro_buffer) / BUFFER_LENGTH
            pid.setpoint = get_desired_setpoint(
                [
                    sum(light_sensor_1_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                    sum(light_sensor_2_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                    sum(light_sensor_3_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                    sum(light_sensor_4_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                    sum(light_sensor_5_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                    sum(light_sensor_6_buffer) / LIGHT_SENSOR_BUFFER_LENGTH,
                ]
            )
            diff = pid.get_control_output(averaged_input)
            current_speed = nominal_speed + diff
            set_speed(current_speed)
