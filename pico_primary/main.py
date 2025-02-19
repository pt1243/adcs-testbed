from micropython import const
from machine import ADC, I2C, Pin, PWM, UART
from time import sleep, sleep_ms

led = Pin("LED", Pin.OUT)


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
    i2c.writeto_mem(IMU, REG_CONFIG, bytes([0b0_0_000_110]))
    i2c.writeto_mem(IMU, REG_GYRO_CONFIG, bytes([0b000_00_000]))
    i2c.writeto_mem(IMU, REG_ACCEL_CONFIG, bytes([0b000_00_000]))
    i2c.writeto_mem(IMU, REG_ACCEL_CONFIG_2, bytes([0b00_11_1_000]))
    i2c.writeto_mem(IMU, REG_LP_MODE_CFG, bytes([0b1_111_0000]))


setup_IMU()

# these are floats so unfortunately no const() optimisations
GYRO_FACTOR = 250 / 32768  # LSB to degrees per second
ACCEL_FACTOR = 2000 / 32768  # LSB to milli-g


def read_16_bit_signed(value: bytes) -> int:
    number = 256 * value[0] + value[1]
    if number >= 32768:
        return number - 65536
    return number


def read_imu() -> tuple[float, float, float, float, float, float]:
    # Registers needed for reading the IMU values
    REG_ACCEL_XOUT = const(0x3B)
    REG_ACCEL_YOUT = const(0x3D)
    REG_ACCEL_ZOUT = const(0x3F)
    REG_GYRO_XOUT = const(0x43)
    REG_GYRO_YOUT = const(0x45)
    REG_GYRO_ZOUT = const(0x47)

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


while True:
    sleep_ms(1000)
