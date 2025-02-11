from machine import Pin, I2C
from time import sleep

# IMU
IMU = 0x69  # I2C bus address for the IMU

# Registers
REG_ACCEL_XOUT = 0x3B
REG_ACCEL_YOUT = 0x3D
REG_ACCEL_ZOUT = 0x3F
REG_GYRO_XOUT = 0x43
REG_GYRO_YOUT = 0x45
REG_GYRO_ZOUT = 0x47

REG_CONFIG = 0x1A
REG_GYRO_CONFIG = 0x1B
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_CONFIG_2 = 0x1D
REG_LP_MODE_CFG = 0x1E
REG_PWR_MGMT_1 = 0x6B

# set up I2C
i2c = I2C(0, sda=Pin(4), scl=Pin(5))
if IMU not in i2c.scan():
    print("Error: IMU not detected on I2C bus!")
    exit()

# disable sleep mode
i2c.writeto_mem(IMU, REG_PWR_MGMT_1, b"\x01")

# set sensitivity values
# TODO: document all these values!
i2c.writeto_mem(IMU, REG_CONFIG, bytes([0b0_0_000_110]))
i2c.writeto_mem(IMU, REG_GYRO_CONFIG, bytes([0b000_00_000]))
i2c.writeto_mem(IMU, REG_ACCEL_CONFIG, bytes([0b000_00_000]))
i2c.writeto_mem(IMU, REG_ACCEL_CONFIG_2, bytes([0b00_11_1_000]))
i2c.writeto_mem(IMU, REG_LP_MODE_CFG, bytes([0b1_111_0000]))


def read_16_bit_signed(value: bytes):
    number = 256 * value[0] + value[1]
    if number >= 32768:
        return number - 65536
    return number

GYRO_FACTOR = 250 / 32768  # LSB to degrees per second
ACCEL_FACTOR = 2000 / 32768  # LSB to milli-g

while True:
    x_accel = i2c.readfrom_mem(IMU, REG_ACCEL_XOUT, 2)
    y_accel = i2c.readfrom_mem(IMU, REG_ACCEL_YOUT, 2)
    z_accel = i2c.readfrom_mem(IMU, REG_ACCEL_ZOUT, 2)

    print("Accelerometer:")
    print(f"{read_16_bit_signed(x_accel) * ACCEL_FACTOR:+.6f} {read_16_bit_signed(y_accel) * ACCEL_FACTOR:+.6f} {read_16_bit_signed(z_accel) * ACCEL_FACTOR:+.6f} milli-g")

    x_gyro = i2c.readfrom_mem(IMU, REG_GYRO_XOUT, 2)
    y_gyro = i2c.readfrom_mem(IMU, REG_GYRO_YOUT, 2)
    z_gyro = i2c.readfrom_mem(IMU, REG_GYRO_ZOUT, 2)

    print("Gyro:")
    print(f"{read_16_bit_signed(x_gyro) * GYRO_FACTOR:+.6f} {read_16_bit_signed(y_gyro) * GYRO_FACTOR:+.6f} {read_16_bit_signed(z_gyro) * GYRO_FACTOR:+.6f} deg/s")

    print()
    sleep(0.5)
