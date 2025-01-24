from machine import Pin, ADC, I2C
from time import sleep


# A0 = ADC(Pin(26))
# A1 = ADC(Pin(27))
# A2 = ADC(Pin(28))

# REF_VOLTAGE = 3.3
# conversion_factor = REF_VOLTAGE / 65535  # measurement is only 12-bit, but micropython reads it as a 16-bit int

# IMU
IMU = 0x69  # I2C bus address for the IMU
GYRO_SENSITIVITY = 0  # 0 through 3 correspond to +/- 250, 500, 1000, or 2000 deg per second respectively
ACCEL_SENSITIVITY = 0  # 0 through 3 correspond to +/- 2, 4, 8, or 16 g respectively

# Registers
REG_ACCEL_CONFIG = 0x1C
REG_ACCEL_XOUT = 0x3B
REG_ACCEL_YOUT = 0x3D
REG_ACCEL_ZOUT = 0x3F

REG_GYRO_CONFIG = 0x1B
REG_GYRO_XOUT = 0x43
REG_GYRO_YOUT = 0x45
REG_GYRO_ZOUT = 0x47

REG_PWR_MGMT_1 = 0x6B


# set up I2C
i2c = I2C(0, sda=Pin(4), scl=Pin(5))
if IMU not in i2c.scan():
    print("Error: IMU not detected on I2C bus!")
    exit()

# disable sleep mode
i2c.writeto_mem(IMU, REG_PWR_MGMT_1, b"\x01")

# set sensitivity values
# TODO: investigate low-noise mode for built-in averaging
# i2c.writeto_mem(IMU, REG_ACCEL_CONFIG, ACCEL_SENSITIVITY << 3)
# i2c.writeto_mem(IMU, REG_GYRO_CONFIG, GYRO_SENSITIVITY << 3)


def read_16_bit_signed(value):
    number = 256 * value[0] + value[1]
    if number >= 32768:
        return ~(number - 32768) + 1
    return number


while True:
    # read the three analogue inputs, and convert to a voltage
    # a0_value = A0.read_u16() * conversion_factor
    # a1_value = A1.read_u16() * conversion_factor
    # a2_value = A2.read_u16() # * conversion_factor

    # print(a2_value)

    # print(f"A0: {a0_value:.6f}, A1: {a1_value:.6f}, A2: {a2_value:.6f}")

    x_accel = i2c.readfrom_mem(IMU, REG_ACCEL_XOUT, 2)
    y_accel = i2c.readfrom_mem(IMU, REG_ACCEL_YOUT, 2)
    z_accel = i2c.readfrom_mem(IMU, REG_ACCEL_ZOUT, 2)
    # print(x, y, z)

    print("Accelerometer:")

    x_str = bin(256 * x_accel[0] + x_accel[1])[2:]
    y_str = bin(256 * y_accel[0] + y_accel[1])[2:]
    z_str = bin(256 * z_accel[0] + z_accel[1])[2:]
    print((16 - len(x_str)) * "0" + x_str, (16 - len(y_str)) * "0" + y_str, (16 - len(z_str)) * "0" + z_str)

    print(read_16_bit_signed(x_accel), read_16_bit_signed(y_accel), read_16_bit_signed(z_accel))

    x_gyro = i2c.readfrom_mem(IMU, REG_GYRO_XOUT, 2)
    y_gyro = i2c.readfrom_mem(IMU, REG_GYRO_YOUT, 2)
    z_gyro = i2c.readfrom_mem(IMU, REG_GYRO_ZOUT, 2)

    print("Gyro:")
    print(read_16_bit_signed(x_gyro), read_16_bit_signed(y_gyro), read_16_bit_signed(z_gyro))

    print()
    sleep(0.2)
