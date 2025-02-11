from machine import Pin, ADC
from time import sleep


A0 = ADC(Pin(26))
A1 = ADC(Pin(27))
A2 = ADC(Pin(28))

REF_VOLTAGE = 3.3
conversion_factor = REF_VOLTAGE / 65535  # measurement is only 12-bit, but micropython reads it as a 16-bit int


while True:
    # read the three analogue inputs, and convert to a voltage
    # a0_value = A0.read_u16() * conversion_factor
    # a1_value = A1.read_u16() * conversion_factor
    # a2_value = A2.read_u16() # * conversion_factor

    # print(a2_value)

    # print(f"A0: {a0_value:.6f}, A1: {a1_value:.6f}, A2: {a2_value:.6f}")

    print()
    sleep(0.2)
