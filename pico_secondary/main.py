from micropython import const
from machine import ADC, Pin, UART
from time import ticks_us, ticks_diff

led = Pin("LED", Pin.OUT)
led.value(0)

light_sensor_4 = ADC(Pin(26))
light_sensor_5 = ADC(Pin(27))
light_sensor_6 = ADC(Pin(28))

uart = UART(0)
uart.init(tx=Pin(16), rx=Pin(17))

values_buffer = bytearray(6)

while True:
    # led.value(not led.value())
    # sleep(0.1)
    # u16 over UART -> bytes([num >> 8, num & 255])
    # ...

    if uart.any() and uart.read() == b"\xff":
        uart.write(values_buffer)
    else:
        # start = ticks_us()
        light_sensor_4_value = light_sensor_4.read_u16()
        light_sensor_5_value = light_sensor_5.read_u16()
        light_sensor_6_value = light_sensor_6.read_u16()
        values_buffer[0] = light_sensor_4_value >> 8
        values_buffer[1] = light_sensor_4_value & 255
        values_buffer[2] = light_sensor_5_value >> 8
        values_buffer[3] = light_sensor_5_value & 255
        values_buffer[4] = light_sensor_6_value >> 8
        values_buffer[5] = light_sensor_6_value & 255
        # end = ticks_us()
