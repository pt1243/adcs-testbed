from machine import ADC, Pin, UART
from time import sleep_ms, sleep_us
import rp2

# set up LED
led = Pin("LED", Pin.OUT)
led.value(0)

# set up ADC for the light sensors
light_sensor_4 = ADC(Pin(26))
light_sensor_5 = ADC(Pin(27))
light_sensor_6 = ADC(Pin(28))

# initialise UART controller
uart = UART(0)
uart.init(tx=Pin(16), rx=Pin(17))

# create a values buffer for the light sensor values that are being sent to the other chip
values_buffer = bytearray(6)

# startup sequence: if the button is pressed, communicate this; otherwise, if the primary button is pressed, exit this
# loop and proceed to the main loop
while True:
    if rp2.bootsel_button():
        uart.write(b"\xff")
        while not uart.txdone():
            sleep_us(200)
        break
    if uart.any() and uart.read() == b"\x00":
        break
    sleep_ms(5)

uart.read()  # clear any remaining reads
while True:  # main control loop
    if uart.any() and uart.read() == b"\xff":  # light sensor values are requested
        uart.write(values_buffer)
    else:
        # encode 16-bit light sensor values to bytes and store these in the buffer
        light_sensor_4_value = light_sensor_4.read_u16()
        light_sensor_5_value = light_sensor_5.read_u16()
        light_sensor_6_value = light_sensor_6.read_u16()
        values_buffer[0] = light_sensor_4_value >> 8
        values_buffer[1] = light_sensor_4_value & 255
        values_buffer[2] = light_sensor_5_value >> 8
        values_buffer[3] = light_sensor_5_value & 255
        values_buffer[4] = light_sensor_6_value >> 8
        values_buffer[5] = light_sensor_6_value & 255
