from machine import ADC, I2C, Pin, PWM, UART
from time import sleep, sleep_ms

led = Pin("LED", Pin.OUT)

light_sensor_0 = ADC(Pin(26))
light_sensor_1 = ADC(Pin(27))
light_sensor_2 = ADC(Pin(28))

uart = UART(0)
uart.init(tx=Pin(16), rx=Pin(17))

while True:
    led.value(not led.value())
    sleep(0.1)
