from machine import Pin, UART
from time import sleep_ms


led = Pin("LED", Pin.OUT)

uart = UART(0)
uart.init(tx=Pin(0), rx=Pin(1))

# indicate successful startup:
led.value(1)
sleep_ms(1000)
led.value(0)

while True:
    while not uart.any():
        sleep_ms(50)

    value = uart.read()
    sleep_ms(200)
    if len(value) > 1:
        # indicate error
        for _ in range(3):
            led.value(1)
            sleep_ms(400)
            led.value(0)
            sleep_ms(200)
    else:
        for _ in range(value[0]):
            led.value(1)
            sleep_ms(100)
            led.value(0)
            sleep_ms(100)
