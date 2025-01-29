from machine import Pin
from time import sleep_ms

led = Pin("LED", Pin.OUT)
led.value(0)

control = Pin(0, Pin.OUT, pull=None)



while True:
    # high value
    control.value(0)
    led.value(1)
    sleep_ms(2000)

    # low value
    control.value(1)
    led.value(0)
    sleep_ms(2000)