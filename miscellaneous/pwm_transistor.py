from machine import Pin, PWM
from time import sleep_ms

led = Pin("LED", Pin.OUT)
led.value(0)

control = Pin(0, Pin.OUT, pull=None)

pwm = PWM(control, freq=50, duty_u16=65535 - 1600)

while True:
    sleep_ms(100)

# while True:
#     # high value
#     # control.value(0)
#     # led.value(1)
#     # sleep_ms(2000)

#     # # low value
#     # control.value(1)
#     # led.value(0)
#     # sleep_ms(2000)

#     sleep_ms(100)