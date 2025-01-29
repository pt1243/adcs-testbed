from machine import Pin, PWM
import rp2
from time import sleep_ms

led = Pin("LED", Pin.OUT)
led.value(0)

control = Pin(0, Pin.OUT, pull=None)


### 1630 for 0.5 ms
### 8200 for 2.5 ms

def set_speed(pwm: PWM, speed: float):
    speed = min(1, max(0, speed))  # clip to [0, 1]
    pwm.duty_u16(65535 - int(1630 * (1 - speed) + 8200 * speed))

pwm = PWM(control, freq=50)


set_speed(pwm, 0.75)
print("setting max speed")
while True:
    if rp2.bootsel_button():
        break
    sleep_ms(50)

set_speed(pwm, 0.25)
print("setting min speed")

sleep_ms(1000)

print("press button to go to 5% throttle...")
while True:
    if rp2.bootsel_button():
        break
    sleep_ms(50)

set_speed(pwm, 0.30)

while True:
    sleep_ms(1000)

# set_speed(pwm, 0)
# sleep_ms(500)
# set_speed(pwm, 1)

# while True:
#     # sleep_ms(100)
#     for i in range(11):
#         set_speed(pwm, 0.1 * i)
#         sleep_ms(500)
