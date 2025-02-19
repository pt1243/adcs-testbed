from machine import Pin, PWM
import rp2
from time import sleep_ms

led = Pin("LED", Pin.OUT)
led.value(0)

control = Pin(0, Pin.OUT, pull=None)


### 1630 for 0.5 ms
### 3260 for 1.0 ms
### 6540 for 2.0 ms
### 8200 for 2.5 ms

def set_speed(pwm: PWM, speed: float):
    speed = min(1, max(0, speed))  # clip to [0, 1]
    pwm.duty_u16(65535 - int(3260 * (1 - speed) + 6540 * speed))

pwm = PWM(control, freq=50)

set_speed(pwm, 0)
current_speed = 0

while True:
    if rp2.bootsel_button():
        current_speed += 0.0025
        set_speed(pwm, current_speed)
        print(f"Current speed is now {current_speed:.4f}")
        sleep_ms(200)
    sleep_ms(10)
