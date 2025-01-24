from machine import Pin, UART
from time import sleep_ms


led = Pin("LED", Pin.OUT)


uart_0 = UART(0)
uart_0.init(tx=Pin(0), rx=Pin(1))

uart_1 = UART(1)
uart_1.init(tx=Pin(8), rx=Pin(9))

msg = b"\x10"

uart_0.write(msg)

while not uart_0.txdone() or not uart_1.any():
    pass

value = uart_1.read()[0]
sleep_ms(200)
for _ in range(value):
    led.value(1)
    sleep_ms(50)
    led.value(0)
    sleep_ms(50)
