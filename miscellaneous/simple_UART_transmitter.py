from machine import Pin, UART
from time import sleep_ms


led = Pin("LED", Pin.OUT)


uart = UART(0)
uart.init(tx=Pin(0), rx=Pin(1))

msg = 10

if isinstance(msg, int):
    value = bytes([msg])
else:
    value = bytes(msg)

uart.write(value)

while not uart.txdone():
    sleep_ms(25)