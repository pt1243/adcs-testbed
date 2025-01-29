from time import sleep_ms
import rp2


while True:
    print(rp2.bootsel_button())
    sleep_ms(100)