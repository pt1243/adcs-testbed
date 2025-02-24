from machine import ADC, UART, Pin
from time import sleep_us, sleep_ms, ticks_us, ticks_diff

light_sensor_1 = ADC(Pin(28))
light_sensor_2 = ADC(Pin(27))
light_sensor_3 = ADC(Pin(26))

uart = UART(0)
uart.init(tx=Pin(12), rx=Pin(13))

light_sensor_buffer = bytearray(6)

def read_light_sensors_from_secondary() -> tuple[int, int, int]:
    uart.write(b"\xff")
    # TODO: figure out how long to wait
    while not uart.any():
        sleep_ms(1)
    sleep_ms(10)
    uart.readinto(light_sensor_buffer)
    return (
        (light_sensor_buffer[0] << 8) + light_sensor_buffer[1],
        (light_sensor_buffer[2] << 8) + light_sensor_buffer[3],
        (light_sensor_buffer[4] << 8) + light_sensor_buffer[5],
    )


# last_reading = ticks_us()
while True:
    # now = ticks_us()
    # dt_us = ticks_diff(now, last_reading)
    # print(dt_us)
    # sleep_us(max(0, 1_000_000 - dt_us))
    
    # last_reading = now
    light_sensor_1_value = light_sensor_1.read_u16()
    light_sensor_2_value = light_sensor_2.read_u16()
    light_sensor_3_value = light_sensor_3.read_u16()

    light_sensor_4_value, light_sensor_5_value, light_sensor_6_value = read_light_sensors_from_secondary()
    print("Light sensor readings:")
    print(f"Light sensor 1: {light_sensor_1_value}")
    print(f"Light sensor 2: {light_sensor_2_value}")
    print(f"Light sensor 3: {light_sensor_3_value}")
    print(f"Light sensor 4: {light_sensor_4_value}")
    print(f"Light sensor 5: {light_sensor_5_value}")
    print(f"Light sensor 6: {light_sensor_6_value}")
    print()

    sleep_ms(400)
