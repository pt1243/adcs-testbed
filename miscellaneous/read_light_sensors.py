from machine import ADC, UART, Pin
from time import sleep_us, sleep_ms, ticks_us, ticks_diff
from collections import deque

light_sensor_1 = ADC(Pin(28))
light_sensor_2 = ADC(Pin(27))
light_sensor_3 = ADC(Pin(26))

uart = UART(0)
uart.init(tx=Pin(12), rx=Pin(13))

light_sensor_buffer = bytearray(6)

LIGHT_SENSOR_BUFFER_LENGTH = 100
light_sensor_1_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_2_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_3_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_4_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_5_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)
light_sensor_6_buffer = deque([0 for _ in range(LIGHT_SENSOR_BUFFER_LENGTH)], LIGHT_SENSOR_BUFFER_LENGTH)

def read_light_sensors_from_secondary() -> tuple[int, int, int]:
    uart.write(b"\xff")
    # TODO: figure out how long to wait
    while not uart.any():
        sleep_ms(1)
    sleep_ms(10)
    bytes_read = uart.readinto(light_sensor_buffer)
    return (
        (light_sensor_buffer[0] << 8) + light_sensor_buffer[1],
        (light_sensor_buffer[2] << 8) + light_sensor_buffer[3],
        (light_sensor_buffer[4] << 8) + light_sensor_buffer[5],
    )


last_reading = ticks_us()
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

    light_sensor_1_buffer.append(light_sensor_1_value)
    light_sensor_2_buffer.append(light_sensor_2_value)
    light_sensor_3_buffer.append(light_sensor_3_value)
    light_sensor_4_buffer.append(light_sensor_4_value)
    light_sensor_5_buffer.append(light_sensor_5_value)
    light_sensor_6_buffer.append(light_sensor_6_value)

    now = ticks_us()
    if ticks_diff(now, last_reading) / 1000 >= 400:
        last_reading = now
        print("Light sensor readings:")
        print(f"Light sensor 1: {sum(light_sensor_1_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print(f"Light sensor 2: {sum(light_sensor_2_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print(f"Light sensor 3: {sum(light_sensor_3_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print(f"Light sensor 4: {sum(light_sensor_4_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print(f"Light sensor 5: {sum(light_sensor_5_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print(f"Light sensor 6: {sum(light_sensor_6_buffer) / LIGHT_SENSOR_BUFFER_LENGTH:.0f}")
        print()
