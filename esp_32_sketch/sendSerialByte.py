import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)

HEADER = 0xAA

def set_relays(state):
    packet = bytes([HEADER, state])
    ser.write(packet)

# Example usage:
try:
    while True:
        set_relays(0b0000)
        time.sleep(1)
        set_relays(0b0001)
        time.sleep(1)
finally:
    set_relays(0b0000)
