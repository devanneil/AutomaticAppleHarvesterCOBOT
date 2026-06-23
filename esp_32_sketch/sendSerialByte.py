import serial

ser = serial.Serial('COM3', 115200)

HEADER = 0xAA

def set_relays(state):
    packet = bytes([HEADER, state])
    ser.write(packet)

# Example usage:
set_relays(0b1000)  # relay 3 ON
set_relays(0b0011)  # relay 0 and 1 ON