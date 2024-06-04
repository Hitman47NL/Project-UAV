import serial
import time

ser = serial.Serial('/dev/serial0', 9600)

def adjust_coordinates(coords):
    x, y = map(int, coords.split(','))
    return f"{x+1},{y+1}"

try:
    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()  # Read a line from the serial port
            print("Original coordinates:", line)
            adjusted_coords = adjust_coordinates(line)
            ser.write((adjusted_coords + "\n").encode())  # Send back adjusted coordinates
            ser.flush()  # Clear the serial buffer after sending
except KeyboardInterrupt:
    ser.close()  # Close serial port when done
