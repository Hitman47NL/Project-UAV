#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed May  1 16:33:22 2024

@author: iwan
"""

import serial
import time

# Open the serial port that your Raspberry Pi is connected to.
ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Adjust '/dev/ttyUSB0' to match your configuration

def modify_coordinates(data):
    # Dummy function to modify coordinates
    return "20,2"

try:
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode().strip()  # Read data from Arduino
            print(f"Received coordinates: {data}")
            if data == "0,0":
                modified_data = modify_coordinates(data)
                ser.write((modified_data + '\n').encode())  # Send back modified coordinates
                print(f"Sent modified coordinates: {modified_data}")
            time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("Closed connection.")
