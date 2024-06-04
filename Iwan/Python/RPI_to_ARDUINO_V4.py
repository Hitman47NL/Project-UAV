import serial
import time

# Constants for the default coordinates and angle
X_acuro = 20
Y_acuro = 2
Angle_acuro = 90

# Set up the serial connection (The device name might be different)
ser = serial.Serial('/dev/ttyS0', 9600, timeout=1)

def send_coordinates(x, y, angle):
    """Sends formatted coordinates to the connected device."""
    message = f"{x},{y},{angle}\n"
    ser.write(message.encode('utf-8'))
    print(f"Sent to device: {message.strip()}")

def handle_device_response():
    """Handles incoming messages from the device."""
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        if line == "Send":
            # Only send coordinates when the Arduino sends the "Send" command
            send_coordinates(X_acuro, Y_acuro, Angle_acuro)
            # Optional: Send more data or adjust based on additional Arduino responses
            time.sleep(1)  # Delay to prevent spamming; adjust as needed

# Main loop to handle communication continuously
try:
    while True:
        handle_device_response()
except KeyboardInterrupt:
    print("Program terminated by user.")
except serial.SerialException as e:
    print(f"Serial error: {e}")
finally:
    ser.close()
    print("Serial connection closed.")
