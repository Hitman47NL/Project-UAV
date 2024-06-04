import serial
import time

# Setup the serial connection
ser = serial.Serial('/dev/serial0', 9600, timeout=1)  # Set timeout to prevent blocking

def read_coordinates():
    """Read string coordinates from the serial port."""
    data = ser.readline().decode().strip()  # Read a line and decode it from bytes to string
    print("Received from Arduino:", data)
    return data

def send_coordinates(x, y):
    """Send coordinates as a formatted string."""
    coord_str = f"X={x},Y={y}\n"
    ser.write(coord_str.encode())  # Encode string to bytes and send
    print("Sent to Arduino:", coord_str)

def main():
    while True:
        data = read_coordinates()
        if data:  # Check if any data was received
            if data == "X=0,Y=0":
                send_coordinates(10, 20)  # Change coordinates
                break  # Stop after sending new coordinates
        time.sleep(1)  # Delay between checks

if __name__ == "__main__":
    main()
