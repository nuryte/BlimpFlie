import serial
import time

# Define the serial port and baud rate. Ensure the 'COM#' corresponds to what you see in the Arduino IDE
SERIAL_PORT = 'COM9'  # You might need to adjust this (e.g., '/dev/ttyUSB0' or '/dev/ttyACM0' for Linux)
BAUD_RATE = 115200

def read_from_serial_port(serial_port):
    while True:
        if serial_port.in_waiting:
            # Read a line from the serial port
            line = serial_port.readline().decode('utf-8').strip()
            print(line)

if __name__ == '__main__':
    # Set up the serial port connection
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        # Give some time for the connection to establish
        time.sleep(2)
        read_from_serial_port(ser)