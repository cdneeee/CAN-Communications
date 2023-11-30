import serial
import time

'''# Open serial port
ser = serial.Serial('/dev/ttyUSB0'(for Raspbian), 9600)#serial port (may differ), baud rate
time.sleep(2)  # Wait for the serial connection to initialize

# Write to the serial port
ser.write(b'Hello, ESP32!\n')

# Read from the serial port
data = ser.readline()
print(f"Received: {data}")

# Close the serial port
ser.close()'''
#real time communication(usb)
# Set up the serial connection (adjust the port name and baud rate)
ser = serial.Serial('COM5', 9600)
print("Started")
time.sleep(2)  # Wait for the connection to initialize

try:
    while True:
        if ser.in_waiting:
            data = ser.readline().decode('utf-8').rstrip()  # Read data from serial
            print(f"Data received: {data}")
        time.sleep(0.1)  # Short delay to prevent CPU overuse
except KeyboardInterrupt:
    print("Serial reading stopped.")
finally:
    ser.close()