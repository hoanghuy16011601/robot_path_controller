import serial
import threading
import time
def on_data_received(data):
    """Callback function to handle incoming serial data."""
    print(f"Callback: Received data -> {data}")

def serial_listener(serial_port, callback):
    """Thread function to listen for data on the serial port."""
    while True:
        if serial_port.in_waiting > 0:
            # Read the data from the serial port
            data = serial_port.readline().decode("utf-8").strip()
            # Call the callback function with the received data
            callback(data)

# Serial port configuration
port_name = "/dev/ttyUSB0"  # Replace with your port
baud_rate = 57600

try:
    # Open the serial port
    ser = serial.Serial(port_name, baud_rate, timeout=1)
    print(f"Connected to {port_name}")
    
    # Start the listener thread
    listener_thread = threading.Thread(target=serial_listener, args=(ser, on_data_received), daemon=True)
    listener_thread.start()
    
    # Main thread can do other work or wait
    message = "Hoang Huy\0"
    while True:
        time.sleep(2)
        ser.write(message.encode("utf-8"))

except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print("Serial port closed.")
