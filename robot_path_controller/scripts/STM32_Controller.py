from flood_fill.msg import Command
import rospy
import serial
import threading
import time
from std_msgs.msg import String 

port_name = "/dev/ttyUSB0"  # Replace with your port
baud_rate = 57600
rospy.init_node('STM32_Controller',anonymous = True)
STM32_Message_Publisher = rospy.Publisher("STM32_Message",String,queue_size=50)

def on_data_received(data):
    """Callback function to handle incoming serial data."""
    global STM32_Message_Publisher
    print(f"Receive Data : {data}")
    STM32_Message_Publisher.publish(data)

def serial_listener(serial_port:serial.Serial, callback):
    """Thread function to listen for data on the serial port."""
    while True:
        if serial_port.in_waiting > 0:
            # Read the data from the serial port
            data = serial_port.readline().decode("utf-8").strip()
            # Call the callback function with the received data
            if data == "Start":
                serial_port.write("Okay\0")
            callback(data)

def Command_Handler(Command):
    global ser
    Data = ""
    Command_Type = Command.type
    Command_Value = Command.value
    if (Command_Type == "Foward" or Command_Type == "Backward"):
        Data = f"{Command_Type}\0"
    elif (Command_Type == "Rotate-Right" or Command_Type == "Rotate-Left"):
        Data = f"{Command_Type}/{Command_Value}\0"
    else:
        pass
    ser.write(Data)

def Ros_Subcribe():
    rospy.Subscriber("Commander",Command, Command_Handler)
    rospy.spin()


try:# Open the serial port
    ser = serial.Serial(port_name, baud_rate, timeout=1)
    print(f"Connected to {port_name}")
    
    # Start the listener thread
    listener_thread = threading.Thread(target=serial_listener, args=(ser, on_data_received), daemon=True)
    listener_thread.start()
    Ros_Subcribe()
    while True:
        pass


except serial.SerialException as e:
    print(f"Serial error: {e}")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print("Serial port closed.")




