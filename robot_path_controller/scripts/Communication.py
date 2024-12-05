from robot_path_controller.msg import Command
import rospy
import serial
import threading
import time
from std_msgs.msg import String 


class STM32_Message_Controller():
    def __init__(self) -> None:
        rospy.init_node('STM32_Controller',anonymous = True)
        self.STM32_Message_Publisher = rospy.Publisher("STM32_Message",String,queue_size=50)

    def On_Data_Receive(self,data):
        """Callback function to handle incoming serial data."""
        print(f"Receive Data : {data}")
        self.STM32_Message_Publisher.publish(data)

    def Serial_Listener(self,serial_port:serial.Serial, callback):
        """Thread function to listen for data on the serial port."""
        while self.ThreadRunning_Flag:
            if serial_port.in_waiting > 0:
                # Read the data from the serial port
                data = serial_port.readline().decode("utf-8").strip()
                # Call the callback function with the received data
                callback(data)

    def Command_Handler(self,Command):
        Data = ""
        Command_Type = Command.type
        Command_Value = Command.value
        if (Command_Type == "Forward" or Command_Type == "Backward"):
            Data = f"{Command_Type}/{Command_Value}\0"
        elif (Command_Type == "Rotate-Right" or Command_Type == "Rotate-Left"):
            Data = f"{Command_Type}/{Command_Value}\0"
        else:
            pass
        print(Data)
        self.Serial.write(Data.encode("utf-8"))

    def Ros_Subcribe(self):
        rospy.Subscriber("Commander",Command, self.Command_Handler)
        rospy.spin()


    def Start(self):
        Port_Name = "/dev/ttyUSB1"  # Replace with your port
        BaudRate = 57600
        try:# Open the serial port
            self.Serial = serial.Serial(Port_Name, BaudRate, timeout=1)
            print(f"Connected to {Port_Name}")
            
            # Start the listener thread
            self.ThreadRunning_Flag = True
            listener_thread = threading.Thread(target=self.Serial_Listener, args=(self.Serial, self.On_Data_Receive), daemon=True)
            listener_thread.start()
            self.Ros_Subcribe()
            while True:
                pass


        except serial.SerialException as e:
            self.ThreadRunning_Flag = False
            print(f"Serial error: {e}")
        except Exception as e:
            self.ThreadRunning_Flag = False
            print(f"Error: {e}")
        finally:
            if 'ser' in locals() and self.Serial.is_open:
                self.Serial.close()
            self.ThreadRunning_Flag = False
            print("Serial port closed.")

if __name__ == "__main__":
    Controller = STM32_Message_Controller()
    Controller.Start()




