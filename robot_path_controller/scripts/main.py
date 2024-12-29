#!/usr/bin/env
import sys
import os
Script_Folder = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0,Script_Folder)

import rospy
from std_msgs.msg import String 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from robot_path_controller.msg import Command

from Path_Planning import Dijkstra
from Lidar import Lidar
from Occupied_Map import Penalty_Map
from Robot_State import Position
from Robot_Controller import Controller
import time

class Main():
    def __init__(self) -> None:
        self.Command_Message = Command()
        self.Publisher = rospy.Publisher("Commander",Command,queue_size=50)
        self.Algorithm_Controller = Controller(Penalty_Map=Penalty_Map(),Robot_Lidar= Lidar(),Robot_Position=Position(),
                                        Path_Planning=Dijkstra(Number_X_Axis=Penalty_Map().Get_Number_X_Axis_In_Map(),Number_Y_Axis=Penalty_Map().Get_Number_X_Axis_In_Map()))

        self.List_Commands = []
        self.Flag_Map = False
        self.Flag_Position = False
        self.Is_Movement = False

    def __Update_Position(self,Type:str):
        if Type == "Passed":
            Now_Position = self.Algorithm_Controller.Robot_Position.Get_Now_Position()
            self.Algorithm_Controller.Robot_Position.Update_Passed_Position(Now_Position)
            Passed_Positions = self.Algorithm_Controller.Robot_Position.Get_Passed_Positions()
            self.Algorithm_Controller.Penalty_Map.Calcutate_Penalty_Map_With_Passed_Positions(Passed_Positions)
        elif Type == "Occupied":
            Occupied_Positions = self.Algorithm_Controller.Robot_Position.Get_Occupied_Positions()
            self.Algorithm_Controller.Penalty_Map.Calculate_Penalty_Map_With_With_Occupied_Positions(Occupied_Positions)
        else:
            pass

    def __On_Task_Is_Done(self):
        self.Is_Movement = False
        if len(self.List_Commands) > 0:
            if "Rotate" in self.List_Commands[0]["Type"] and self.List_Commands[0]["Value"] > 30:
                rospy.sleep(0.5)
        else:
            pass

        if len(self.List_Commands) == 1:
            self.Algorithm_Controller.Robot_Position.Update_Now_Angle(Angle=self.Algorithm_Controller.Robot_Position.Get_Target_Angle())
        else:
            pass

        if len(self.List_Commands) > 0:
            del self.List_Commands[0]
        else:
            pass

    def __Command_Robot(self):
        if len(self.List_Commands) ==0:
            self.List_Commands = self.Algorithm_Controller.Fix_Error_Degreed()
            if len(self.List_Commands) == 0:
                rospy.sleep(0.2)
                self.__Update_Position(Type="Passed")
                self.List_Commands = self.Algorithm_Controller.Get_List_Command_Robot()  
            else:
                pass
        else: 
            pass
        if self.List_Commands[0]["Type"] == "Finish":
            print("Finish")
        else:
            self.__Send_Command_To_Robot(Command = self.List_Commands[0])

    def __Send_Command_To_Robot(self, Command):
        if "Rotate" in Command["Type"] or Command["Type"] == "Forward":
            self.Is_Movement = True 
        self.Command_Message.type = Command["Type"]
        self.Command_Message.value = Command["Value"]
        self.Publisher.publish(self.Command_Message)

    def __Robot_Backward(self,Distance):
        self.Command_Message.type = "Backward"
        self.Command_Message.value = Distance
        self.Publisher.publish(self.Command_Message)

    def __Stop_Robot(self):
        self.Command_Message.type = "Stop"
        self.Command_Message.value = 0
        self.Publisher.publish(self.Command_Message)
        print("Stop Robot Urgency")
    
    def Object_Detected(self,Source = "Lidar"):
        self.Is_Movement = False
        self.Algorithm_Controller.Robot_Lidar.Valid_Value = False
        del self.List_Commands[:]
        self.__Stop_Robot()
        time.sleep(2)
        Distances = self.Algorithm_Controller.Robot_Lidar.Get_Distances()
        if Distances[0] < 0.25 or Source == "Ultrasonic":
            Target_Position = self.Algorithm_Controller.Robot_Position.Get_Target_Position()
            Now_Position = self.Algorithm_Controller.Robot_Position.Get_Now_Position()
            if Now_Position != Target_Position:
                Occupied_Postion = Target_Position
            else:
                Occupied_Postion = self.Algorithm_Controller.Robot_Position.Get_Ahead_Position()
                self.__Update_Position(Type="Passed")
            print(f"Occupied Position : {Occupied_Postion}")
            self.Algorithm_Controller.Robot_Position.Update_Occupied_Position(Position=Occupied_Postion)
            self.__Update_Position(Type="Occupied")
        self.__Robot_Backward(Distance = 20)
        time.sleep(1)


    def Map_Callback_Handler(self,data):
        self.Flag_Map = True
        Passed_Positions = self.Algorithm_Controller.Robot_Position.Get_Passed_Positions()
        Occupied_Positions = self.Algorithm_Controller.Robot_Position.Get_Occupied_Positions()
        self.Algorithm_Controller.Penalty_Map.Convert_Occupancy_Map_To_Penalty_Map(data)
        self.Algorithm_Controller.Penalty_Map.Calcutate_Penalty_Map_With_Passed_Positions(Passed_Positions=Passed_Positions)
        self.Algorithm_Controller.Penalty_Map.Calculate_Penalty_Map_With_With_Occupied_Positions(Occupied_Positions)

    def Position_Callback_Handler(self,msg:PoseStamped):
        self.Flag_Position = True
        SLAM_Position_X = msg.pose.position.x
        SLAM_Position_Y = msg.pose.position.y
        Angle_Z = msg.pose.orientation.z
        Angle_W = msg.pose.orientation.w

        Now_Position,Is_Checked = self.Algorithm_Controller.Robot_Position.Determine_Now_Position(SLAM_Pose=(SLAM_Position_X,SLAM_Position_Y))
        SLAM_Now_Angle = self.Algorithm_Controller.Robot_Position.Determine_SLAM_Now_Angle(Angle_Z=Angle_Z,Angle_W=Angle_W)

        if Is_Checked:
            self.Algorithm_Controller.Robot_Position.Update_SLAM_Now_Pose(SLAM_Pose=(SLAM_Position_X,SLAM_Position_Y))
        self.Algorithm_Controller.Robot_Position.Update_Now_Position(Position=Now_Position)
        self.Algorithm_Controller.Robot_Position.Update_SLAM_Now_Angle(Degrees_Value=SLAM_Now_Angle)

    def Lidar_Callback_Handler(self,msg:LaserScan):
        self.Algorithm_Controller.Robot_Lidar.Convert_To_Sides_Distance(msg=msg)
        if self.Is_Movement == True and self.Algorithm_Controller.Robot_Lidar.Valid_Value == True:
            Distances = self.Algorithm_Controller.Robot_Lidar.Get_Distances()       #=>(Head , Right , Back, Left)
            if Distances[0] <= 0.25:
                self.Object_Detected(Source= "Lidar")


        
    def STM32_Message_Callback_Handler(self,Message):
        if Message.data == "Start":
            while (self.Flag_Map == False or self.Flag_Position == False):
                pass                                                ## Waiting for setup finish
        elif Message.data == "Movement_Okay" or Message.data == "Rotation_Okay":
            self.__On_Task_Is_Done()
        elif Message.data == "Object_Detected":
            self.Object_Detected(Source = "Ultrasonic")
        else:   
            pass # Reserve 

        self.__Command_Robot()
        
    def Node_subscribe(self):
        rospy.init_node('main',anonymous = True)
        rospy.Subscriber("map",OccupancyGrid, self.Map_Callback_Handler)
        rospy.Subscriber('scan', LaserScan, self.Lidar_Callback_Handler)
        rospy.Subscriber("slam_out_pose",PoseStamped, self.Position_Callback_Handler)
        rospy.Subscriber("STM32_Message",String, self.STM32_Message_Callback_Handler)
        print("Controller Started")
        rospy.spin()



if __name__ == '__main__':
    print("Initialize Controller")
    Main_Controller = Main()
    try:
        Main_Controller.Node_subscribe()
    except rospy.ROSInterruptException:
        print("Cannot subscribe topic map")
        pass

    
    # rospy.init_node("Determine_Path")
    # # Path_Publish = rospy.Publisher("path",)
    # Path_Finding_Rate = rospy.Rate(1)

