#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time
import math

class Lidar():
    def __init__(self):
        self.Angle_Increment = 0.47368  # degrees
        self.Head_Distance = 1000
        self.Back_Distance = 1000
        self.Left_Distance = 1000
        self.Right_Distance = 1000

    def Get_Distance(self):
        return (self.Head_Distance ,self.Right_Distance ,self.Back_Distance ,self.Left_Distance )
    
    def Convert_To_Sides_Distance(self,msg:LaserScan):
        Length_Message = len(msg.ranges)
        Head_Index = Length_Message // 2
        Right_Index = Length_Message // 4
        Back_Index = 0
        Left_Index = (Length_Message*3) //4
        Head_Distance = 10
        Right_Distance = 10
        Back_Distance = 10
        Left_Distance =10
        
        for Index in range(Head_Index-15,Head_Index + 15):
            if msg.ranges[Index] < Head_Distance:
                Head_Distance = msg.ranges[Index]

        for Index in range(Right_Index -15, Right_Index + 15):
            if msg.ranges[Index] < Right_Distance:
                Right_Distance = msg.ranges[Index]

        for Index in range(Left_Index -15, Left_Index + 15):
            if msg.ranges[Index] < Left_Distance:
                Left_Distance = msg.ranges[Index]
        
        for Index in range(Back_Index, Back_Index + 15):
            if msg.ranges[Index] < Back_Distance:
                Back_Distance = msg.ranges[Index]

        for Index in range(Length_Message-15, Length_Message):
            if msg.ranges[Index] < Back_Distance:
                Back_Distance = msg.ranges[Index]
        
        self.Head_Distance = Head_Distance
        self.Right_Distance = Right_Distance
        self.Back_Distance = Back_Distance
        self.Left_Distance = Left_Distance



class Controller():
    def __init__(self):
        self.Lidar_Handler = Lidar()
        rospy.init_node('scan_listener', anonymous=True)

        # Tạo subscriber lắng nghe topic /scan
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Duy trì node hoạt động
        rospy.spin()

    def scan_callback(self,msg:LaserScan):
        self.Lidar_Handler.Convert_To_Sides_Distance(msg=msg)
        (a,b,c,d) = self.Lidar_Handler.Get_Distance()
        print(f"{a} ---- {b} ---- {c} ---- {d}")



if __name__ == '__main__':
    Controller()
