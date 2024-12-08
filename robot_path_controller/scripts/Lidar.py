#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import time

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
        Left_Index = Length_Message*3 //4
        Head_Distance = 0
        Right_Distance = 0
        Back_Distance = 0
        Left_Distance =0
        
        for Index in range(Head_Index-30,Head_Index + 31):
            Head_Distance += msg.ranges[Index]
        
        for Index in range(Right_Index -30, Right_Index + 31):
            Left_Distance += msg.ranges[Index]

        for Index in range(Left_Index -30, Left_Index + 31):
            Right_Distance += msg.ranges[Index]
        
        for Index in range(Back_Index, Right_Index + 31):
            Back_Distance += msg.ranges[Index]

        for Index in range(Length_Message-30, Length_Message):
            Back_Distance += msg.ranges[Index]
        
        self.Head_Distance = Head_Distance / 71
        self.Right_Distance = Right_Distance / 71
        self.Back_Distance = Back_Distance / 71
        self.Left_Distance = Left_Distance / 71



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
        print(a)



if __name__ == '__main__':
    Controller()
