#!/usr/bin/env python
from sensor_msgs.msg import LaserScan


class Lidar():
    def __init__(self):
        self.Valid_Value = False
        self.Angle_Increment = 0.47368  # degrees
        self.Head_Distance = 10
        self.Head_Count_Check = 0
        self.Last_Head_Distance = 0

        self.Right_Distance = 10
        self.Right_Count_Check = 0
        self.Last_Right_Distance = 0
        
        self.Back_Distance = 10
        self.Back_Count_Check = 0
        self.Last_Back_Distance = 0

        self.Left_Distance = 10
        self.Left_Count_Check = 0
        self.Last_Left_Distance = 0

    def Get_Distances(self):
        return (self.Head_Distance ,self.Right_Distance ,self.Back_Distance ,self.Left_Distance )
    
    def Convert_To_Sides_Distance(self,msg:LaserScan):
        Length_Message = len(msg.ranges)
        Head_Index = Length_Message // 2
        Right_Index = Length_Message // 4
        Back_Index = 0
        Left_Index = (Length_Message*3) //4
        Index_Parameter = 60
        Head_Distance = 10
        Right_Distance = 10
        Back_Distance = 10
        Left_Distance =10
        
        for Index in range(Head_Index-Index_Parameter,Head_Index + Index_Parameter + 10):
            if msg.ranges[Index] < Head_Distance:
                Head_Distance = msg.ranges[Index]

        for Index in range(Right_Index -Index_Parameter, Right_Index + Index_Parameter + 10):
            if msg.ranges[Index] < Right_Distance:
                Right_Distance = msg.ranges[Index]

        for Index in range(Left_Index -Index_Parameter, Left_Index + Index_Parameter):
            if msg.ranges[Index] < Left_Distance:
                Left_Distance = msg.ranges[Index]
        
        for Index in range(Back_Index, Back_Index + Index_Parameter):
            if msg.ranges[Index] < Back_Distance:
                Back_Distance = msg.ranges[Index]

        for Index in range(Length_Message-Index_Parameter, Length_Message):
            if msg.ranges[Index] < Back_Distance:
                Back_Distance = msg.ranges[Index]
        
        if abs(Head_Distance - self.Last_Head_Distance) <= 0.15:
            self.Head_Count_Check += 1
            if self.Head_Count_Check == 2:
                self.Valid_Value = True
                self.Head_Distance = Head_Distance
                self.Head_Count_Check = 0
        else:
            self.Head_Count_Check = 0
        self.Last_Head_Distance = Head_Distance

        if abs(Right_Distance - self.Last_Right_Distance) <= 0.15:
            self.Right_Count_Check += 1
            if self.Right_Count_Check == 2:
                self.Right_Distance = Right_Distance
                self.Right_Count_Check = 0
        else:
            self.Right_Count_Check = 0
        self.Right_Distance = Right_Distance

        if abs(Back_Distance - self.Last_Back_Distance) <= 0.15:
            self.Back_Count_Check += 1
            if self.Back_Count_Check == 2:
                self.Back_Distance = Back_Distance
                self.Back_Count_Check = 0
        else:
            self.Back_Count_Check = 0
        self.Back_Distance = Back_Distance

        if abs(Left_Distance - self.Last_Left_Distance) <= 0.15:
            self.Left_Count_Check += 1
            if self.Left_Count_Check == 2:
                self.Left_Distance = Left_Distance
                self.Left_Count_Check = 0
        else:
            self.Left_Count_Check = 0
        self.Last_Left_Distance = Left_Distance