import math
class Position():
    def __init__(self) -> None:
        self.Start_Position = (12,12)
        self.Passed_Positions = [self.Start_Position]
        self.Now_Position = (self.Start_Position)
        self.Last_Position = self.Start_Position
        self.SLAM_Now_Pose = self.Convert_PenaltyMap_Position_To_SLAM_Pose(PenaltyMap_Position=self.Start_Position) 
        self.SLAM_Now_Angle = 0
        self.Target_Position = ()
        self.Scope_Position = self.Start_Position
        self.Count_Check = 0
        self.Now_Angle = 0
        self.Target_Angle = 0
        self.Occupied_Positions = []

    
    def Convert_PenaltyMap_Position_To_SLAM_Pose(self, PenaltyMap_Position:tuple):
        Position_X = PenaltyMap_Position[0]
        Position_y = PenaltyMap_Position[1]

        SLAM_Pose_X = (Position_X - 12)*0.4
        SLAM_Pose_Y = (Position_y - 12)*0.4

        return (SLAM_Pose_X,SLAM_Pose_Y)
    
    def __Convert_SLAM_Pose_To_PenaltyMap_Position(self,Raw_SLAM_Pose:tuple):
        SLAM_Pose_X = Raw_SLAM_Pose[0]
        SLAM_Pose_Y = Raw_SLAM_Pose[1]

        if SLAM_Pose_X > 0:
            Position_X = (int)((SLAM_Pose_X + 0.2)/0.4) + 12
        else:
            Position_X = (int)((SLAM_Pose_X-0.2)/0.4) + 12

        if SLAM_Pose_Y > 0:
            Position_Y = (int)((SLAM_Pose_Y + 0.2)/0.4) + 12
        else:
            Position_Y = (int)((SLAM_Pose_Y-0.2)/0.4) + 12
        
        return (Position_X,Position_Y)
    
    def __Is_Middle_Position_In_SLAM_Pose(self,SLAM_Pose, Raw_SLAM_Pose):
        Raw_Middle_Pose_X = (SLAM_Pose[0] - 12)*0.4  
        Raw_Middle_Pose_Y = (SLAM_Pose[1] - 12)*0.4
        if ((Raw_Middle_Pose_X -0.15 <= Raw_SLAM_Pose[0]) and (Raw_SLAM_Pose[0] <= Raw_Middle_Pose_X + 0.15)):
            if ((Raw_Middle_Pose_Y -0.15 <= Raw_SLAM_Pose[1]) and (Raw_SLAM_Pose[1] <= Raw_Middle_Pose_Y + 0.15)):
                Is_Middle_Position =True
            else:
                Is_Middle_Position = False
        else:
            Is_Middle_Position = False
        return Is_Middle_Position
            

    def Determine_Now_Position(self,SLAM_Pose:tuple):
        Check_Position = self.__Convert_SLAM_Pose_To_PenaltyMap_Position(Raw_SLAM_Pose=SLAM_Pose)

        if Check_Position == self.Last_Position:
            self.Count_Check +=1
        else:
            self.Count_Check == 0

        self.Last_Position = Check_Position
        
        if self.Count_Check == 3:
            self.Count_Check = 0
            Now_Position = Check_Position
            Is_Checked = True
        else:
            Now_Position = self.Now_Position
            Is_Checked = False
        return Now_Position, Is_Checked
    
    def Calculate_Distance_In_SLAM(self, From_Pose:tuple , To_Pose:tuple , Axis:str):
        if Axis == "X":
            Distance = To_Pose[0] - From_Pose[0]
        elif Axis == "Y":
            Distance = To_Pose[1] - From_Pose[1]
        else:
            Distance = 0

        return abs(Distance)

    def Determine_SLAM_Now_Angle(self,Angle_Z, Angle_W):
        Theta_Angle = 2*math.atan2(Angle_Z,Angle_W)
        Degrees_Angle = math.degrees(Theta_Angle)
        if Degrees_Angle >= 0:
            Degrees_Angle = 360 - Degrees_Angle
        else:
            Degrees_Angle = -Degrees_Angle 
        return Degrees_Angle
    
    def Get_Start_Position(self):
        return self.Start_Position
    
    def Get_Now_Position(self):
        return self.Now_Position
    
    def Update_Now_Position(self,Position):
        self.Now_Position = Position

    def Get_SLAM_Now_Pose(self):
        return self.SLAM_Now_Pose

    def Update_SLAM_Now_Pose(self,SLAM_Pose:tuple):
        self.SLAM_Now_Pose = SLAM_Pose

    def Update_SLAM_Now_Angle(self,Degrees_Value):
        self.SLAM_Now_Angle = Degrees_Value

    def Get_SLAM_Now_Angle(self):
        return self.SLAM_Now_Angle
    
    def Get_Target_Position(self):
        return self.Target_Position
    
    def Update_Target_Position(self,Position):
        self.Target_Position = Position

    def Get_Scope_Position(self):
        return self.Scope_Position
    
    def Reset_Scope_Pose(self):
        self.Scope_Position = ()
    
    def Update_Scope_Position(self,Position):
        self.Scope_Position = Position
    
    def Get_Now_Angle(self):
        return self.Now_Angle
    
    def Update_Now_Angle(self,Angle:int):
        self.Now_Angle = Angle
    
    def Get_Target_Angle(self):
        return self.Target_Angle
    
    def Update_Target_Angle(self,Angle:int):
        self.Target_Angle = Angle

    def Get_Passed_Positions(self):
        return self.Passed_Positions
    
    def Update_Passed_Position(self,Position):
        if Position not in self.Passed_Positions:
            self.Passed_Positions.append(Position)
        else:
            pass

    def Get_Occupied_Positions(self):
        return self.Occupied_Positions

    def Update_Occupied_Position(self,Position):
        if Position not in self.Occupied_Positions:
            self.Occupied_Positions.append(Position)
        else:
            pass

    def Get_Ahead_Position(self):
        if self.Now_Angle == 0:
            Direction_Bias = (1,0)
        elif self.Now_Angle == 90:
            Direction_Bias = (0,-1)
        elif self.Now_Angle == 180:
            Direction_Bias = (-1,0)
        elif self.Now_Angle == 270:
            Direction_Bias = (0,1)
        
        X = self.Now_Position[0] + Direction_Bias[0]
        Y = self.Now_Position[1] + Direction_Bias[1]
        return (X,Y)
    
    def Get_Behind_Position(self):
        if self.Now_Angle == 0:
            Direction_Bias = (1,0)
        elif self.Now_Angle == 90:
            Direction_Bias = (0,-1)
        elif self.Now_Angle == 180:
            Direction_Bias = (-1,0)
        elif self.Now_Angle == 270:
            Direction_Bias = (0,1)
        
        X = self.Now_Position[0] - Direction_Bias[0]
        Y = self.Now_Position[1] - Direction_Bias[1]
        return (X,Y)

    