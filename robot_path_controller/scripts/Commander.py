#!/usr/bin/env
import math
import rospy
import copy
from std_msgs.msg import String 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from robot_path_controller.msg import Command
import time



class Dijkstra():
    def __init__(self,Number_X_Axis:int,Number_Y_Axis:int) -> None:
        self.Grid_Planning_Information = {}
        self.Grid_List = []
        for X in range(0,Number_X_Axis):
            for Y in range(0,Number_Y_Axis):
                self.Grid_Planning_Information[(X,Y)]={"Status":"Finding","Point":10000000,"From":()}
                self.Grid_List.append((X,Y))

    def __Reset_Path_Planning(self):
        for Grid in self.Grid_Planning_Information:
            self.Grid_Planning_Information[Grid]["Status"] = "Finding" 
            self.Grid_Planning_Information[Grid]["Point"]  = 10000000
            self.Grid_Planning_Information[Grid]["From"]   = ()
    
    def __Find_Lowest_Penalty_Of_Grid_Is_Not_Found(self,Target_Grid:tuple):
        Min_Penalty = 1000000000
        Lowest_Grid = ()
        for Grid_Information in self.Grid_Planning_Information:
            if (self.Grid_Planning_Information[Grid_Information]["Status"] == "Finding"):
                if self.Grid_Planning_Information[Grid_Information]["Point"] < Min_Penalty:
                    Lowest_Grid = Grid_Information
                    Min_Penalty = self.Grid_Planning_Information[Grid_Information]["Point"]
                elif self.Grid_Planning_Information[Grid_Information]["Point"] == Min_Penalty:
                    if (Target_Grid[0]-Grid_Information[0]) < (Target_Grid[0] - Lowest_Grid[0]):
                        Lowest_Grid = Grid_Information
                    else:
                        pass
                else: ## self.Grid_Planning_Information[Grid_Information]["Point"] > Min_Penalty:
                    pass
        return Lowest_Grid
    
    def __Update_Grid_Planning_Information(self,Grid:tuple,Status:str,Point:int,From:tuple):
        self.Grid_Planning_Information[Grid]["Status"]  = Status
        self.Grid_Planning_Information[Grid]["Point"]   = Point
        self.Grid_Planning_Information[Grid]["From"]    = From

    def __Update_Status_Of_Grid_Planning(self,Grid:tuple,Status:str):
        self.Grid_Planning_Information[Grid]["Status"]  = Status 
    def __Get_Status_Of_Grid(self,Grid:tuple):
        Status:str = self.Grid_Planning_Information[Grid]["Status"]
        return Status

    def __Update_Point_Of_Grid_Planning(self,Grid:tuple,Point:int):
        self.Grid_Planning_Information[Grid]["Point"]  = Point

    def __Get_Point_Of_Grid(self,Grid:tuple):
        Point:int = self.Grid_Planning_Information[Grid]["Point"]
        return Point
    
    def __Update_From_Of_Grid_Planning(self,Grid:tuple,From:tuple):
        self.Grid_Planning_Information[Grid]["From"]  = From 
    def __Get_From_Of_Grid(self,Grid:tuple):
        From:tuple = self.Grid_Planning_Information[Grid]["From"]
        return From
    
    def __Find_Path_Of_Found_Grid(self,Found_Grid:tuple):
        Path_Planning = []
        if self.Grid_Planning_Information[Found_Grid]["Status"] == "Found":
            From_Grid = Found_Grid 
            while self.__Get_Status_Of_Grid(From_Grid) != "Start":
                Path_Planning.append(From_Grid)
                From_Grid = self.__Get_From_Of_Grid(Grid = From_Grid)
        else:
            print("Cannot find path of found grid")
        Path_Planning = Path_Planning[::-1]
        return Path_Planning


    
    def Find_Path(self,Start_Grid:tuple,End_Grid:tuple,Penalty_Map:dict):
        self.__Reset_Path_Planning()
        self.__Update_Status_Of_Grid_Planning(Grid=Start_Grid,Status="Start")
        self.__Update_Point_Of_Grid_Planning(Grid=Start_Grid,Point=0)
        Grid_State_End = Start_Grid
        while(Grid_State_End != End_Grid):
            ## Begin grid of this state is the end grid of last state
            Grid_State_Begin = Grid_State_End

            # Finding Available_Grids which Robot can move in this state
            Available_Grids = [(Grid_State_Begin[0],Grid_State_Begin[1] + 1) , (Grid_State_Begin[0],Grid_State_Begin[1] - 1),
                               (Grid_State_Begin[0] + 1,Grid_State_Begin[1]) , (Grid_State_Begin[0] - 1,Grid_State_Begin[1])]
            
            ## Calculate Penalty for this state
            for Grid_State_Available_Check in Available_Grids:
                if (Grid_State_Available_Check in self.Grid_List) and (Grid_State_Available_Check != Start_Grid) and (self.__Get_Status_Of_Grid(Grid=Grid_State_Available_Check) != "Found"):
                    Grid_State_Check_Penalty = Penalty_Map[Grid_State_Available_Check[0]][Grid_State_Available_Check[1]]
                    Path_Penaty = self.__Get_Point_Of_Grid(Grid=Grid_State_Begin) + Grid_State_Check_Penalty
                    if Path_Penaty < self.__Get_Point_Of_Grid(Grid=Grid_State_Available_Check): 
                        self.__Update_Point_Of_Grid_Planning(Grid=Grid_State_Available_Check, Point=Path_Penaty)
                        self.__Update_From_Of_Grid_Planning(Grid=Grid_State_Available_Check, From=Grid_State_Begin)
                    else:
                        pass
                else:
                    pass    

            ## Finding the end grid of this state, which have lowest Penalty_point
            Grid_State_End = self.__Find_Lowest_Penalty_Of_Grid_Is_Not_Found(Target_Grid=End_Grid)
            self.__Update_Status_Of_Grid_Planning(Grid=Grid_State_End,Status="Found")
        Path_Planning = self.__Find_Path_Of_Found_Grid(Found_Grid=Grid_State_End)
        print(Path_Planning)
        return Path_Planning


class Lidar():
    def __init__(self):
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
        
        if abs(Head_Distance - self.Last_Head_Distance) <= 0.1:
            self.Head_Count_Check += 1
            if self.Head_Count_Check == 2:
                self.Head_Distance = Head_Distance
                self.Head_Count_Check = 0
        else:
            self.Head_Count_Check = 0
        self.Last_Head_Distance = Head_Distance

        if abs(Right_Distance - self.Last_Right_Distance) <= 0.1:
            self.Right_Count_Check += 1
            if self.Right_Count_Check == 2:
                self.Right_Distance = Right_Distance
                self.Right_Count_Check = 0
        else:
            self.Right_Count_Check = 0
        self.Right_Distance = Right_Distance

        if abs(Back_Distance - self.Last_Back_Distance) <= 0.1:
            self.Back_Count_Check += 1
            if self.Back_Count_Check == 2:
                self.Back_Distance = Back_Distance
                self.Back_Count_Check = 0
        else:
            self.Back_Count_Check = 0
        self.Back_Distance = Back_Distance

        if abs(Left_Distance - self.Last_Left_Distance) <= 0.1:
            self.Left_Count_Check += 1
            if self.Left_Count_Check == 2:
                self.Left_Distance = Left_Distance
                self.Left_Count_Check = 0
        else:
            self.Left_Count_Check = 0
        self.Last_Left_Distance = Left_Distance


class Penalty_Map():
    def __init__(self) -> None:
        self.__Number_Cell_In_Edge_Map = 100 
        Resoluntion = 0.1
        Grid_Size = 0.4
        self.__Number_Cel_In_Edge_Grid = int(Grid_Size/Resoluntion)
        self.__Number_Grid_In_Edge_Map = int(self.__Number_Cell_In_Edge_Map/self.__Number_Cel_In_Edge_Grid)
        self.__Create_Penalty_Map()
    
    def __Create_Penalty_Map(self):
        self.Penalty_Map = {}
        for X_Axis in range(self.__Number_Grid_In_Edge_Map):
   
            self.Penalty_Map[X_Axis] = []
            for Y_Axis in range(self.__Number_Grid_In_Edge_Map):
                self.Penalty_Map[X_Axis].append(0)
    
    def Convert_Occupancy_Map_To_Penalty_Map(self,map:list):
        for X in range(0,self.__Number_Cell_In_Edge_Map):
            for Y in range(0,self.__Number_Cell_In_Edge_Map):
                ## Check reset available
                X_In_Penalty_Map = int(X/self.__Number_Cel_In_Edge_Grid)
                Y_In_Penalty_Map = int(Y/self.__Number_Cel_In_Edge_Grid)
                if (X % self.__Number_Cel_In_Edge_Grid == 0) and (Y % self.__Number_Cel_In_Edge_Grid == 0):
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] = 0
                ## Calculate Penalty Point for map
                if map.data[100*X + Y] == 0:                                             ## blank cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 1          
                elif map.data[100*X + Y] == -1:                                          ## unexplored cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 5
                else:                                                                    ## occupied cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 500

    def Calcutate_Penalty_Map_With_Passed_Positions(self,Passed_Positions:list):
        for Position in Passed_Positions:
            X_Axis = Position[0]
            Y_Axis = Position[1]
            self.Penalty_Map[X_Axis][Y_Axis] = 40

    def Calculate_Penalty_Map_With_With_Occupied_Positions(self,Occuupied_Positions):
        for Position in Occuupied_Positions:
            self.Penalty_Map[Position[0]][Position[1]] = 5000
    
    def Get_Penalty_Map(self):
        return self.Penalty_Map

    def Get_Number_X_Axis_In_Map(self):
        return self.__Number_Grid_In_Edge_Map 
    
    def Get_Penalty_Point_Of_Position(self,Position):
        return self.Penalty_Map[Position[0]][Position[1]]

class Position():
    def __init__(self) -> None:
        self.Start_Position = (12,12)
        self.Passed_Positions = [self.Start_Position]
        self.Now_Position = (self.Start_Position)
        self.Last_Position = self.Start_Position
        self.SLAM_Now_Pose = self.Convert_PenaltyMap_Position_To_SLAM_Pose(PenaltyMap_Position=self.Start_Position) 
        self.SLAM_Now_Angle = 0
        self.Target_Position = ()
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
    
    def __Convert_SLAM_Pose_To_PenaltyMap_Position(self,SLAM_Pose:tuple):
        SLAM_Pose_X = SLAM_Pose[0]
        SLAM_Pose_Y = SLAM_Pose[1]

        if SLAM_Pose_X > 0:
            Position_X = (int)((SLAM_Pose_X + 0.2)/0.4) + 12
        else:
            Position_X = (int)((SLAM_Pose_X-0.2)/0.4) + 12

        if SLAM_Pose_Y > 0:
            Position_Y = (int)((SLAM_Pose_Y + 0.2)/0.4) + 12
        else:
            Position_Y = (int)((SLAM_Pose_Y-0.2)/0.4) + 12
        
        return (Position_X,Position_Y)

    def Determine_Now_Position(self,SLAM_Pose:tuple):
        Check_Position = self.__Convert_SLAM_Pose_To_PenaltyMap_Position(SLAM_Pose=SLAM_Pose)

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

    def Get_Occupied_Position(self):
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
        
        Ahead_Pose = self.Now_Position + Direction_Bias
        return Ahead_Pose


class Controller():
    def __init__(self,Penalty_Map:Penalty_Map,Robot_Position:Position, Path_Planning:Dijkstra, Robot_Lidar:Lidar) -> None:
        self.Penalty_Map = Penalty_Map
        self.Robot_Position = Robot_Position
        self.Path_Planning = Path_Planning
        self.Robot_Lidar = Robot_Lidar
        self.Finish_Flag = False

    def Check_Is_Cover_Full_Map(self, Penalty_Map:dict):
        Lowest_Point = 24
        Length_Axis = self.Penalty_Map.Get_Number_X_Axis_In_Map() 
        for X in range(0,Length_Axis):
            for Y in range(0,Length_Axis):
                if Penalty_Map[X][Y] < Lowest_Point:
                    Lowest_Point = Penalty_Map[X][Y]
                else:
                    pass
        if Lowest_Point < 24:
            Is_Cover_Full_Map = False
        else:
            Is_Cover_Full_Map = True
        return Is_Cover_Full_Map
    

    def __Calculate_PointMap_To_Choose_Pose_For_Movement(self, PenaltyMap:dict, Now_Position:tuple):
        List_Free = []
        Length_Axis = self.Penalty_Map.Get_Number_X_Axis_In_Map()
        Now_Angle = self.Robot_Position.Get_Now_Angle()
        X_Now = Now_Position[0]
        Y_Now = Now_Position[1]
        ## Create penalty map for possible position finding, too far to X_Now (in X-Axis) too more penalty point 
        if Now_Angle == 0:
            Direction_Point = (1,0)
            Bias = (1,0)
        elif Now_Angle == 180:
            Direction_Point = (1,0)
            Bias = (0,1)
        elif Now_Angle == 90:
            Direction_Point = (0,1)
            Bias = (0,1)
        else:
            Direction_Point = (0,1)
            Bias = (1,0)

        PointMap_For_Posible_Pose = copy.deepcopy(PenaltyMap)
        PointMap_For_Posible_Pose[X_Now][Y_Now] += 100000
        for X_PointMap_For_Posible_Pose in range(0,Length_Axis):
            for Y_PointMap_For_Posible_Pose in range(0,Length_Axis):
                Penalty_Point = abs(Y_Now - Y_PointMap_For_Posible_Pose) + abs(X_Now - X_PointMap_For_Posible_Pose)*0.03  # prefer move in same X-Axis
                
                # Calculate Extra point base from direction of robot
                if X_PointMap_For_Posible_Pose > X_Now:
                    X_Weight_Point =(X_PointMap_For_Posible_Pose - X_Now)/25 - Bias[0]
                else:
                    X_Weight_Point = (X_Now - X_PointMap_For_Posible_Pose)/25 - Bias[1]
                
                if Y_PointMap_For_Posible_Pose > Y_Now:
                    Y_Weight_Point = (Y_PointMap_For_Posible_Pose - Y_Now)/25 - Bias[0]
                else:
                    Y_Weight_Point = (Y_Now - Y_PointMap_For_Posible_Pose)/25 - Bias[1] 


                Extra_point = X_Weight_Point*Direction_Point[0] + Y_Weight_Point*Direction_Point[1]

                if PointMap_For_Posible_Pose[X_PointMap_For_Posible_Pose][Y_PointMap_For_Posible_Pose] == 16:
                    List_Free.append((X_PointMap_For_Posible_Pose,Y_PointMap_For_Posible_Pose))
                    
                if PointMap_For_Posible_Pose[X_PointMap_For_Posible_Pose][Y_PointMap_For_Posible_Pose] == 35:
                    PointMap_For_Posible_Pose[X_PointMap_For_Posible_Pose][Y_PointMap_For_Posible_Pose] += 50 + Penalty_Point + Extra_point
                else:
                    PointMap_For_Posible_Pose[X_PointMap_For_Posible_Pose][Y_PointMap_For_Posible_Pose] += Penalty_Point + Extra_point
        print(List_Free)
        return PointMap_For_Posible_Pose
    
    def __Choose_New_Position_To_Move(self, PointMap_For_Posible_Pose, Now_Position):
        Length_Axis = self.Penalty_Map.Get_Number_X_Axis_In_Map()
        Point_Of_New_Pose = 10000
        for X_Check in range(0,Length_Axis):
            for Y_Check in range(0,Length_Axis):
                Point_Of_Checking_Pose = PointMap_For_Posible_Pose[X_Check][Y_Check]
                if Point_Of_Checking_Pose < Point_Of_New_Pose:
                    New_Pose = (X_Check,Y_Check)
                    Point_Of_New_Pose = Point_Of_Checking_Pose 
                elif Point_Of_Checking_Pose == Point_Of_New_Pose : 
                    if (abs(X_Check - Now_Position[0]) + abs(Y_Check - Now_Position[1])) < (abs(New_Pose[0] - Now_Position[0]) + abs(New_Pose[1] - Now_Position[1])):
                        New_Pose = (X_Check,Y_Check)
        return New_Pose

    def __Determine_Possible_NewPosition_To_Move(self):
        Penalty_Map = self.Penalty_Map.Get_Penalty_Map()
        Position = self.Robot_Position.Get_Now_Position()
        PointMap_For_Posible_Pose = self.__Calculate_PointMap_To_Choose_Pose_For_Movement(PenaltyMap=Penalty_Map,Now_Position=Position)
        New_Pose = self.__Choose_New_Position_To_Move(PointMap_For_Posible_Pose=PointMap_For_Posible_Pose, Now_Position= Position)
        return New_Pose
    
    def Determine_New_Position_For_Robot(self,Now_Position:tuple, Now_Penalty_Map:dict):
        Reponse = self.Check_Is_Cover_Full_Map(Penalty_Map=Now_Penalty_Map)
        if Reponse == True:
            print("Robot has covered full map")
            if Now_Position != self.Robot_Position.Get_Start_Position():
                Target_Pose = self.Robot_Position.Get_Start_Position()
            else:
                return self.Robot_Position.Get_Start_Position(),True
        else:
            Target_Pose = self.__Determine_Possible_NewPosition_To_Move()
        print(f"Target Pose:{Target_Pose}")
        Path = self.Path_Planning.Find_Path(Start_Grid=Now_Position,End_Grid=Target_Pose,Penalty_Map=Now_Penalty_Map)
        return Path[0], False
    
    def Determine_New_Angle_For_Robot(self,Target_Position:tuple, Now_Position:tuple, Now_Angle:int):
        if Now_Position[0] == Target_Position[0]: # X_Now == X_Target => Y_Now != Y_Target
            if Now_Position[1] < Target_Position[1]:    # Y_Now < Y_Target
                New_Angle = 270
            else:                                       # Y_Now > Y_Target
                New_Angle = 90
        else:                                     # X_Now != X_Target  => Y_Now == Y_Target
            if Now_Position[0] < Target_Position[0]:    # X_Now < X_Target
                New_Angle = 0
            else:
                New_Angle = 180
        return New_Angle
    
    def Fix_Error_Degreed(self):
        Command = {
            "Type"  : "",
            "Value" : 0
        }

        SLAM_Now_Angle = self.Robot_Position.Get_SLAM_Now_Angle()
        Now_Angle = self.Robot_Position.Get_Now_Angle()
        if Now_Angle == 0:
            Now_Angle = 360
            if SLAM_Now_Angle >= 0 and SLAM_Now_Angle < 180: 
                Error = SLAM_Now_Angle
            else:
                Error = Now_Angle - SLAM_Now_Angle
        else:
            Error = abs(Now_Angle - SLAM_Now_Angle)
        if Error > 3:
            if (Now_Angle - SLAM_Now_Angle) > 0:
                if (Now_Angle - SLAM_Now_Angle) > 180:
                    Command["Type"] = "Rotate-Left"
                    Command["Value"] = 360-(Now_Angle - SLAM_Now_Angle)
                else:
                    Command["Type"] = "Rotate-Right"
                    Command["Value"] = Now_Angle - SLAM_Now_Angle

            else:
                if (Now_Angle - SLAM_Now_Angle) < -180:
                    Command["Type"] = "Rotate-Right"
                    Command["Value"] = 360 + (Now_Angle - SLAM_Now_Angle)
                else:
                    Command["Type"] = "Rotate-Left"
                    Command["Value"] = -(Now_Angle - SLAM_Now_Angle)
            List_Commands = [Command]
        else:
            List_Commands = []
        return List_Commands
    

    def __Get_Command_For_Control_Robot_Forward(self):
        Target_Position = self.Robot_Position.Get_Target_Position()
        SLAM_Target_Pose = self.Robot_Position.Convert_PenaltyMap_Position_To_SLAM_Pose(PenaltyMap_Position=Target_Position)
        SLAM_Now_Pose = self.Robot_Position.Get_SLAM_Now_Pose()
        Now_Angle = self.Robot_Position.Get_Now_Angle()
        if Now_Angle == 0 or Now_Angle == 180:      #X-Axis
            Distance = self.Robot_Position.Calculate_Distance_In_SLAM(Axis= "X",From_Pose=SLAM_Now_Pose,To_Pose = SLAM_Target_Pose)
        else:                                       #Y-Axis
            Distance = self.Robot_Position.Calculate_Distance_In_SLAM(Axis= "Y",From_Pose=SLAM_Now_Pose,To_Pose = SLAM_Target_Pose)
        Distance *= 100                              # Convert meters to centimeters
        Command = {
            "Type"  : "Forward",
            "Value" : Distance
        }
        
        return Command
    
    def __Get_Commands_For_Control_Robot_Rotate_BackSide(self):
        (Head_Distance, Right_Distance, Back_Distance, Left_Distance) = self.Robot_Lidar.Get_Distances()
        List_Commands = []

        if Head_Distance <= 0.4:
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 40
            })

        elif Right_Distance > 0.45:
            List_Commands.append({
                "Type"  : "Rotate-Right",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 35
            })
            List_Commands.append({
                "Type"  : "Rotate-Right",
                "Value" : 90
            })

        elif Left_Distance > 0.45:
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 35
            })
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })

        else:
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 40
            })
        return List_Commands

    def Determine_Commands_For_Robot(self, Target_Angle:int, Now_Angle:int):

    # if Angle is 0 degree so the robot in direct toward increase of y-axis
    # if Angle is 90 degree so the robot in direct toward increase of x-axis
    # if angle is 180 degree so the robot in direct toward decrease of y-axis
    # if angle is 270 degree so the robot in direct toward decrease of x-axis

    # 0 <= Angle < 360
        List_Commands = []
        Command = {
            "Type"  : "",
            "Value" : 0
        }

        if Target_Angle == Now_Angle:           # In same direction so can move
            if self.Finish_Flag == True:
                List_Commands.append({
                        "Type"  : "Finish",
                        "Value" : 0
                    })
            else:
                List_Commands.append(self.__Get_Command_For_Control_Robot_Forward())

        elif abs(Target_Angle - Now_Angle) == 180:
            List_Commands = self.__Get_Commands_For_Control_Robot_Rotate_BackSide()
            if List_Commands[0]["Type"] == "Backward":
                self.Robot_Position.Update_Target_Angle(Angle=Now_Angle)

        else:                                   # Not same direction so must be rotate first
            if Target_Angle == 0:
                Target_Angle = 360
            else:
                pass

            Distances = self.Robot_Lidar.Get_Distances()
            if Distances[0] < 23:
                Back_Distance = 25
            else:
                Back_Distance = 20
            if (Target_Angle - Now_Angle) > 0:
                if (Target_Angle - Now_Angle) > 180:
                    Command["Type"] = "Rotate-Left"
                    Command["Value"] = 360-(Target_Angle - Now_Angle)
                else:
                    Command["Type"] = "Rotate-Right"
                    Command["Value"] = Target_Angle - Now_Angle

            else:
                if (Target_Angle - Now_Angle) < -180:
                    Command["Type"] = "Rotate-Right"
                    Command["Value"] = 360 + (Target_Angle - Now_Angle)
                else:
                    Command["Type"] = "Rotate-Left"
                    Command["Value"] = -(Target_Angle - Now_Angle)

            List_Commands.append({
                "Type"  : "Backward",
                "Value" : Back_Distance
            })
            List_Commands.append(Command)
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : Back_Distance
            })
        
        return List_Commands
    
    def Get_List_Command_Robot(self):
        Now_Position = self.Robot_Position.Get_Now_Position()
        Now_Angle = self.Robot_Position.Get_Now_Angle()
        Now_Penalty_Map = self.Penalty_Map.Get_Penalty_Map()
        print(f"Now_Position {Now_Position} ---- Now_Angle {Now_Angle}")
        New_Position,Was_Back_To_Start = self.Determine_New_Position_For_Robot(Now_Position=Now_Position,Now_Penalty_Map=Now_Penalty_Map)
        if Was_Back_To_Start == False:
            New_Angle = self.Determine_New_Angle_For_Robot(Target_Position=New_Position,Now_Position=Now_Position,Now_Angle=Now_Angle)
        else:
            New_Angle = 0
            self.Finish_Flag = True

        self.Robot_Position.Update_Target_Position(New_Position)
        self.Robot_Position.Update_Target_Angle(New_Angle)

        print(f"New_Position {New_Position} ---- New_Angle {New_Angle}")
        List_Commands = self.Determine_Commands_For_Robot(Target_Angle=New_Angle,Now_Angle=Now_Angle)
        return List_Commands


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

    def __On_Task_Is_Done(self):
        self.Is_Movement = False
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
                time.sleep(0.2)
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

    def __Robot_Backward(self):
        self.Command_Message.type = "Backward"
        self.Command_Message.value = 15
        self.Publisher.publish(self.Command_Message)
        print("Robot Backward")

    def __Stop_Robot(self):
        self.Command_Message.type = "Stop"
        self.Command_Message.value = 0
        self.Publisher.publish(self.Command_Message)
        print("Stop Robot Urgency")
    
    def Object_Detected(self,Source = "Lidar"):
        self.Is_Movement = False
        del self.List_Commands[:]
        self.__Stop_Robot()
        self.__Robot_Backward()
        rospy.sleep(1)
        Distances = self.Algorithm_Controller.Robot_Lidar.Get_Distances()
        if Distances[0] < 0.3 or Source == "Ultrasonic":
            if self.Algorithm_Controller.Robot_Position.Get_Target_Position() != self.Algorithm_Controller.Robot_Position.Get_Now_Position():
                self.Algorithm_Controller.Robot_Position.Update_Occupied_Position(Position=self.Algorithm_Controller.Robot_Position.Get_Target_Position())
            else:
                self.Algorithm_Controller.Robot_Position.Update_Occupied_Position(Position=self.Algorithm_Controller.Robot_Position.Get_Ahead_Position())



    def Map_Callback_Handler(self,data):
        self.Flag_Map = True
        Passed_Positions = self.Algorithm_Controller.Robot_Position.Get_Passed_Positions()
        Occupied_Positions = self.Algorithm_Controller.Robot_Position.Get_Occupied_Position()
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
        self.Algorithm_Controller.Robot_Position.Update_Passed_Position(Position=Now_Position)

    def Lidar_Callback_Handler(self,msg:LaserScan):
        self.Algorithm_Controller.Robot_Lidar.Convert_To_Sides_Distance(msg=msg)
        if self.Is_Movement == True:
            Distances = self.Algorithm_Controller.Robot_Lidar.Get_Distances()       #=>(Head , Right , Back, Left)
            if Distances[0] <= 0.3:
                self.Object_Detected(Source= "Lidar")
                rospy.sleep(0.5)

        
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
        rospy.init_node('flood_fill',anonymous = True)
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

