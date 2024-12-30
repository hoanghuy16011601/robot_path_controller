import copy

from Path_Planning import Dijkstra
from Lidar import Lidar
from Occupied_Map import Penalty_Map
from Robot_State import Position

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
    

    def Calculate_Optimize_Point(self,Now_Position:tuple, PenaltyMap):
        X_Now = Now_Position[0]
        Y_Now = Now_Position[1]
        Accumulate_Optimize_Point = {
            "X"  : [],
            "Y"  : [],
        }
        Optimize_Point_X_Lower = 0
        Optimize_Point_X_Higher = 0
        Optimize_Point_Y_Lower = 0
        Optimize_Point_Y_Higher = 0
        for Index in range(0,len(PenaltyMap)):
            if Index < X_Now:
                if Index == 0:
                    Optimize_Point = 0
                else:
                    Occupied_Point = PenaltyMap[X_Now-Index][Y_Now]
                    if Occupied_Point == 40:
                        Optimize_Point = 0.02
                    elif Occupied_Point > 500:
                        Optimize_Point = 0.2
                    else:
                        Optimize_Point = 0
                Optimize_Point_X_Lower += Optimize_Point
                Accumulate_Optimize_Point["X"].insert(0,Optimize_Point_X_Lower)

            elif Index == X_Now:
                Accumulate_Optimize_Point["X"].append(0)

            else:
                if Index - X_Now == 1:
                    Optimize_Point = 0
                else:
                    Occupied_Point = PenaltyMap[Index - 1][Y_Now]
                    if Occupied_Point == 40:
                        Optimize_Point = 0.01
                    elif Occupied_Point > 500:
                        Optimize_Point= 0.2
                    else:
                        Optimize_Point = 0
                Optimize_Point_X_Higher += Optimize_Point
                Accumulate_Optimize_Point["X"].append(Optimize_Point_X_Higher)

            if Index < Y_Now:
                if Index == 0:
                    Optimize_Point = 0
                else:
                    Occupied_Point = PenaltyMap[X_Now][Y_Now-Index]
                    if Occupied_Point == 40:
                        Optimize_Point = 0.01
                    elif Occupied_Point > 500:
                        Optimize_Point = 0.2
                    else:
                        Optimize_Point = 0
                Optimize_Point_Y_Lower += Optimize_Point
                Accumulate_Optimize_Point["Y"].insert(0,Optimize_Point_Y_Lower)

            elif Index == Y_Now:
                Accumulate_Optimize_Point["Y"].append(0)

            else:
                if Index - Y_Now == 1:
                    Optimize_Point = 0
                else:
                    Occupied_Point = PenaltyMap[X_Now][Index - 1]
                    if Occupied_Point == 40:
                        Optimize_Point = 0.01
                    elif Occupied_Point > 500:
                        Optimize_Point= 0.2
                    else:
                        Optimize_Point = 0
                Optimize_Point_Y_Higher += Optimize_Point
                Accumulate_Optimize_Point["Y"].append(Optimize_Point_Y_Higher)
        return Accumulate_Optimize_Point

            

            
            




    def __Calculate_PointMap_To_Choose_Pose_For_Movement(self, PenaltyMap:dict, Now_Position:tuple):
        Length_Axis = self.Penalty_Map.Get_Number_X_Axis_In_Map()
        Now_Angle = self.Robot_Position.Get_Now_Angle()
        X_Now = Now_Position[0]
        Y_Now = Now_Position[1]
        ## Create penalty map for possible position finding, too far to X_Now (in X-Axis) too more penalty point 
        if Now_Angle == 0:
            Angle_Point = (0,0.03)
            Axis_Point = (1,0)
        elif Now_Angle == 180:
            Angle_Point = (0.03,0)
            Axis_Point = (1,0)
        elif Now_Angle == 90:
            Angle_Point = (0.03,0)
            Axis_Point = (0,1)
        else:
            Angle_Point = (0,0.03)
            Axis_Point = (0,1)

        Axis_Optimize_Points = self.Calculate_Optimize_Point(Now_Position=Now_Position,PenaltyMap=PenaltyMap)
        PointMap_For_Posible_Pose = copy.deepcopy(PenaltyMap)
        for Y_PointMap_For_Posible_Pose in range(0,Length_Axis):
            for X_PointMap_For_Posible_Pose in range(0,Length_Axis):
                Position_Point = abs(Y_Now - Y_PointMap_For_Posible_Pose)*0.1 + abs(X_Now - X_PointMap_For_Posible_Pose)*0.01  # prefer move in same X-Axis
                
                # Calculate Extra point base from direction of robot   
                if X_PointMap_For_Posible_Pose == X_Now and Y_PointMap_For_Posible_Pose == Y_Now:
                    PointMap_For_Posible_Pose[X_Now][Y_Now] = 100000
                    continue
                elif Y_PointMap_For_Posible_Pose == Y_Now:
                    Optimize_Point = Axis_Optimize_Points["X"][X_PointMap_For_Posible_Pose]
                    if X_PointMap_For_Posible_Pose > X_Now:
                        Direction_Point = Axis_Point[0]*Angle_Point[0] 
                        # Calculate optimize point to avoid move to covered position and decrese priority for position which is behind by object 
                    else:
                        Direction_Point = Axis_Point[0]*Angle_Point[1] 

                elif X_PointMap_For_Posible_Pose == X_Now:
                    Optimize_Point = Axis_Optimize_Points["Y"][Y_PointMap_For_Posible_Pose]
                    if Y_PointMap_For_Posible_Pose > Y_Now:
                        Direction_Point = Axis_Point[1]*Angle_Point[0]
                    else:
                        Direction_Point = Axis_Point[1]*Angle_Point[1]
                else:
                    Direction_Point = 0
                    Optimize_Point = 0

                PointMap_For_Posible_Pose[X_PointMap_For_Posible_Pose][Y_PointMap_For_Posible_Pose] += Position_Point + Direction_Point + Optimize_Point
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

    def __Determine_Possible_NewPosition_To_Move(self,Position, Penalty_Map):
        PointMap_For_Posible_Pose = self.__Calculate_PointMap_To_Choose_Pose_For_Movement(PenaltyMap=Penalty_Map,Now_Position=Position)
        New_Pose = self.__Choose_New_Position_To_Move(PointMap_For_Posible_Pose=PointMap_For_Posible_Pose, Now_Position= Position)
        return New_Pose
    
    def Determine_New_Position_For_Robot(self,Now_Position:tuple, Now_Penalty_Map:dict):
        Valid_Position = False
        while Valid_Position == False:
            # print(f"Occupied Position {self.Robot_Position.Get_Occupied_Positions()}")
            Reponse = self.Check_Is_Cover_Full_Map(Penalty_Map=Now_Penalty_Map)
            if Reponse == True:
                print("Robot has covered full map")
                if Now_Position != self.Robot_Position.Get_Start_Position():
                    self.Robot_Position.Update_Scope_Position(self.Robot_Position.Get_Start_Position())
                else:
                    return self.Robot_Position.Get_Start_Position(),True
            else:
                Scope_Position = self.Robot_Position.Get_Scope_Position() 
                if Scope_Position == Now_Position or Scope_Position == ():
                    self.Robot_Position.Update_Scope_Position(self.__Determine_Possible_NewPosition_To_Move(Position= Now_Position, Penalty_Map=Now_Penalty_Map))
            Scope_Pose = self.Robot_Position.Get_Scope_Position()
            print(f"Scope Pose:{Scope_Pose}")
            Path = self.Path_Planning.Find_Path(Start_Grid=Now_Position,End_Grid=Scope_Pose,Penalty_Map=Now_Penalty_Map)
            if self.Penalty_Map.Get_Penalty_Point_Of_Position(Path[0]) > 48 and Reponse == False: 
                self.Robot_Position.Update_Scope_Position(Position = ())
                self.Robot_Position.Update_Occupied_Position(Scope_Pose)
                Occupied_Positions = self.Robot_Position.Get_Occupied_Positions()
                self.Penalty_Map.Calculate_Penalty_Map_With_With_Occupied_Positions(Occupied_Positions)
                Valid_Position = False
            else:
                Valid_Position = True
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

        if Right_Distance < 0.3 and Left_Distance < 0.3:
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 40
            })
        
        elif Head_Distance < 0.4:
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 20
            })
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 20
            })
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 20
            })

        elif Right_Distance > 0.45:
            List_Commands.append({
                "Type"  : "Rotate-Right",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 15
            })
            List_Commands.append({
                "Type"  : "Rotate-Right",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 15
            })

        elif Left_Distance > 0.45:
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 15
            })
            List_Commands.append({
                "Type"  : "Rotate-Left",
                "Value" : 90
            })
            List_Commands.append({
                "Type"  : "Backward",
                "Value" : 15
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
            if List_Commands[0]["Type"] == "Backward" and len(List_Commands) == 1:
                self.Robot_Position.Update_Target_Angle(Angle=Now_Angle)

        else:                                   # Not same direction so must be rotate first
            if Target_Angle == 0:
                Target_Angle = 360
            else:
                pass

            Back_Distance = 15
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
            if Now_Angle == 0:
                self.Finish_Flag = True

        self.Robot_Position.Update_Target_Position(New_Position)
        self.Robot_Position.Update_Target_Angle(New_Angle)

        print(f"New_Position {New_Position} ---- New_Angle {New_Angle}")
        List_Commands = self.Determine_Commands_For_Robot(Target_Angle=New_Angle,Now_Angle=Now_Angle)
        return List_Commands