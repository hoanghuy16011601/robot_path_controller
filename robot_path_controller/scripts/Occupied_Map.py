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
        for Y in range(0,self.__Number_Cell_In_Edge_Map):
            for X in range(0,self.__Number_Cell_In_Edge_Map):
                ## Check reset available
                X_In_Penalty_Map = int(X/self.__Number_Cel_In_Edge_Grid)
                Y_In_Penalty_Map = int(Y/self.__Number_Cel_In_Edge_Grid)
                if (X % self.__Number_Cel_In_Edge_Grid == 0) and (Y % self.__Number_Cel_In_Edge_Grid == 0):
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] = 0
                ## Calculate Penalty Point for map
                if map.data[100*Y + X] == 0:                                             ## blank cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 1          
                elif map.data[100*Y + X] == -1:                                          ## unexplored cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 5
                else:                                                                    ## occupied cell
                    self.Penalty_Map[X_In_Penalty_Map][Y_In_Penalty_Map] += 500

    def Calcutate_Penalty_Map_With_Passed_Positions(self,Passed_Positions:list):
        for Position in Passed_Positions:
            X_Axis = Position[0]
            Y_Axis = Position[1]
            self.Penalty_Map[X_Axis][Y_Axis] = 40

    def Calculate_Penalty_Map_With_With_Occupied_Positions(self,Occupied_Positions):
        for Position in Occupied_Positions:
            self.Penalty_Map[Position[0]][Position[1]] = 5000

    def Update_Occupied_In_PenaltyMap(self,Occupied_Position):
        self.Penalty_Map[Occupied_Position[0]][Occupied_Position[1]] = 5000
    
    def Get_Penalty_Map(self):
        return self.Penalty_Map

    def Get_Number_X_Axis_In_Map(self):
        return self.__Number_Grid_In_Edge_Map 
    
    def Get_Penalty_Point_Of_Position(self,Position):
        return self.Penalty_Map[Position[0]][Position[1]]