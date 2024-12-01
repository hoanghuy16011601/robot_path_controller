
class Dijkstra():
    def __init__(self,Number_X_Axis:int,Number_Y_Axis:int) -> None:
        self.Grid_Planning_Information = {}
        self.Grid_List = []
        for X in range(0,Number_X_Axis):
            for Y in range(0,Number_Y_Axis):
                self.Grid_Planning_Information[(X,Y)]={"Status":"Finding","Point":10000000,"From":()}
                self.Grid_List.append((X,Y))

    def Reset_Path_Planning(self):
        for Grid in self.Grid_Planning_Information:
            Grid["Status"] = "Finding" 
            Grid["Point"]  = 10000000
            Grid["From"]   = ()
    
    def __Find_Lowest_Penalty_Of_Grid_Is_Not_Found(self,Target_Grid:tuple):
        Min_Penalty = 1000000
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
            print("Cannot find path of not found grid")
        Path_Planning = Path_Planning[::-1]
        return Path_Planning


    
    def Find_Path(self,Start_Grid:tuple,End_Grid:tuple,Penalty_Map:dict):
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


