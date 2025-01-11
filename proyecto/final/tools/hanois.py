from typing import List

def stringReplaceIndex(string,index,new_char) -> str:
    string_list = list(string)
    string_list[index] = new_char
    new_string = "".join(string_list)
    
    return new_string
#Esta función se utiliza para pintar el PaintHanoi.

class HanoiMovement:
    def __init__(
        self,
        source_tower: int,
        target_tower: int,
        source_disc_height: int,
        target_disc_height: int,
        disc_size: int,
    ):
        self.source_tower: int = source_tower
        self.target_tower: int = target_tower
        self.source_disc_height: int = source_disc_height
        self.target_disc_height: int = target_disc_height
        self.disc_size: int = disc_size
    def getMatrix(self)-> List[int]:
        return [self.source_tower,self.target_tower,self.source_disc_height,0,self.disc_size]
    def __str__(self):
        return f"Source:{self.source_tower}, Target:{self.target_tower}, S-Heigth:{self.source_disc_height} disc_size:{self.disc_size}"


class HanoiTower:
    def __init__(self, discs: List[int]):
        self.discs = discs

    def __str__(self):
        return f"{self.discs}"

    """
    def getTopOfTower(self) -> list[int]:

        for i in range(0, len(self.discs)):
            if self.discs[i] != 0:
                return [self.discs[i], i]
        # si no encuentra nada
        return [self.discs[-1], len(self.discs)]
    """

    def getTopOfTower(self) -> dict:
        result: dict = {"size": 0, "position": 0}
        for i in range(0, len(self.discs)):
            if self.discs[i] != 0:
                result["size"] = self.discs[i]
                result["position"] = i
                return result
        # si no encuentra nada
        result["size"] = self.discs[-1]
        result["position"] = len(self.discs)
        return result

    def popTopOfTower(self) -> int:
        top: dict = self.getTopOfTower()
        result: int = top["size"]
        position: int = top["position"]
        self.discs[position] = 0
        return result

    def placeOnTopOfTower(self, disc: int) -> None:
        top: dict = self.getTopOfTower()
        # print(f'top:{top}, discs:{self.discs}')
        if (disc < top["size"]) or (top["size"] == 0):
            self.discs[top["position"] - 1] = disc
        elif disc > top["size"]:
            # raise Exception(
            #     f"Hanoi placeOnTopOfTower Error: trying to place bigger:{disc} on top of smaller{top["size"]}"
            # )
            pass
        elif disc == top["size"]:
            print("how did we get here")
            # raise Exception(
            #     f"Hanoi placeOnTopOfTower Error: two of the same size: size {disc}"
            # )
            pass


class HanoiGame:
    def __init__(
        self,
        towers: int = 3,
        discs: int = 5,
        start_tower: int = 0,
        aux_tower: int = 1,
        target_tower: int = 2,
        standard_start: bool = True,
    ):
        self.towers: List[HanoiTower] = []
        self.discs: int = discs
        self.target_tower: int = target_tower
        self.start_tower: int = start_tower
        self.aux_tower: int = aux_tower
        if standard_start:
            self.towers.append(HanoiTower(list(range(1, discs + 1))))
            for i in range(0, towers - 1):
                towerDiscs = []
                for i in range(0, discs):
                    towerDiscs.append(0)
                self.towers.append(HanoiTower(discs=towerDiscs))

        else:
            #print("\nrandom start not implemented yet!\n")
            pass

    def __str__(self):
        result = f"[HANOI] \nTowers:{len(self.towers)}\nDiscs:{self.discs}\nTarget_tower:{self.target_tower}\n"
        for i in range(0, len(self.towers)):
            result = result.__add__(f"Tower {i+1}: {str(self.towers[i])}\n")

        return result

    def checkSolved(self) -> bool:
        # objetivo: devuelve True sólo si están todas las piezas ordenadas así: [1,2,3,4,5] en el Target
        current = self.towers[self.target_tower]
        goal = list(range(1, self.discs + 1))
        # print(current)
        # print(goal)
        return f"{current}" == f"{goal}"  # esto es muy cutre pero tira

    def moveFromTo(self, origin: int, target: int) -> bool:
        disc: int = self.towers[origin].popTopOfTower()
        self.towers[target].placeOnTopOfTower(disc=disc)

    def solveHanoi(self):
        print(
            "Esta es la implementación de la solución tradicional. Uso ilustrativo only."
        )

        def solveHanoiInner(
            disc: int, source_tower: int, destination_tower: int, aux_tower
        ):
            # begin
            if disc == 1:
                print(f"Move the plate from {source_tower} to {destination_tower} ")
            else:
                solveHanoiInner(
                    disc=disc - 1,
                    source_tower=source_tower,
                    destination_tower=aux_tower,
                    aux_tower=destination_tower,
                )
                print(f"Move the plate from {source_tower} to {destination_tower}")
                solveHanoiInner(
                    disc=disc - 1,
                    source_tower=aux_tower,
                    destination_tower=destination_tower,
                    aux_tower=source_tower,
                )

        solveHanoiInner(
            disc=self.discs,
            source_tower=self.start_tower,
            destination_tower=self.target_tower,
            aux_tower=self.aux_tower,
        )
        
    def findDisk(self,disk: int,) -> int:
        result:int = 0
        
        for i in range(0,len(self.towers)):
            if disk in self.towers[i].discs:
                result = i
            
        return result
    
    def findTop(self) -> dict:
            result: dict = {
                "tower": 0,
                "size": 0,
            }
            #encontramos el 1
            for tower in range(0, len(self.towers)):
                topOfTower = self.towers[tower].getTopOfTower()
                if topOfTower["size"] == 1:
                    result["tower"] = tower
                    result["size"] = 1
            #miramos a ver si la misma torre tiene los siguentes números y si no los tiene sale a devolver.
            while True:
                if result["size"]+1 in self.towers[result["tower"]].discs:
                    result["size"] = result["size"]+1
                else:
                    return result
        

    def getRandomHanoiSolutionArray(self) -> List[HanoiMovement]:
        #print("Utiliza esta función en conjunto con processSolution()")
        instructions: List[HanoiMovement] = []
        
        #Bug: Random solver se rompe si el disco mas grande no comienza en la torre destino.
        tow = self.findDisk(disk=self.discs)
        self.target_tower = tow
        #solucion: reescribir el destino a donde se encuentre el mayor disco.        
        
        #Bug: Si está resuelto de base, con 0 movimientos, no hay nada que limpiar:
        if(self.checkSolved() == True):
            print("Está resuelto de base")
            return instructions
        #solucion: if -> return y un print y que no vuelva a pasar    
        
        while self.checkSolved() == False:
            # step 1: find the largest continuous stack starting with 1
            top:dict = self.findTop()
            #print(top)
            
            # step 2: move this stack to the next largest disk using normal hanoi logic
            # step 2.1: encuentra la posición del disco
            nextTower:int = self.findDisk(disk=(top['size']+1))
            # print(f'next disk tower: {nextTower}')
            
            # step 2.2 : mueve el top a el siguiente tal
            #step 2.2.1 calcula la aux tower cagoen
            varAux:List = [0,1,2]
            varAux.remove(nextTower)
            varAux.remove(top["tower"])
            #print(f'varAux:{varAux}')
            
            ## continuamos con el 2.2
            fakeHanoi: HanoiGame = HanoiGame(towers=3,discs=top["size"],start_tower=top["tower"],target_tower=nextTower, aux_tower=varAux[0],standard_start=True,)
            movements:List[HanoiMovement] = fakeHanoi.getStandardHanoiSolutionArray()
            movements = self.processSolution(movements)
            
            for movement in movements:
                instructions.append(movement)
            
                # repeat until solved
                
        print(f"Steps Before Optimization:{len(instructions)}")  
##
        def CleanRandomInstructions(instructions: List[HanoiMovement]):   
            
            result:List[HanoiMovement] = []
            memory: HanoiMovement = instructions[0]
            for ins in range(1,len(instructions)):
                if instructions[ins].disc_size == memory.disc_size:
                    memory = instructions[ins]
                if instructions[ins].disc_size != memory.disc_size:
                    result.append(memory)
                    memory = instructions[ins]
            return result
##        
        instructions = CleanRandomInstructions(instructions);      
        
        print(f"Steps After Optimization:{len(instructions)}")  
        return instructions

    def getStandardHanoiSolutionArray(self) -> List[HanoiMovement]:
        # print("Utiliza esta función en conjunto con processSolution()")
        def solveHanoiInnerInstructions(
            disc: int,
            source_tower: str,
            destination_tower: str,
            aux_tower: str,
            pool: List[HanoiGame],
        ):
            instructions = pool
            if disc == 1:
                instructions.append(
                    HanoiMovement(
                        source_tower=source_tower,
                        target_tower=destination_tower,
                        target_disc_height=0,
                        source_disc_height=0,
                        disc_size=0,
                    )
                )
            else:
                solveHanoiInnerInstructions(
                    disc=disc - 1,
                    source_tower=source_tower,
                    destination_tower=aux_tower,
                    aux_tower=destination_tower,
                    pool=instructions,
                )
                instructions.append(
                    HanoiMovement(
                        source_tower=source_tower,
                        target_tower=destination_tower,
                        target_disc_height=0,
                        source_disc_height=0,
                        disc_size=0,
                    )
                )
                solveHanoiInnerInstructions(
                    disc=disc - 1,
                    source_tower=aux_tower,
                    destination_tower=destination_tower,
                    aux_tower=source_tower,
                    pool=instructions,
                )
            return pool

        instructions: list[HanoiMovement] = solveHanoiInnerInstructions(
            disc=self.discs,
            source_tower=self.start_tower,
            destination_tower=self.target_tower,
            aux_tower=self.aux_tower,
            pool=[],
        )
        return instructions

    def processSolution(self, solution: List[HanoiMovement]) -> List[HanoiMovement]:
        for i in solution:
            top_of_source = self.towers[i.source_tower].getTopOfTower()
            i.disc_size = top_of_source["size"]
            i.source_disc_height = top_of_source["position"]
            top_of_target = self.towers[i.target_tower].getTopOfTower()
            i.target_disc_height = top_of_target["position"]
            self.moveFromTo(origin=i.source_tower, target=i.target_tower)
            # print(i)
        return solution

    def paintHanoi(self) -> None:
        result = []
        bar = "_"
        max_radius = self.discs
        max_diameter = max_radius * 2 + 2
        max_height = self.discs + 3
        base_len = max_diameter

        for height in range(0, self.discs):
            res = ""
            for tower in range(0, len(self.towers)):
                # print(f"height:{height}, tower:{tower}")
                if self.towers[tower].discs[height] == 0:
                    res = res + (f" " * max_radius + "||" + " " * max_radius)
                else:
                    size = self.towers[tower].discs[height]
                    res = res + (
                        f" " * (max_radius - size)
                        + "|"
                        + " " * size
                        + " " * size
                        + "|"
                        + " " * (max_radius - size)
                    )
            for tower in range(0, len(self.towers)):
                res = res + f"[{self.towers[tower].discs[height]}]"

            result.append(res)

        result.append(f"{bar*(max_diameter*len(self.towers))}")
        result.append(f"|{bar*((max_diameter*len(self.towers))-2)}|")
        # parte del codigo que calcula donde poner el target
        target_idx = 1 + ((self.target_tower) * max_diameter) + max_radius
        # print(f'target_idx={target_idx}')
        # parte del codigo de pintar que pone el target
        result[-1] = stringReplaceIndex(
            new_char="T",
            string=result[-1],
            index=target_idx - 2,
        )
        result[-1] = stringReplaceIndex(
            new_char="A",
            string=result[-1],
            index=target_idx - 1,
        )
        result[-1] = stringReplaceIndex(
            new_char="R",
            string=result[-1],
            index=target_idx,
        )
        result[-1] = stringReplaceIndex(
            new_char="G",
            string=result[-1],
            index=target_idx + 1,
        )
        result[-1] = stringReplaceIndex(
            new_char="T",
            string=result[-1],
            index=target_idx + 2,
        )

        for a in result:
            print(a)