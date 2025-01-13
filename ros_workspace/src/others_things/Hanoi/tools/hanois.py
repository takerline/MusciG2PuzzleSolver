from typing import List, Dict
from tools.larreaUtils import stringReplaceIndex


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

    def __str__(self):
        return (
            f"Source:{self.source_tower}, Target:{self.target_tower}, "
            f"S-Heigth:{self.source_disc_height}, T-Height:{self.target_disc_height}, "
            f"disc_size:{self.disc_size}"
        )
    def getMatrix(self):
            return [self.source_tower,self.target_tower,self.source_disc_height,self.target_disc_height,self.disc_size]


class HanoiTower:
    def __init__(self, discs):
        self.discs = discs

    def __str__(self):
        return f"{self.discs}"

    def getTopOfTower(self) -> Dict[str, int]:
        result: Dict[str, int] = {"size": 0, "position": 0}
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
        top: Dict[str, int] = self.getTopOfTower()
        result: int = top["size"]
        position: int = top["position"]
        self.discs[position] = 0
        return result

    def placeOnTopOfTower(self, disc: int) -> None:
        top: Dict[str, int] = self.getTopOfTower()
        if (disc < top["size"]) or (top["size"] == 0):
            self.discs[top["position"] - 1] = disc
        elif disc > top["size"]:
            raise Exception(
                f"Hanoi placeOnTopOfTower Error: trying to place bigger:{disc} "
                f"on top of smaller:{top['size']}"
            )
        elif disc == top["size"]:
            raise Exception(
                f"Hanoi placeOnTopOfTower Error: two of the same size: size {disc}"
            )


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
        self.towers = []
        self.discs: int = discs
        self.target_tower: int = target_tower
        self.start_tower: int = start_tower
        self.aux_tower: int = aux_tower
        if standard_start:
            self.towers.append(HanoiTower(list(range(1, discs + 1))))
            for i in range(0, towers - 1):
                tower_discs = [0 for _ in range(discs)]
                self.towers.append(HanoiTower(discs=tower_discs))
        else:
            pass  # Random start not implemented

    def __str__(self):
        result = (
            f"[HANOI] \nTowers:{len(self.towers)}\nDiscs:{self.discs}\nTarget_tower:"
            f"{self.target_tower}\n"
        )
        for i, tower in enumerate(self.towers):
            result += f"Tower {i + 1}: {str(tower)}\n"
        return result

    def checkSolved(self) -> bool:
        current = self.towers[self.target_tower]
        goal = list(range(1, self.discs + 1))
        return f"{current}" == f"{goal}"

    def moveFromTo(self, origin: int, target: int) -> bool:
        disc: int = self.towers[origin].popTopOfTower()
        self.towers[target].placeOnTopOfTower(disc=disc)

    def solveHanoi(self):
        print(
            "Esta es la implementación de la solución tradicional. Uso ilustrativo only."
        )

        def solveHanoiInner(
            disc: int, source_tower: int, destination_tower: int, aux_tower: int
        ):
            if disc == 1:
                print(f"Move the plate from {source_tower} to {destination_tower}")
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

    # Define el resto de métodos igual, ajustando las anotaciones

        
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
        

    def getRandomHanoiSolutionArray(self):
        print("Utiliza esta función en conjunto con processSolution()")
        instructions = []
        
        
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
            varAux = [0,1,2]
            varAux.remove(nextTower)
            varAux.remove(top["tower"])
            #print(f'varAux:{varAux}')
            
            ## continuamos con el 2.2
            fakeHanoi: HanoiGame = HanoiGame(towers=3,discs=top["size"],start_tower=top["tower"],target_tower=nextTower, aux_tower=varAux[0],standard_start=True,)
            movements = fakeHanoi.getStandardHanoiSolutionArray()
            movements = self.processSolution(movements)
            
            for movement in movements:
                instructions.append(movement)
            
                # repeat until solved
        return instructions

    def getStandardHanoiSolutionArray(self):
        # print("Utiliza esta función en conjunto con processSolution()")
        def solveHanoiInnerInstructions(
            disc: int,
            source_tower: str,
            destination_tower: str,
            aux_tower: str,
            pool,
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

        instructions= solveHanoiInnerInstructions(
            disc=self.discs,
            source_tower=self.start_tower,
            destination_tower=self.target_tower,
            aux_tower=self.aux_tower,
            pool=[],
        )
        return instructions

    def processSolution(self, solution):
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
