from tools.hanois import HanoiGame, HanoiTower, HanoiMovement
from tools.location_circles import DetectHanoiTower

def testHanoiTower() -> None:
    tower0: HanoiTower = HanoiTower(discs=[1, 2, 3, 4])
    tower1: HanoiTower = HanoiTower(discs=[0, 0, 0, 0])
    tower2: HanoiTower = HanoiTower(discs=[0, 0, 0, 0])

    disc = tower0.popTopOfTower()
    # print(f'popped disc:{disc}')
    tower2.placeOnTopOfTower(disc=disc)
    disc = tower0.popTopOfTower()
    tower1.placeOnTopOfTower(disc=disc)

    print(tower0)
    print(tower1)
    print(tower2)


def testHanoiGameBase():
    game = HanoiGame(
        towers=3,
        discs=8,
        standard_start=True,
        start_tower=0,
        aux_tower=1,
        target_tower=2,
    )
    game.towers[0] = HanoiTower(discs=[0, 0, 0, 0, 0, 2, 3, 4])
    game.towers[2] = HanoiTower(discs=[0, 0, 0, 1, 5, 6, 7, 8])
    game.towers[1] = HanoiTower(discs=[0, 0, 1, 2, 3, 4, 5, 6])
    game.paintHanoi()


def testHanoiPopAndPlace():
    game = HanoiGame(towers=3, discs=8, target_tower=2, standard_start=True)
    print(game.towers[0].getTopOfTower())
    disc = game.towers[0].popTopOfTower()
    game.towers[2].placeOnTopOfTower(disc)
    print(game.checkSolved())
    game.paintHanoi()


def testHanoiFromTo():
    game = HanoiGame(towers=3, discs=8, target_tower=2, standard_start=True)
    game.moveFromTo(
        target=2,
        origin=0,
    )


def testHanoiSolverExample():
    game = HanoiGame(towers=3, discs=5, target_tower=2, standard_start=True)
    game.solveHanoi()


def testHanoiSolverAlpha():
    game = HanoiGame(towers=3, discs=5, target_tower=2, standard_start=True)
    solutions = game.getStandardHanoiSolutionArray()
    for i in solutions:
        print(i)


def testHanoiSolverBeta():
    game = HanoiGame(towers=3, discs=5, target_tower=2, standard_start=True)
    game.paintHanoi()
    solutions = game.getStandardHanoiSolutionArray()
    solution = game.processSolution(solution=solutions)
    for i in solution:
        print(i)
    game.paintHanoi()
    
def testHanoiRandomSolver():
    game = HanoiGame(towers=3, discs=5,target_tower=0,start_tower=1,aux_tower=2,standard_start=False)
    game.towers.append(HanoiTower(discs=[0,0,0,2,5]))
    game.towers.append(HanoiTower(discs=[0,0,0,1,4]))
    game.towers.append(HanoiTower(discs=[0,0,0,0,3]))
    game.paintHanoi()
    solutions = game.getRandomHanoiSolutionArray()
    for sol in solutions:
        print(sol)
    print(game.checkSolved())
    game.paintHanoi()
    

if __name__ == "__main__":
    image_file = 'imagenes_5_dic_lab_2/imagen_20241205_204151.png'
    detect_hanoi = DetectHanoiTower()
    output = detect_hanoi.main(image_file)
    output = output.tolist()
    print("Salida formateada", output)

    game = HanoiGame(towers=3, discs=5,target_tower=2,start_tower=1,aux_tower=2,standard_start=False)
    game.towers.append(HanoiTower(discs=output[0]))
    game.towers.append(HanoiTower(discs=output[1]))
    game.towers.append(HanoiTower(discs=output[2]))
    game.paintHanoi()
    solutions = game.getRandomHanoiSolutionArray()
    print(solutions)
    for sol in solutions:
        print(sol)
    print(game.checkSolved())
    game.paintHanoi()
