# #!/usr/bin/env python


# import sys
# import rospy
# import moveit_commander
# from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
# from geometry_msgs.msg import Pose,PoseStamped
# import math
# from typing import List
# from actionlib import SimpleActionClient
# from control_msgs.msg import GripperCommandAction, GripperCommandGoal


# class ControlRobot:
#     def __init__(self) -> None:
#         moveit_commander.roscpp_initialize(sys.argv)
#         rospy.init_node("control_robot", anonymous=True)
#         self.robot = RobotCommander()
#         self.scene = PlanningSceneInterface()
#         self.group_name = "robot"
#         self.move_group = MoveGroupCommander(self.group_name)
#         self.gripper_group_name = "gripper" 
#         self.gripper_move_group = MoveGroupCommander(self.gripper_group_name)
        
#         # Client d'action pour la pince
#         self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
#         self.add_floor()

#     def get_motor_angles(self) -> list:
#         return self.move_group.get_current_joint_values()

#     def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
#         return self.move_group.go(joint_goal, wait=wait)

#     def get_pose(self) -> Pose:
#         return self.move_group.get_current_pose().pose

#     def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
#         self.move_group.set_pose_target(pose_goal)
#         return self.move_group.go(wait=wait)

#     def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
#         box_pose = PoseStamped()
#         box_pose.header.frame_id = "base_link"
#         box_pose.pose = pose_caja
#         self.scene.add_box(name, box_pose, size=tamaño)

#     def move_trajectory(self, poses: List[Pose], wait: bool=True) -> bool:
        
#         waypoints = [self.get_pose()]  

        
#         for pose in poses:
#             waypoints.append(pose)

#         eef_step = 0.01  


        
#         (plan, fraction) = self.move_group.compute_cartesian_path(
#             waypoints,  
#             eef_step   
            
#         )

        
#         if fraction < 1.0:
#             rospy.logwarn("Le plan cartésien n'a pas été calculé entièrement. Fraction: %.2f", fraction)
#             return False

        
#         self.move_group.execute(plan, wait=wait)
#         return True

#     def add_floor(self) -> None:
#         pose_suelo = Pose()
#         pose_suelo.position.z = -0.026
#         self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))

#     def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
#         goal = GripperCommandGoal()
#         goal.command.position = anchura_dedos
#         goal.command.max_effort = fuerza
        
#         # Attendez que le serveur d'action soit disponible
#         self.gripper_action_client.wait_for_server()
        
#         self.gripper_action_client.send_goal(goal)
#         self.gripper_action_client.wait_for_result()
#         result = self.gripper_action_client.get_result()
#         return result.reached_goal



# if __name__ == '__main__':

#     control = ControlRobot()
   

#     home = control.get_pose()


#     #picturePosition = 

#     hanoiTower1 = [1.3915610313415527, -1.1725092691234131, 1.1399858633624476, -1.5397275120816012, -1.5647466818438929, 1.373042345046997]

#     hanoiTower2 = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]

#     hanoiTower3 = [1.0720824003219604, -1.1485782724669953, 1.0744693914996546, -1.4980019640973588, -1.5647185484515589, 1.053541898727417]

#     hanoiTower1_up = [1.3915601925948256, -1.1340504452983395, 0.8449526292457683, -1.2831501110030425, -1.5640829885656997, 1.3711656168526227]
#     hanoiTower2_up = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]
#     hanoiTower3_up = [1.0722912641604738, -1.095657552992958, 0.7543437464556352, -1.2303381850900927, -1.564056227550558, 1.052359306638575]






#     #circulo = [-1.1172803084002894, -2.4601484737791957, -0.5697794556617737, -1.6810199222960414, 1.5753254890441895, 0.04865626245737076]
#     circulo = [-1.1171626036508016, -2.489588926722259, -0.4183369747981829, -1.80239532902398, 1.5750330790340847, 0.0487740814771208]


#     #rectangulo =  [-1.2099936644183558, -2.9969846210875453, 0.6101601759540003, -2.3240682087340296, 1.5735208988189697, 0.34467044472694397]
#     rectangulo = [-1.2097694444993685, -2.889007183470147, 0.46590411816550026, -2.2881487199401285, 1.5729671048210363, 0.3440684867251832]



#      #triangulo =  [-1.3156259695636194, -3.0364886722960414, 0.6880100409137171, -2.362382551232809, 1.573357343673706, -1.372143570576803]
#     #triangulo = [-1.3157341311716557, -2.938884557395238, 0.5664930708477415, -2.3375212635277367, 1.573744176812276, -1.3713000944816411]
#     triangulo = [-1.315338436757223, -3.0407010517516078, 0.6889575163470667, -2.360192438165182, 1.5731886625289917, 1.7895170450210571]
#     #cuadrado =  [-1.436256233845846, -3.0123282871642054, 0.6199458281146448, -2.3184458218016566, 1.5735805034637451, -1.472196404133932]
#     cuadrado = [-1.436194765280185, -2.906349252303708, 0.48089900974235, -2.285860498570584, 1.573533708745596, -1.4718585309429348]

#     #pentagono =  [-1.5571845213519495, -2.9525276623167933, 0.458411995564596, -2.2168127499022425, 1.5739445686340332, -1.5487306753741663]
#     pentagono = [-1.5573169807463338, -2.8152436653271917, 0.2507632885660138, -2.1473071059355147, 1.5738987347322055, -1.5475563677325608]

#     grande = [
#     0.036277213272569414,
#     0.4230686657097666,
#     0.22033777251554207,
#     -0.9999162464840853,
#     -0.012940020115801266,
#     -0.00011269336628698163
#     ]

#     mediano = [
#         0.03303689150924758,
#         0.44073259769192036,
#         0.22558458931373082,
#         -0.9999903657277472,
#         -0.004383415401406698,
#         -0.00011107893553631872    ]

#     pequenio = [
#         0.03063455456872848,
#         0.4481129074931096,
#         0.24301136735694795,
#         -0.999632532145592,
#         -0.02710620316908553,
#         -0.00012658315297034618    ]


#     #Z coordonate of each pieces.
#     #P0_z = 
#     #P1_z = 
#     #P2_z = 
#     #P3_z = 
#     #P4_z = 


#     t1_1 = [-1.374493424092428, -2.1778112850584925, -0.437191903591156, -2.095952173272604, 1.5742331743240356, -1.4433601538287562]
#     t1_2 = [-1.3774264494525355, -2.038260599175924, -0.9830565452575684, -1.6894885502257289, 1.575183391571045, -1.4442108313189905]
#     t2_1 = [-1.2160595099078577, -2.0711685619749964, -0.7529467940330505, -1.8857790432372035, 1.5753809213638306, -1.2431162039386194]
#     t2_2 = [-1.2175233999835413, -2.0248166523375453, -1.0042200088500977, -1.6806789837279261, 1.5758346319198608, -1.2435653845416468]
#     t3_1 = [-1.0644963423358362, -2.2208391628661097, -0.3601789176464081, -2.1299525700011195, 1.5741199254989624, -1.1335981527911585]
#     t3_2 = [-1.0651372114764612, -2.0701753101744593, -0.9336225390434265, -1.7074572048582972, 1.5751261711120605, -1.1320918242083948]
#     tower_positions = {
#         0: [t1_1, t1_2],
#         1: [t2_1, t2_2],
#         2: [t3_1, t3_2]
#     }

#     def down(positionheight: int, size: int) -> None:
#         zAxis = [0.005, 0.005, 0.006, 0.011][5 - size]  # Taille
#         zAxis += [0.035, 0.041, 0.048, 0.055, 0.061][positionheight]  # Hauteur

#         pose_act = control.get_pose()
#         pose_act.position.z -= zAxis
#         control.move_to_pose(pose_act)

#     def MoveAPiece(sourceTower: int, targetTower: int, sourceheight: int, targetheight: int, size: int) -> None:
#         state = "openGriper"
#         while state != "END":
#             if state == "openGriper":
#                 control.mover_pinza(100, 10)
#                 state = "findTowerSource"

#             elif state == "findTowerSource":
#                 state = f"st{sourceTower}"

#             elif state.startswith("st"):
#                 tower_id = int(state[-1])
#                 print(f"#Source : tower {tower_id + 1}")
#                 control.move_motors(*tower_positions[tower_id])
#                 state = "DowntToPieceSource"

#             elif state == "DowntToPieceSource":
#                 down(sourceheight, size)
#                 state = "TakePieceSource"

#             elif state == "TakePieceSource":
#                 control.mover_pinza(0, 5)
#                 state = "UpPieceSource"

#             elif state == "UpPieceSource":
#                 print(f"#Bring piece from tower {sourceTower + 1}")
#                 control.move_motors(tower_positions[sourceTower][0])
#                 state = "FindTargetTower"

#             elif state == "FindTargetTower":
#                 print(f"#Bring piece to tower {targetTower + 1}")
#                 control.move_motors(*tower_positions[targetTower])
#                 state = "ReleasePiece"

#             elif state == "ReleasePiece":
#                 control.mover_pinza(100, 10) 
#                 state = "Goback"

#             elif state == "Goback":
#                 control.move_motors(tower_positions[targetTower][0])
#                 state = "END"

#     list_of_moves =  [[0, 2, 0, 5, 1], [0, 1, 1, 5, 2], [2, 1, 4, 4, 1], [0, 2, 2, 5, 3], [1, 0, 3, 3, 1], [1, 2, 4, 4, 2], [0, 2, 2, 3, 1], [0, 1, 3, 5, 4], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [2, 1, 4, 4, 3], [0, 2, 2, 5, 1], [0, 1, 3, 3, 2], [2, 1, 4, 2, 1], [0, 2, 4, 5, 5], [1, 0, 1, 5, 1], [1, 2, 2, 4, 2], [0, 2, 4, 3, 1], [1, 0, 3, 5, 3], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [1, 2, 4, 4, 4], [0, 2, 2, 3, 1], [0, 1, 3, 5, 2], [2, 1, 2, 4, 1], [0, 2, 4, 3, 3], [1, 0, 3, 5, 1], [1, 2, 4, 2, 2], [0, 2, 4, 1, 1]]

#     while True:
#         input("start ?")
#         for move in list_of_moves:
#             MoveAPiece(*move)



#!/usr/bin/env python


import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose,PoseStamped
import math
from typing import List
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import Pose, PoseArray
import copy

class ControlRobot:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_group_name = "gripper" 
        self.gripper_move_group = MoveGroupCommander(self.gripper_group_name)
        
        # Client d'action pour la pince
        self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
        self.add_floor()

        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)

    def get_motor_angles(self) -> list:
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal: List[float], wait: bool=True) -> bool:
        return self.move_group.go(joint_goal, wait=wait)

    def get_pose(self) -> Pose:
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=wait)

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    def move_trajectory(self, path: List[Pose]) -> None:
        """
        Mueve el robot a través de una trayectoria especificada.

        Args:
            - path (List[Pose]): Lista de poses que definen la trayectoria a seguir.
        """
        path = [copy.deepcopy(pose) for pose in path.poses]
        plan, fraction = self.move_group.compute_cartesian_path(
            path, 0.01,  # waypoints to follow  # eef_step
        )

        if fraction != 1.0:
            print(
                "No se puede completar la trayectoria debido a que solo se puede completar el siguiente porcentaje: ",
                fraction,
            )
            return fraction

        self.move_group.execute(plan, wait=True)
        return None
    
    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.026
        self.add_box_to_planning_scene(pose_suelo, "suelo", (2, 2, .05))

    def mover_pinza(self, anchura_dedos: float, fuerza: float) -> bool:
        goal = GripperCommandGoal()
        goal.command.position = anchura_dedos
        goal.command.max_effort = fuerza
        
        # Attendez que le serveur d'action soit disponible
        self.gripper_action_client.wait_for_server()
        
        self.gripper_action_client.send_goal(goal)
        self.gripper_action_client.wait_for_result()
        result = self.gripper_action_client.get_result()
        return result.reached_goal



if __name__ == '__main__':

    control = ControlRobot()
   

    home = control.get_pose()


    #picturePosition = 

    hanoiTower1 = [1.3915610313415527, -1.1725092691234131, 1.1399858633624476, -1.5397275120816012, -1.5647466818438929, 1.373042345046997]

    hanoiTower2 = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]

    hanoiTower3 = [1.0720824003219604, -1.1485782724669953, 1.0744693914996546, -1.4980019640973588, -1.5647185484515589, 1.053541898727417]

    hanoiTower1_up = [1.3915601925948256, -1.1340504452983395, 0.8449526292457683, -1.2831501110030425, -1.5640829885656997, 1.3711656168526227]
    hanoiTower2_up = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]
    hanoiTower3_up = [1.0722912641604738, -1.095657552992958, 0.7543437464556352, -1.2303381850900927, -1.564056227550558, 1.052359306638575]






    #circulo = [-1.1172803084002894, -2.4601484737791957, -0.5697794556617737, -1.6810199222960414, 1.5753254890441895, 0.04865626245737076]
    circulo = [-1.1171626036508016, -2.489588926722259, -0.4183369747981829, -1.80239532902398, 1.5750330790340847, 0.0487740814771208]


    #rectangulo =  [-1.2099936644183558, -2.9969846210875453, 0.6101601759540003, -2.3240682087340296, 1.5735208988189697, 0.34467044472694397]
    rectangulo = [-1.2097694444993685, -2.889007183470147, 0.46590411816550026, -2.2881487199401285, 1.5729671048210363, 0.3440684867251832]



     #triangulo =  [-1.3156259695636194, -3.0364886722960414, 0.6880100409137171, -2.362382551232809, 1.573357343673706, -1.372143570576803]
    #triangulo = [-1.3157341311716557, -2.938884557395238, 0.5664930708477415, -2.3375212635277367, 1.573744176812276, -1.3713000944816411]
    triangulo = [-1.315338436757223, -3.0407010517516078, 0.6889575163470667, -2.360192438165182, 1.5731886625289917, 1.7895170450210571]
    #cuadrado =  [-1.436256233845846, -3.0123282871642054, 0.6199458281146448, -2.3184458218016566, 1.5735805034637451, -1.472196404133932]
    cuadrado = [-1.436194765280185, -2.906349252303708, 0.48089900974235, -2.285860498570584, 1.573533708745596, -1.4718585309429348]

    #pentagono =  [-1.5571845213519495, -2.9525276623167933, 0.458411995564596, -2.2168127499022425, 1.5739445686340332, -1.5487306753741663]
    pentagono = [-1.5573169807463338, -2.8152436653271917, 0.2507632885660138, -2.1473071059355147, 1.5738987347322055, -1.5475563677325608]

    grande = [
    0.036277213272569414,
    0.4230686657097666,
    0.22033777251554207,
    -0.9999162464840853,
    -0.012940020115801266,
    -0.00011269336628698163
    ]

    mediano = [
        0.03303689150924758,
        0.44073259769192036,
        0.22558458931373082,
        -0.9999903657277472,
        -0.004383415401406698,
        -0.00011107893553631872    ]

    pequenio = [
        0.03063455456872848,
        0.4481129074931096,
        0.24301136735694795,
        -0.999632532145592,
        -0.02710620316908553,
        -0.00012658315297034618    ]


    #Z coordonate of each pieces.
    #P0_z = 
    #P1_z = 
    #P2_z = 
    #P3_z = 
    #P4_z = 


    t1_1 = [-1.37730580965151, -2.041831155816549, -0.9518078565597534, -1.7170382938780726, 1.5750515460968018, -1.4442861715899866]
    t1_2 = [-1.3774264494525355, -2.038260599175924, -0.9830565452575684, -1.6894885502257289, 1.575183391571045, -1.4442108313189905]
    t2_1 = [-1.2174947897540491, -2.0282207928099574, -0.9729140996932983, -1.708386560479635, 1.575901746749878, -1.2437756697284144]
    t2_2 = [-1.2175233999835413, -2.0248166523375453, -1.0042200088500977, -1.6806789837279261, 1.5758346319198608, -1.2435653845416468]
    t3_1 = [-1.0651605764972132, -2.0746122799315394, -0.9000778794288635, -1.736502309838766, 1.5750483274459839, -1.1321924368487757]
    t3_2 = [-1.0651372114764612, -2.0701753101744593, -0.9336225390434265, -1.7074572048582972, 1.5751261711120605, -1.1320918242083948]

    def down( positionheight : int, size : int) -> bool:
        zAxis = 0

        if size == 5 :
            zAxis += 0.011
        if size == 4 :
            zAxis += 0.006
        if size == 3 :
            zAxis += 0.005
        if size == 2 :
            zAxis += 0.005

        if positionheight == 0 :
            zAxis += 0.035
        if positionheight == 1 :
            zAxis += 0.041
        if positionheight == 2 :
            zAxis += 0.048
        if positionheight == 3 :
            zAxis += 0.055
        if positionheight == 4 :
            zAxis += 0.061

        
        pose_act = control.get_pose()
        pose_act.position.z -= zAxis

        control.move_to_pose(pose_act)


    def MoveAPiece(sourceTower : int, targetTower : int, sourceheight: int, targetheight: int, size: int) -> None:
        state = "openGriper"
        while state != "END" :

            if state == "openGriper":
                if size == 5 :
                    control.mover_pinza(90, 20)
                if size == 4 :
                    control.mover_pinza(70, 20)
                if size == 3 :
                    control.mover_pinza(50, 20)
                if size == 2 :
                    control.mover_pinza(40, 20)
                if size == 1 :
                    control.mover_pinza(30, 20)
                # rospy.sleep(3)
                state = "findTowerSource"
                

            if state == "findTowerSource":
                if sourceTower == 0 :
                    state = "st1"
                if sourceTower == 1 :
                    state = "st2"
                if sourceTower == 2 :
                    state = "st3"

            if state == "st1":
                    print("#Source : tower 1")
                    
                    control.move_motors(t1_2)
                    state = "DowntToPieceSource"
            if state == "st2":
                    print("#Source : tower 2")
                    
                    control.move_motors(t2_2)
                    state = "DowntToPieceSource"
            if state == "st3":
                    print("#Source : tower 3")
                    
                    control.move_motors(t3_2)
                    state = "DowntToPieceSource"

            if state == "DowntToPieceSource" :
                down(sourceheight,size)
                state = "TakePieceSource"

            if state == "TakePieceSource" :
                control.mover_pinza(0, 20)
                rospy.sleep(0.5)
                state = "UpPieceSource"  

            
            if state == "UpPieceSource" :
                if sourceTower == 0 :
                    print("#Bring piece from tower 1")
                    # arr = PoseArray()
                    # pose_act = control.get_pose()
                    # arr.poses.append(copy.deepcopy(pose_act))


                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))
                    # control.move_trajectory(arr)
                    #control.move_motors(t1_2)
                    control.move_motors(t1_1)
                if sourceTower == 1 :
                    print("#Bring piece from tower 2")
                    # arr = PoseArray()
                    # pose_act = control.get_pose()
                    # arr.poses.append(copy.deepcopy(pose_act))


                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))
                    # control.move_trajectory(arr)
                    #control.move_motors(t2_2)
                    control.move_motors(t2_1)
                if sourceTower == 2 :
                    print("#Bring piece from tower 3")
                    # arr = PoseArray()
                    # pose_act = control.get_pose()
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # pose_act.position.z += 0.02
                    # arr.poses.append(copy.deepcopy(pose_act))

                    # control.move_trajectory(arr)
                    #control.move_motors(t3_2)
                    control.move_motors(t3_1)
                state = "FindTargetTower"
                  
            if state == "FindTargetTower" :
                if targetTower == 0 :
                    print("#Bring piece to tower 1")
                    control.move_motors(t1_1)
                    control.move_motors(t1_2)
                if targetTower == 1 :
                    print("#Bring piece to tower 2")
                    control.move_motors(t2_1)
                    control.move_motors(t2_2)
                if targetTower == 2 :
                    print("#Bring piece to tower 3")
                    control.move_motors(t3_1)
                    control.move_motors(t3_2)
                state = "ReleasePiece"

            if state == "ReleasePiece" :  
                if size == 5 :
                    control.mover_pinza(90, 20)
                if size == 4 :
                    control.mover_pinza(70, 20)
                if size == 3 :
                    control.mover_pinza(50, 20)
                if size == 2 :
                    control.mover_pinza(40, 20)
                if size == 1 :
                    control.mover_pinza(30, 20)  
                # rospy.sleep(3)       
                state = "Goback"
              
            if state == "Goback" :
                if targetTower == 0 :
                    control.move_motors(t1_1)
                if targetTower == 1 :
                    control.move_motors(t2_1)
                if targetTower == 2 :
                    control.move_motors(t3_1)
                state = "END"

            if state == "END" :
                 break



    list_of_moves = [[0, 2, 0, 5, 1], [0, 1, 1, 5, 2], [2, 1, 4, 4, 1], [0, 2, 2, 5, 3], [1, 0, 3, 3, 1], [1, 2, 4, 4, 2], [0, 2, 2, 3, 1], [0, 1, 3, 5, 4], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [2, 1, 4, 4, 3], [0, 2, 2, 5, 1], [0, 1, 3, 3, 2], [2, 1, 4, 2, 1], [0, 2, 4, 5, 5], [1, 0, 1, 5, 1], [1, 2, 2, 4, 2], [0, 2, 4, 3, 1], [1, 0, 3, 5, 3], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [1, 2, 4, 4, 4], [0, 2, 2, 3, 1], [0, 1, 3, 5, 2], [2, 1, 2, 4, 1], [0, 2, 4, 3, 3], [1, 0, 3, 5, 1], [1, 2, 4, 2, 2], [0, 2, 4, 1, 1]]

    while True :


        input("start ?")
        pose_act = control.get_pose()
        MinZ = home.position.z + 0.02
        joints_initiaux = control.get_motor_angles()
        print(joints_initiaux)
        print(pose_act)
        for move in list_of_moves:
            source_tower, target_tower, source_height, target_height, size = move
            MoveAPiece(source_tower, target_tower, source_height, target_height, size)



  
        