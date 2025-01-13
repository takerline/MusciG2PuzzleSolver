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
import keyboard


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

    def move_trajectory(self, poses: List[Pose], wait: bool=True) -> bool:
        
        waypoints = [self.get_pose()]  

        
        for pose in poses:
            waypoints.append(pose)

        eef_step = 0.01  


        
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  
            eef_step   
            
        )

        
        if fraction < 1.0:
            rospy.logwarn("Le plan cartésien n'a pas été calculé entièrement. Fraction: %.2f", fraction)
            return False

        
        self.move_group.execute(plan, wait=wait)
        return True

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

    #hanoiTower1 = 

    #hanoiTower2 = 

    #hanoiTower3 = 

    circulo = [-1.1172803084002894, -2.4601484737791957, -0.5697794556617737, -1.6810199222960414, 1.5753254890441895, 0.04865626245737076]

    rectangulo =  [-1.2099936644183558, -2.9969846210875453, 0.6101601759540003, -2.3240682087340296, 1.5735208988189697, 0.34467044472694397]

    triangulo =  [-1.3156259695636194, -3.0364886722960414, 0.6880100409137171, -2.362382551232809, 1.573357343673706, -1.372143570576803]

    cuadrado =  [-1.436256233845846, -3.0123282871642054, 0.6199458281146448, -2.3184458218016566, 1.5735805034637451, -1.472196404133932]

    pentagono =  [-1.5571845213519495, -2.9525276623167933, 0.458411995564596, -2.2168127499022425, 1.5739445686340332, -1.5487306753741663]

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





    while True :



        pose_act = control.get_pose()
        MinZ = home.position.z + 0.02
        joints_initiaux = control.get_motor_angles()

        print(joints_initiaux)
        print(pose_act)




        if keyboard.is_pressed('space'):
            match state:
                case "start":
                    input("start")
                    control.move_to_pose(pose_act)
                    state = "move_up"

                case "move_up":
                    input("haut")
                    control.mover_pinza(100, 10)
                    pose_act.position.z += 0.05
                    control.move_to_pose(pose_act)
                    state = "move_down"

                case "move_down":
                    input("bas")
                    pose_act.position.z = MinZ
                    control.move_to_pose(pose_act)
                    state = "close_gripper"

                case "close_gripper":
                    input("pince")
                    control.mover_pinza(0, 10)
                    state = "move_up2"

                case "move_up2":
                    input("move_up2")
                    pose_act.position.z += 0.05
                    control.move_to_pose(pose_act)
               
                    state = "triangle"

                case "triangle":
                    input("triangle")
                    control.move_to_pose(triangulo)
                    state = "move_down_2"

                case "move_down_2":
                    input("bas")
                    pose_act.position.z = MinZ
                    control.move_to_pose(pose_act)
                    state = "open_gripper"

                case "open_gripper":
                    input("pince")
                    control.mover_pinza(100, 10)
                    state = "end"

                case "end":
                    input("end")
                    control.mover_pinza(0, 10)

                    pose_act.position.z += 0.05
                    control.move_to_pose(pose_act)
                    control.move_to_pose(home)
                    print("Fin de la séquence.")
                    break

                case _:
                    print("Error In State Machine Stopping.")
                    break







        #input("Rotation du poignet +10 degrés...") 
        #joints_modifies = joints_initiaux.copy() 
        #joints_modifies[5] += math.radians(360) 
        #control.move_motors(joints_modifies)
        # Conversion en radians  input("Retour à la position initiale...") control.move_motors(joints_initiaux)
        #input("-10degres")
        
        #control.move_to_pose(pose_save)
