#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Pose, PoseArray
# from proyecto.msg import FloatPose
import copy
import rospy

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose,PoseStamped
from typing import List
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from geometry_msgs.msg import Pose, PoseArray
import copy

import rospy

class ControlRobot:
    def __init__(self) -> None:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("control_robot_2", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = "robot"
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_group_name = "gripper" 
        self.gripper_move_group = MoveGroupCommander(self.gripper_group_name)
        
        columna = Pose()
        techo = Pose()

        columna.position.x = -0.55
        columna.position.y = 0.0
        columna.position.z = 0.2
        columna.orientation.x = 1.55
        self.add_box_to_planning_scene(columna, "columna", (0.2, 0.2, 0.8))
        
        techo.position.x = 0.0
        techo.position.y = 0.58
        techo.position.z = 0.7
        self.add_box_to_planning_scene(techo, "techo", (0.5, 0.7, 0.2))
        # También se añade el suelo.

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
        columna = Pose()
        techo = Pose()
        columna.position.x = -0.45
        columna.position.y = 0.0
        columna.position.z = 0.2
        columna.orientation.x = 1.55
        self.add_box_to_planning_scene(columna, "columna", (0.2, 0.2, 0.8))
        techo.position.x = 0.0
        techo.position.y = 0.58
        techo.position.z = 0.7
        self.add_box_to_planning_scene(techo, "techo", (0.5, 0.7, 0.2))

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