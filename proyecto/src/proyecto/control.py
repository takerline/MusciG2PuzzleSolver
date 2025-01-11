#!/usr/bin/env python
import sys
import copy
import rospy
import yaml
from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, PlanningSceneInterface
import moveit_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String, Int32, Header
from sensor_msgs.msg import JointState
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from practica_1.msg import Floor

class ControlRobot:
    def __init__(self) -> None:
        roscpp_initialize(sys.argv)
        rospy.init_node("control_robot", anonymous=True)
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group_name = 'robot'
        self.move_group = MoveGroupCommander(self.group_name)

    def get_motor_angles(self) -> list: # Obtener ángulos de motores
        return self.move_group.get_current_joint_values()

    def move_motors(self, joint_goal: List[float]) -> bool: # Mover motores
        return self.move_group.go(joint_goal, wait=True) #wait espera hasta finalizar movimiento antes de seguir con el programa.

    def get_pose(self) -> Pose: # Obtener la pose actual
        return self.move_group.get_current_pose().pose

    def move_to_pose(self, pose_goal: Pose)-> bool: # Mover a una pose
        self.move_group.set_pose_target(pose_goal)
        return self.move_group.go(wait=True)

    def add_to_planning_scene(self, pose_caja: Pose,
                              name: str, tamaño: tuple = (.1, .1, .1)) -> None:

        box_pose = PoseStamped()
        box_pose.header.frame_id = 'base_link'
        box_pose.pose = pose_caja
        box_name = name
        self.scene.add_box(box_name, box_pose, size=tamaño)

    def move_trajectory(self, poses: List[Pose], wait: bool = True) -> bool:
        
        (plan, fraction) = self.move_group.compute_cartesian_path(poses, 0.01)

        if fraction != 1.0:
            print("Trayectoria Fallida")
            return False
        
        self.move_group.execute(plan, wait=wait)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.025 # Para no chocar con la base
        self.add_to_planning_scene(pose_suelo, 'suelo', (2, 2, .05)) 

    def action(self) -> None:
        # El nodo suscriptor creado en el __init__ empieza a funcionar
        # gracias al .spin() (se activa el callback).
        rospy.spin()

    def get_initial_conf(self, archivo="/home/laboratorio/ros_workspace/src/proyecto/src/proyecto/confs_y_poses/initial.yaml") -> JointState:
        with open(archivo, "r") as f:
            conf_data = yaml.safe_load(f)
            conf_position = conf_data["configuracion_inicial"]

            conf = JointState()
            conf.header.stamp = rospy.Time.now()
            conf.position = conf_position

        return conf.position

if __name__ == "__main__":
    control = ControlRobot()

    conf_init = control.get_initial_conf()
    control.move_motors(conf_init)

    # Empezamos a ejecutar ordenes asociadas a mensajes que llegan.
    control.action()

    