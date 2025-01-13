#!/usr/bin/python3


import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotTrajectory
from geometry_msgs.msg import Pose,PoseStamped
import math
from typing import List
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
import copy
#import match



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

    def move_to_pose_tries(self, pose_goal: Pose,attemps: int=5, wait: bool=True) -> bool:
        self.move_group.set_pose_target(pose_goal)
        
        results = []
        for i in range(attemps):
            success, traj, planning_time, error_code = self.move_group.plan()
            if not success: continue
            results.append(traj)
        results.sort(key=lambda x: len(x.joint_trajectory.points))

        return self.move_group.execute(results[0])

    def add_box_to_planning_scene(self, pose_caja: Pose, name: str, tamaño: tuple = (.1, .1, .1)) -> None:
        box_pose = PoseStamped()
        box_pose.header.frame_id = "base_link"
        box_pose.pose = pose_caja
        self.scene.add_box(name, box_pose, size=tamaño)

    #def move_trajectory(self, poses: List[Pose], wait: bool=True) -> bool:
    #    
    #    waypoints = [self.get_pose()]  
#
    #    
    #    for pose in poses:
    #        waypoints.append(pose)
#
    #    eef_step = 0.01  
#
#
    #    
    #    (plan, fraction) = self.move_group.compute_cartesian_path(
    #        waypoints,  
    #        eef_step   
    #        
    #    )
#
    #    
    #    if fraction < 1.0:
    #        rospy.logwarn("Le plan cartésien n'a pas été calculé entièrement. Fraction: %.2f", fraction)
    #        return False
#
    #    
    #    self.move_group.execute(plan, wait=wait)
    #    return True

    def move_trajectory(self, poses: List[Pose], wait: bool = True):
        poses_aux = copy.deepcopy(poses)

        poses_aux.insert(0, self.get_pose())

        (plan, fraction) = self.move_group.compute_cartesian_path(poses_aux, 0.01)

        if fraction != 1.0:
            return False

        return self.move_group.execute(plan, wait=wait)

    def add_floor(self) -> None:
        pose_suelo = Pose()
        pose_suelo.position.z = -0.04
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

#if __name__=='__main__':
#    control = ControlRobot()
#    pose_act = control.get_pose()
#    pose_act.position.z -= .05
#    pose_act.position.x += .15
#    pose_act.position.y += .0
#    control.move_to_pose_tries(pose_act)