#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Int16, Int32
from tools import hanois, larreaUtils
from tools.location_circles import DetectHanoiTower
from tools.supplements import ControlRobot

from geometry_msgs.msg import Pose, PoseArray
# from proyecto.msg import FloatPose
from sensor_msgs.msg import JointState
from math import pi, tau, dist, fabs, cos
import copy
import tf.transformations as tf
from std_msgs.msg import Bool
from queue import Queue
import pickle
from tools.hanois import HanoiGame, HanoiTower
import json

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, tau, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from typing import List
from geometry_msgs.msg import Pose
import yaml
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from actionlib import SimpleActionClient

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('father', anonymous=True)

    master = rospy.Publisher(
    "CurrentPart", Int32, queue_size=10
    )
    entry = 2
    while entry != -1:
        entry = int(input("Orden a enviar: "))
        
        value = Int32()
        value.data = entry

        master.publish(value)
