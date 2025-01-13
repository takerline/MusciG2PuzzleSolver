#!/usr/bin/python3

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import rospy
import math

import sys
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import Pose,PoseStamped
import math
from typing import List
from actionlib import SimpleActionClient
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

import control_robot

# Variable globale pour stocker les données reçues
data_from_topic = []

# Callback pour recevoir les données des pièces depuis un topic
def piece_data_callback(msg):
    global data_from_topic
    try:
        # Convertir le message reçu (au format JSON par exemple)
        data = eval(msg.data)  # Attention à la sécurité si les données viennent d'une source externe
        if isinstance(data, list):
            data_from_topic = data
            rospy.loginfo(f"Data received: {data}")
        else:
            rospy.logwarn("Invalid data format received. Expected a list.")
    except Exception as e:
        rospy.logerr(f"Failed to process incoming data: {e}")

# Fonction pour gérer les poses
def handle_pose(pose_command, control):
    pose = control.get_pose()
    try:
        if pose_command.startswith("Pieza"):
            coords_str = pose_command.replace("Pieza(", "").replace(")", "")
            coords = tuple(map(float, coords_str.split(';')))
            pose.position.x += coords[0]/100.0
            pose.position.y += coords[1]/100.0
            print(f"Moving to coordinates: ({pose.position.x}, {pose.position.y})")
            control.move_to_pose(pose)
    except ValueError:
        print(f"Invalid pose command: {pose_command}")

# Fonction pour gérer les rotations
def handle_rotation(rotation_command, control):
    config = control.get_motor_angles()
    try:
        angle = float(rotation_command.replace('Rotate', ''))
        print(f"Rotating by {angle} degrees.")
        config[5] += math.radians(angle)
    except ValueError:
        print(f"Invalid rotation command: {rotation_command}")
    return config

def main():


    global data_from_topic

    control = control_robot.ControlRobot()
    
    #rospy.init_node('order_node', anonymous=True)

    # Souscrire au topic pour recevoir les données
    rospy.Subscriber('/piece_data', String, piece_data_callback)


    photo = [-0.9833429495440882, -1.4916654390147706, 1.447803799306051, -1.5284587901881714, -1.564780060444967, 0.6537775993347168]
    #initial_configuration = [2.1953916549682617, -1.776830335656637, 1.7792037169085901, -1.5747095547118128, -1.5647032896624964, -2.5059061686145228]
    initial_configuration =  [1.5948715209960938, -1.8853417835631312, 2.091506306325094, -1.7784701786436976, -1.565052334462301, 0.037766531109809875]
    circulo =[1.4934119138984767, -0.6353057350872686, 0.2472642862064662, -1.1844178327438015, -1.565240686661542, -3.172128937247056]
    #circulo = [1.4932942390441895, -0.7354418796351929, 0.6956236998187464, -1.5324603256634255, -1.5649130980121058, -3.1704841295825403]
    #rectangulo = [-1.2097694444993685, -2.889007183470147, 0.46590411816550026, -2.2881487199401285, 1.5729671048210363, 0.3440684867251832]
    rectangulo = [1.3882344961166382, -0.6569866699031373, 0.29826051393617803, -1.2138090294650574, -1.564258877431051, 1.3948004245758057]
    #cuadrado = [-1.436194765280185, -2.906349252303708, 0.48089900974235, -2.285860498570584, 1.573533708745596, -1.4718585309429348]
    cuadrado = [1.1679961681365967, -0.6431990426829834, 0.31789905229677373, -1.2469386619380494, -1.5649970213519495, 1.1535685062408447]
    #triangulo = [-1.315338436757223, -3.0407010517516078, 0.6889575163470667, -2.360192438165182, 1.5731886625289917, 1.7895170450210571]
    triangulo = [1.2802895307540894, -0.6506897372058411, 0.26107675233949834, -1.1823463600924988, -1.5638726393329065, -5.053858820592062]
    #pentagono = [-1.5573169807463338, -2.8152436653271917, 0.2507632885660138, -2.1473071059355147, 1.5738987347322055, -1.5475563677325608]
    pentagono = [1.064298152923584, -0.4126228851130982, -0.11839926987886429, -1.0411964815906067, -1.565357510243551, -2.0639222303973597]    
    rospy.loginfo("Robot ready. Waiting for data...")

    while not rospy.is_shutdown():
        if data_from_topic:
            orders = []
            counter = 1
            control.move_motors(photo)

            for piece_data in data_from_topic:
                if counter == 1:
                    columna = Pose()
                    techo = Pose()
                    columna.position.x = -0.40
                    columna.position.y = 0.0
                    columna.position.z = 0.2
                    columna.orientation.x = 1.55
                    control.add_box_to_planning_scene(columna, "columna", (0.2, 0.2, 0.8))
                    techo.position.x = 0.0
                    techo.position.y = 0.58
                    techo.position.z = 0.7
                    control.add_box_to_planning_scene(techo, "techo", (0.5, 0.7, 0.2))
                    control.add_floor()
                    orders.append({'Grip': 'Open'})
                    orders.append({'Conf': 'Initial'})
                coords = piece_data[0]
                angle = piece_data[1]
                piece_type = piece_data[2]

                print(piece_data[2])

                orders.append({'Pose': f'Pieza({coords[0]};{coords[1]})'})
                if piece_type == 'triangulos' or 'pentagonos':
                    orders.append({'Conf': f'Rotate{angle - 180}'})
                orders.append({'Conf': f'Rotate{angle}'})
                orders.append({'Pose': 'Go down'})
                orders.append({'Grip': 'Close'})
                orders.append({'Pose': 'Go up'})
                if piece_type == 'triangulos':
                    orders.append({'Conf': 'Triangulos'})
                elif piece_type == 'cuadrados':
                    orders.append({'Conf': 'Cuadrados'})
                elif piece_type == 'rectangulos':
                    orders.append({'Conf': 'Rectangulos'})
                elif piece_type == 'pentagonos':
                    orders.append({'Conf': 'Pentagonos'})
                elif piece_type == 'circulos':
                    orders.append({'Conf': 'Circulos'})
                    print("qppend cir")
                if counter == 1:
                    orders.append({'Pose': 'Go down tablero primera'})
                else:
                    orders.append({'Pose': 'Go down tablero'})

                orders.append({'Grip': 'Open'})
                #orders.append({'Pose': 'Go up'})
                counter += 1
                orders.append({'Conf': 'Initial'})
            orders.append({'Grip': 'Close'})

            # Traiter les ordres
            for element in orders:
                for order_type, order in element.items():
                    if order_type == 'Conf':
                        if order == 'Initial':
                            control.move_motors(initial_configuration)
                        if order == 'Photo':
                            control.move_motors(photo)
                        if order.startswith('Rotate'):
                            config = handle_rotation(order, control)
                            control.move_motors(config)
                        if order == 'Triangulos':
                            control.move_motors(triangulo)
                        if order == 'Cuadrados':
                            control.move_motors(cuadrado)
                        if order == 'Rectangulos':
                            control.move_motors(rectangulo)
                        if order == 'Pentagonos':
                            control.move_motors(pentagono)
                        if order == 'Circulos':
                            print("circle")
                            control.move_motors(circulo)

                    elif order_type == 'Pose':
                        if order == 'Go down':
                            
                            pose_act = control.get_pose()
                            #pose_act.position.z = 0.22033777251554207
                            pose_act.position.z = 0.228
                            control.move_to_pose_tries(pose_act)

                        if order == 'Go down tablero':
                            
                            pose_act = control.get_pose()
                            #pose_act.position.z = 0.22033777251554207
                            pose_act.position.z = 0.257
                            control.move_to_pose_tries(pose_act)

                        if order == 'Go down tablero primera':
                            pose_act = control.get_pose()
                            #pose_act.position.z = 0.22033777251554207
                            pose_act.position.z = 0.246
                            control.move_to_pose_tries(pose_act)
                        if order == 'Go up':
                            
                            pose_act = control.get_pose()
                            pose_act.position.z += 0.10
                            control.move_to_pose_tries(pose_act)
                        if order.startswith('Pieza'):
                            handle_pose(order, control)

                    elif order_type == 'Grip':
                        if order == 'Open':
                            print("")
                            control.mover_pinza(100, 15)
                        elif order == 'Close':
                            print("")
                            control.mover_pinza(0, 10)
                        rospy.sleep(5)
                    rospy.sleep(2)
            control.move_motors(photo)

            # Réinitialiser les données après traitement
            data_from_topic = []

        rospy.sleep(1)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
