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
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
from std_msgs.msg import String
import pickle
import base64

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
import cv2

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
from std_msgs.msg import String
from driver import ControlRobot
import os

bridge = CvBridge()

class HanoiNode():
    def __init__(self, name_node: str = "nodo_global", topic_orders: str = "hanoi_orders", 
                 topic_images: str = "CurrentImage", topic_movements:  str = 'HanoiMvmt') -> None:
        """
        Inicializa el nodo de ROS

        Args:
            - nombre_nodo (str): Nombre del nodo de ROS.
        """
        rospy.init_node(name_node)

        # # Definir una cola para gestionar el hilo del suscriber
        self.order_data = False

        # # Se define cola para gestionar imagenes
        self.images_data = []

        self.topic_images = topic_images

        # Empezar a leer ordenes del nodo maestro
        rospy.Subscriber(topic_orders, Bool, self.callback_read_orders)
        
        # Empezar a leer las imagenes de la camara
        rospy.Subscriber(
            '/cam1/usb_cam_estiven_ASUS_TUF_Gaming_A15_FA506ICB_FA506ICB_83457_8746564968562245146/image_raw',
            Image,
            self.__cb_image
        )

        # Leer imagenes
        self.group_images = []

        self.hanoi_mvmt = rospy.Publisher(
            topic_movements, String, queue_size=10
        )
        self.cv_image = []

    def callback_read_orders(self, data: Int32) -> None:
        """
        Callback para mover el robot a una pose dada.

        Args:
            - data (Pose): La pose a la que se desea mover el robot.
        """
        self.order_data = data.data

    def __cb_image(self, image: Image):
        self.cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')
        # image_rgb = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        # cv2.imshow("Imagen",image_rgb)
        # cv2.waitKey(1)

    def validate_detections(self) -> None:
        voting = {}

        for k, detect in enumerate(self.detections):
            # Asumiendo que detect es un array de NumPy de 2x3
            row_sums = detect.sum(axis=1)
            
            # Convertir las sumas a una tupla para poder usarla como clave del diccionario
            key = tuple(row_sums)
            
            if key in voting:
                voting[key][1] += 1
            else:
                voting[key] = [k, 1]

        #  Encontrar la configuración más votada basándose en el conteo (segunda posición)
        most_voted_key = max(voting, key=lambda x: voting[x][1])
        
        # Obtener el índice k correspondiente a la configuración más votada
        most_voted_k = voting[most_voted_key][0]
        
        return self.detections[most_voted_k].tolist()


    def listen_topics(self) -> None:
        """
        Mantiene el nodo activo, escuchando los topicos de ROS hasta que se detenga.
        """
        
        # files_images = os.listdir('/home/laboratorio/ros_workspace/src/proyecto/final/images')
        # self.detections = []
        # for k, file in enumerate(files_images):
        #     #cv2.imwrite(f'imagen_{k}.png',img)
        #     detect_hanoi = DetectHanoiTower()
            
        #     try:
        #         output,detections,_,_ = detect_hanoi.main('/home/laboratorio/ros_workspace/src/proyecto/final/images/'+file)
        #         output = output[::-1]
        #         cv2.imwrite(f'detection_{k}.png',detections)
        #     except Exception as err:
        #         print("Error: ",err)
        #         continue                
        #     self.detections.append(output)
        
        # self.group_images = []

        # try:
        #     most_votated = self.validate_detections()
        # except Exception as err:
        #     print("Error: ",err)

        # game = HanoiGame(towers=3, discs=5,target_tower=2,start_tower=1,aux_tower=2,standard_start=False)
        # game.towers.append(HanoiTower(discs=most_votated[0]))
        # game.towers.append(HanoiTower(discs=most_votated[1]))
        # game.towers.append(HanoiTower(discs=most_votated[2]))
        # game.paintHanoi()

        # solutions = game.getRandomHanoiSolutionArray()

        # sendToMaylen = []
        # for sol in solutions:
        #     sendToMaylen.append(sol.getMatrix())

        # print("Salidas: ",sendToMaylen)
        # msg = String()
        # msg.data = str(sendToMaylen)
        # self.hanoi_mvmt.publish(msg)
        # print("debbug")
        # #return sendToMaylen

        #rate = rospy.Rate(10)  
        while not rospy.is_shutdown():

            self.detections = []

            if self.order_data is True:
                #print(self.order_data)
                
                if len(self.group_images) == 30:
                    self.order_data = False
                    #self.group_images = []

                    #print("Imagenes: ",self.images_data)
                    for k, img in enumerate(self.group_images):
                        cv2.imwrite(f'imagen_{k}.png',img)
                        detect_hanoi = DetectHanoiTower()
                        
                        try:
                            output,detections,_,_ = detect_hanoi.main(img)
                            output = output[::-1]
                            cv2.imwrite(f'detection_{k}.png',detections)
                        except Exception as err:
                            print("Error: ",err)
                            continue                
                        self.detections.append(output)
                    
                    self.group_images = []

                    try:
                        most_votated = self.validate_detections()
                    except Exception as err:
                        print("Error: ",err)
                        continue

                    game = HanoiGame(towers=3, discs=5,target_tower=2,start_tower=1,aux_tower=2,standard_start=False)
                    game.towers.append(HanoiTower(discs=most_votated[0]))
                    game.towers.append(HanoiTower(discs=most_votated[1]))
                    game.towers.append(HanoiTower(discs=most_votated[2]))
                    game.paintHanoi()

                    solutions = game.getRandomHanoiSolutionArray()

                    sendToMaylen = []
                    for sol in solutions:
                        sendToMaylen.append(sol.getMatrix())

                    print("Salidas: ",sendToMaylen)
                    msg = String()
                    msg.data = str(sendToMaylen)
                    self.hanoi_mvmt.publish(msg)
                    print("debbug")
                    #return sendToMaylen
                
                else:
                    if len(self.cv_image) > 0:
                        image = deepcopy(self.cv_image)
                        print("Tamano de imagen: ",image.shape)
                        image_rgb = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                        self.group_images.append(image_rgb)

if __name__ == '__main__':
    hanoi_node = HanoiNode(name_node='process_hanoi')
    movements = hanoi_node.listen_topics()






  
        




























# #!/usr/bin/python3

# import rospy
# from std_msgs.msg import String, Int16, Int32
# from tools import hanois, larreaUtils
# from tools.location_circles import DetectHanoiTower
# from tools.supplements import ControlRobot

# from geometry_msgs.msg import Pose, PoseArray
# # from proyecto.msg import FloatPose
# from sensor_msgs.msg import JointState
# from math import pi, tau, dist, fabs, cos
# import copy
# import tf.transformations as tf
# from std_msgs.msg import Bool
# from queue import Queue
# import pickle
# from tools.hanois import HanoiGame, HanoiTower
# import json
# import rospy
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image
# from copy import deepcopy
# from std_msgs.msg import String
# import pickle
# import base64

# import sys
# import rospy
# import moveit_commander
# from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
# from geometry_msgs.msg import Pose,PoseStamped
# import math
# from typing import List
# from actionlib import SimpleActionClient
# from control_msgs.msg import GripperCommandAction, GripperCommandGoal
# from geometry_msgs.msg import Pose, PoseArray
# import copy
# import cv2

# hanoiTower1 = [1.3915610313415527, -1.1725092691234131, 1.1399858633624476, -1.5397275120816012, -1.5647466818438929, 1.373042345046997]

# hanoiTower2 = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]

# hanoiTower3 = [1.0720824003219604, -1.1485782724669953, 1.0744693914996546, -1.4980019640973588, -1.5647185484515589, 1.053541898727417]

# hanoiTower1_up = [1.3915601925948256, -1.1340504452983395, 0.8449526292457683, -1.2831501110030425, -1.5640829885656997, 1.3711656168526227]
# hanoiTower2_up = [1.203183650970459, -1.2344353956035157, 1.1677129904376429, -1.5045202535441895, -1.5634601751910608, 1.167273998260498]
# hanoiTower3_up = [1.0722912641604738, -1.095657552992958, 0.7543437464556352, -1.2303381850900927, -1.564056227550558, 1.052359306638575]

# #circulo = [-1.1172803084002894, -2.4601484737791957, -0.5697794556617737, -1.6810199222960414, 1.5753254890441895, 0.04865626245737076]
# circulo = [-1.1171626036508016, -2.489588926722259, -0.4183369747981829, -1.80239532902398, 1.5750330790340847, 0.0487740814771208]


# #rectangulo =  [-1.2099936644183558, -2.9969846210875453, 0.6101601759540003, -2.3240682087340296, 1.5735208988189697, 0.34467044472694397]
# rectangulo = [-1.2097694444993685, -2.889007183470147, 0.46590411816550026, -2.2881487199401285, 1.5729671048210363, 0.3440684867251832]



#     #triangulo =  [-1.3156259695636194, -3.0364886722960414, 0.6880100409137171, -2.362382551232809, 1.573357343673706, -1.372143570576803]
# #triangulo = [-1.3157341311716557, -2.938884557395238, 0.5664930708477415, -2.3375212635277367, 1.573744176812276, -1.3713000944816411]
# triangulo = [-1.315338436757223, -3.0407010517516078, 0.6889575163470667, -2.360192438165182, 1.5731886625289917, 1.7895170450210571]
# #cuadrado =  [-1.436256233845846, -3.0123282871642054, 0.6199458281146448, -2.3184458218016566, 1.5735805034637451, -1.472196404133932]
# cuadrado = [-1.436194765280185, -2.906349252303708, 0.48089900974235, -2.285860498570584, 1.573533708745596, -1.4718585309429348]

# #pentagono =  [-1.5571845213519495, -2.9525276623167933, 0.458411995564596, -2.2168127499022425, 1.5739445686340332, -1.5487306753741663]
# pentagono = [-1.5573169807463338, -2.8152436653271917, 0.2507632885660138, -2.1473071059355147, 1.5738987347322055, -1.5475563677325608]

# grande = [
# 0.036277213272569414,
# 0.4230686657097666,
# 0.22033777251554207,
# -0.9999162464840853,
# -0.012940020115801266,
# -0.00011269336628698163
# ]

# mediano = [
#     0.03303689150924758,
#     0.44073259769192036,
#     0.22558458931373082,
#     -0.9999903657277472,
#     -0.004383415401406698,
#     -0.00011107893553631872    ]

# pequenio = [
#     0.03063455456872848,
#     0.4481129074931096,
#     0.24301136735694795,
#     -0.999632532145592,
#     -0.02710620316908553,
#     -0.00012658315297034618    ]

# t1_1 = [-1.37730580965151, -2.041831155816549, -0.9518078565597534, -1.7170382938780726, 1.5750515460968018, -1.4442861715899866]
# t1_2 = [-1.3774264494525355, -2.038260599175924, -0.9830565452575684, -1.6894885502257289, 1.575183391571045, -1.4442108313189905]
# t2_1 = [-1.2174947897540491, -2.0282207928099574, -0.9729140996932983, -1.708386560479635, 1.575901746749878, -1.2437756697284144]
# t2_2 = [-1.2175233999835413, -2.0248166523375453, -1.0042200088500977, -1.6806789837279261, 1.5758346319198608, -1.2435653845416468]
# t3_1 = [-1.0651605764972132, -2.0746122799315394, -0.9000778794288635, -1.736502309838766, 1.5750483274459839, -1.1321924368487757]
# t3_2 = [-1.0651372114764612, -2.0701753101744593, -0.9336225390434265, -1.7074572048582972, 1.5751261711120605, -1.1320918242083948]


# def down( positionheight : int, size : int) -> bool:
#     zAxis = 0

#     if size == 5 :
#         zAxis += 0.011
#     if size == 4 :
#         zAxis += 0.006
#     if size == 3 :
#         zAxis += 0.005
#     if size == 2 :
#         zAxis += 0.005

#     if positionheight == 0 :
#         zAxis += 0.035
#     if positionheight == 1 :
#         zAxis += 0.041
#     if positionheight == 2 :
#         zAxis += 0.048
#     if positionheight == 3 :
#         zAxis += 0.055
#     if positionheight == 4 :
#         zAxis += 0.061

    
#     pose_act = control.get_pose()
#     pose_act.position.z -= zAxis

#     control.move_to_pose(pose_act)


# def MoveAPiece(sourceTower : int, targetTower : int, sourceheight: int, targetheight: int, size: int) -> None:
#     state = "openGriper"
#     while state != "END" :

#         if state == "openGriper":
#             if size == 5 :
#                 control.mover_pinza(90, 20)
#             if size == 4 :
#                 control.mover_pinza(70, 20)
#             if size == 3 :
#                 control.mover_pinza(50, 20)
#             if size == 2 :
#                 control.mover_pinza(40, 20)
#             if size == 1 :
#                 control.mover_pinza(30, 20)
#             rospy.sleep(3)
#             state = "findTowerSource"
            

#         if state == "findTowerSource":
#             if sourceTower == 0 :
#                 state = "st1"
#             if sourceTower == 1 :
#                 state = "st2"
#             if sourceTower == 2 :
#                 state = "st3"

#         if state == "st1":
#                 print("#Source : tower 1")
                
#                 control.move_motors(t1_2)
#                 state = "DowntToPieceSource"
#         if state == "st2":
#                 print("#Source : tower 2")
                
#                 control.move_motors(t2_2)
#                 state = "DowntToPieceSource"
#         if state == "st3":
#                 print("#Source : tower 3")
                
#                 control.move_motors(t3_2)
#                 state = "DowntToPieceSource"

#         if state == "DowntToPieceSource" :
#             down(sourceheight,size)
#             state = "TakePieceSource"

#         if state == "TakePieceSource" :
#             control.mover_pinza(0, 20)
#             rospy.sleep(0.5)
#             state = "UpPieceSource"  

        
#         if state == "UpPieceSource" :
#             if sourceTower == 0 :
#                 print("#Bring piece from tower 1")
#                 control.move_motors(t1_1)
#             if sourceTower == 1 :
#                 print("#Bring piece from tower 2")

#                 control.move_motors(t2_1)
#             if sourceTower == 2 :
                
#                 print("#Bring piece from tower 3")
#                 control.move_motors(t3_1)
#             state = "FindTargetTower"
                
#         if state == "FindTargetTower" :
#             if targetTower == 0 :
#                 print("#Bring piece to tower 1")
#                 control.move_motors(t1_1)
#                 control.move_motors(t1_2)
#             if targetTower == 1 :
#                 print("#Bring piece to tower 2")
#                 control.move_motors(t2_1)
#                 control.move_motors(t2_2)
#             if targetTower == 2 :
#                 print("#Bring piece to tower 3")
#                 control.move_motors(t3_1)
#                 control.move_motors(t3_2)
#             state = "ReleasePiece"

#         if state == "ReleasePiece" :  
#             if size == 5 :
#                 control.mover_pinza(90, 20)
#             if size == 4 :
#                 control.mover_pinza(70, 20)
#             if size == 3 :
#                 control.mover_pinza(50, 20)
#             if size == 2 :
#                 control.mover_pinza(40, 20)
#             if size == 1 :
#                 control.mover_pinza(30, 20)  
#             rospy.sleep(3)       
#             state = "Goback"
            
#         if state == "Goback" :
#             if targetTower == 0 :
#                 control.move_motors(t1_1)
#             if targetTower == 1 :
#                 control.move_motors(t2_1)
#             if targetTower == 2 :
#                 control.move_motors(t3_1)
#             state = "END"

#         if state == "END" :
#                 break

# class ControlRobot:
#     def __init__(self) -> None:
#         moveit_commander.roscpp_initialize(sys.argv)
#         #rospy.init_node("control_robot", anonymous=True)
#         self.robot = RobotCommander()
#         self.scene = PlanningSceneInterface()
#         self.group_name = "robot"
#         self.move_group = MoveGroupCommander(self.group_name)
#         self.gripper_group_name = "gripper" 
#         self.gripper_move_group = MoveGroupCommander(self.gripper_group_name)
        
#         # Client d'action pour la pince
#         self.gripper_action_client = SimpleActionClient("rg2_action_server", GripperCommandAction)
#         self.add_floor()

#         self.move_group.set_max_velocity_scaling_factor(1)
#         self.move_group.set_max_acceleration_scaling_factor(1)

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

#     def move_trajectory(self, path: List[Pose]) -> None:
#         """
#         Mueve el robot a través de una trayectoria especificada.

#         Args:
#             - path (List[Pose]): Lista de poses que definen la trayectoria a seguir.
#         """
#         path = [copy.deepcopy(pose) for pose in path.poses]
#         plan, fraction = self.move_group.compute_cartesian_path(
#             path, 0.01,  # waypoints to follow  # eef_step
#         )

#         if fraction != 1.0:
#             print(
#                 "No se puede completar la trayectoria debido a que solo se puede completar el siguiente porcentaje: ",
#                 fraction,
#             )
#             return fraction

#         self.move_group.execute(plan, wait=True)
#         return None
    
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


# class HanoiNode():
#     def __init__(self, name_node: str = "nodo_global", topic_orders: str = "CurrentPart", 
#                  topic_images: str = "CurrentImage", order_start: int = 2) -> None:
#         """
#         Inicializa el nodo de ROS

#         Args:
#             - nombre_nodo (str): Nombre del nodo de ROS.
#         """
#         rospy.init_node('nodo_hanoi')

#         # # Definir una cola para gestionar el hilo del suscriber
#         self.orders_data = 0

#         # # Se define cola para gestionar imagenes
#         self.images_data = []

#         self.topic_images = topic_images

#         # Empezar a leer ordenes
#         rospy.Subscriber(topic_orders, Int32, self.callback_read_orders)
        
#         rospy.Subscriber(self.topic_images, String, self.callback_capture_images)
        

#         # Leer imagenes
#         self.five_images = []

#         self.hanoi_mvmt = rospy.Publisher(
#             "HanoiMvmt", String, queue_size=10
#         )

#         # Orden para empezar el proceso de deteccion
#         self.order_start = order_start

#     def callback_read_orders(self, data: Int32) -> None:
#         """
#         Callback para mover el robot a una pose dada.

#         Args:
#             - data (Pose): La pose a la que se desea mover el robot.
#         """
#         self.orders_data = data.data

#     def callback_capture_images(self, data: String) -> None:
#         """
#         Callback para mover el robot a una pose dada.

#         Args:
#             - data (Pose): La pose a la que se desea mover el robot.
#         """
#         image_bytes = base64.b64decode(data.data)
#         image = pickle.loads(image_bytes)
#         #print(image.shape)
#         #cv2.imwrite("salida.png",image)

#         order = self.orders_data
#         if order == self.order_start:
#             print(order)
            
#             if len(self.five_images) == 5:
#                 self.images_data += self.five_images
#                 self.five_images = []
#                 self.orders_data = 0
            
#             else:
#                 self.five_images.append(image)

#     def validate_detections(self) -> None:
#         voting = {}

#         for k, detect in enumerate(self.detections):
#             # Asumiendo que detect es un array de NumPy de 2x3
#             row_sums = detect.sum(axis=1)
            
#             # Convertir las sumas a una tupla para poder usarla como clave del diccionario
#             key = tuple(row_sums)
            
#             if key in voting:
#                 voting[key][1] += 1
#             else:
#                 voting[key] = [k, 1]

#         #  Encontrar la configuración más votada basándose en el conteo (segunda posición)
#         most_voted_key = max(voting, key=lambda x: voting[x][1])
        
#         # Obtener el índice k correspondiente a la configuración más votada
#         most_voted_k = voting[most_voted_key][0]
        
#         return self.detections[most_voted_k].tolist()


#     def listen_topics(self) -> None:
#         """
#         Mantiene el nodo activo, escuchando los topicos de ROS hasta que se detenga.
#         """

#         #rate = rospy.Rate(10)  
#         while not rospy.is_shutdown():
            
#             self.detections = []

#             if self.images_data:
#                 print("Imagenes: ",self.images_data)
#                 for k, img in enumerate(self.images_data):
#                     cv2.imwrite(f'imagen_{k}.png',img)
#                     detect_hanoi = DetectHanoiTower()
#                     try:
#                         output,_,_,_ = detect_hanoi.main(img)
#                     except:
#                         continue                
#                     self.detections.append(output)
                
#                 self.images_data = []

#                 try:
#                     most_votated = self.validate_detections()
#                 except:
#                     continue

#                 game = HanoiGame(towers=3, discs=5,target_tower=0,start_tower=1,aux_tower=2,standard_start=False)
#                 game.towers.append(HanoiTower(discs=most_votated[0]))
#                 game.towers.append(HanoiTower(discs=most_votated[1]))
#                 game.towers.append(HanoiTower(discs=most_votated[2]))
#                 game.paintHanoi()
#                 solutions = game.getRandomHanoiSolutionArray()

#                 sendToMaylen = []
#                 for sol in solutions:
#                     sendToMaylen.append(sol.getMatrix())
#                 #response = json.dumps(solutions)
#                 print("Respuesta: ",sendToMaylen)

#                 return sendToMaylen


# if __name__ == '__main__':

#     hanoi_node = HanoiNode(name_node='hanoiNode2')
#     movements = hanoi_node.listen_topics()
    
#     control = ControlRobot()

#     home = control.get_pose()

#     #list_of_moves = [[0, 2, 0, 5, 1], [0, 1, 1, 5, 2], [2, 1, 4, 4, 1], [0, 2, 2, 5, 3], [1, 0, 3, 3, 1], [1, 2, 4, 4, 2], [0, 2, 2, 3, 1], [0, 1, 3, 5, 4], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [2, 1, 4, 4, 3], [0, 2, 2, 5, 1], [0, 1, 3, 3, 2], [2, 1, 4, 2, 1], [0, 2, 4, 5, 5], [1, 0, 1, 5, 1], [1, 2, 2, 4, 2], [0, 2, 4, 3, 1], [1, 0, 3, 5, 3], [2, 1, 2, 4, 1], [2, 0, 3, 4, 2], [1, 0, 3, 3, 1], [1, 2, 4, 4, 4], [0, 2, 2, 3, 1], [0, 1, 3, 5, 2], [2, 1, 2, 4, 1], [0, 2, 4, 3, 3], [1, 0, 3, 5, 1], [1, 2, 4, 2, 2], [0, 2, 4, 1, 1]]
#     list_of_moves = movements

#     pose_act = control.get_pose()
#     MinZ = home.position.z + 0.02
#     joints_initiaux = control.get_motor_angles()
#     print(joints_initiaux)
#     print(pose_act)

#     while len(movements) > 0:
#         source_tower, target_tower, source_height, target_height, size = movements[0]
#         MoveAPiece(source_tower, target_tower, source_height, target_height, size)

#         hanoi_node.orders_data = 2
#         movements = hanoi_node.listen_topics()


