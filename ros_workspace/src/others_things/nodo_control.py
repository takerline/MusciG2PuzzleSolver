#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Int16
from tools import ControlRobot
from geometry_msgs.msg import Pose, PoseArray
# from proyecto.msg import FloatPose
from sensor_msgs.msg import JointState
from math import pi, tau, dist, fabs, cos
import copy
import tf.transformations as tf
from std_msgs.msg import Bool



class DriverRobot(ControlRobot):
    def __init__(self, nombre_nodo: str = "nodo_global"):
        """
        Inicializa el nodo de ROS

        Args:
            - nombre_nodo (str): Nombre del nodo de ROS.
        """
        super().__init__(nombre_nodo)
        self.pub_moving = rospy.Publisher(
            "moving", Bool, queue_size=10
        )

    def callback_mover_pose(self, data: Pose) -> None:
        """
        Callback para mover el robot a una pose dada.

        Args:
            - data (Pose): La pose a la que se desea mover el robot.
        """
        pose_inicial = self.get_pose()
        print("Pose inicial: \n\n", pose_inicial)

        self.move_to_pose(data)

        pose_final = self.get_pose()
        print("Pose final: \n\n", pose_final)

    # def callback_agregar_objeto(self, data: FloatPose, nombre: str = "box") -> None:
    #     """
    #     Callback para agregar un objeto a la escena de planificacion.

    #     Args:
    #         - data (FloatPose): Datos del objeto a agregar, incluyendo su pose.
    #         - nombre (str): Nombre del objeto que se va a agregar. Por defecto es 'box'.
    #     """
    #     tamano = tuple([data.value.data] * 3)

    #     print("Agregando obstaculo con dimensiones: ", tamano)

    #     self.add_to_planning_scene(data.pose, nombre, tamano=tamano)

    def callback_mover_configuracion(self, data: JointState) -> None:
        """
        Callback para mover el robot a una configuracion de motores dada.

        Args:
            - data (JointState): Estados de los motores con las posiciones deseadas.
        """
        print("Configuracion inicial: \n\n", data)

        posiciones = list(data.position[:6])
        self.pub_moving.publish(True)
        self.move_motors(posiciones)
        self.pub_moving.publish(False)

        configuracion_final = self.get_motor_angles()
        print("Configuracion final: \n\n", configuracion_final)

    def callback_mover_trajectoria(self, data: PoseArray) -> None:
        """
        Callback para mover el robot a traves de una trayectoria dada.

        Args:
            - data (PoseArray): Array de poses que definen la trayectoria a seguir.
        """
        # Se mueve el robot a la primera posicion de la trajectoria
        self.move_to_pose(data.poses[0])

        posicion_inicial = self.get_pose()
        print("punto de trajectoria inicial: \n\n", posicion_inicial)

        self.move_to_trajectory(data.poses)

        pose_final = self.get_pose()
        print("Pose final de trayectoria cartesiana: \n\n", pose_final)

    def sub_agregar_objeto(self, nombre_topic: str = "anadir_obstaculo") -> None:
        """
        Suscribe el nodo a un topico para agregar objetos a la escena de planificacion.

        Args:
            - nombre_topic (str): Nombre del topico de ROS para agregar objetos.
        """
        # rospy.Subscriber(nombre_topic, FloatPose, self.callback_agregar_objeto)

    def sub_mover_pose(self, nombre_topic: str = "mover_pose") -> None:
        """
        Suscribe el nodo a un topico para mover el robot a una pose especifica.

        Args:
            - nombre_topic (str): Nombre del topico de ROS para mover el robot.
        """
        rospy.Subscriber(nombre_topic, Pose, self.callback_mover_pose)

    def sub_mover_configuracion(
        self, nombre_topic: str = "mover_configuracion"
    ) -> None:
        """
        Suscribe el nodo a un topico para mover el robot a una configuracion especifica.

        Args:
            - nombre_topic (str): Nombre del topico de ROS para mover la configuracion del robot.
        """
        rospy.Subscriber(nombre_topic, JointState, self.callback_mover_configuracion)

    def sub_mover_trayectoria(
        self, nombre_topic: str = "trayectoria_cartesiana"
    ) -> None:
        """
        Suscribe el nodo a un topico para mover el robot a traves de una trayectoria.

        Args:
            - nombre_topic (str): Nombre del topico de ROS para mover a lo largo de una trayectoria.
        """
        rospy.Subscriber(nombre_topic, PoseArray, self.callback_mover_trajectoria)

    def escuchar_topics(self) -> None:
        """
        Mantiene el nodo activo, escuchando los topicos de ROS hasta que se detenga.
        """
        rospy.spin()


if __name__ == "__main__":
    # Inicializa el nombre del nodo y los topicos de ROS
    nombre_nodo = "nodo_mover_pose"

    topico_anadir_obstaculo = "anadir_obstaculo"
    nombre_topic_mover_pose = "mover_pose"
    topico_mover_configuracion = "mover_configuracion"
    topico_trajectoria_cartesiana = "trayectoria_cartesiana"

    # Crea una instancia del controlador de robot
    mover_pose = DriverRobot(nombre_nodo)

    # Suscribe a los topicos
    mover_pose.sub_agregar_objeto(topico_anadir_obstaculo)
    mover_pose.sub_mover_pose(nombre_topic_mover_pose)
    mover_pose.sub_mover_configuracion(topico_mover_configuracion)
    mover_pose.sub_mover_trayectoria(topico_trajectoria_cartesiana)

    # Comienza a escuchar los topicos
    mover_pose.escuchar_topics()
