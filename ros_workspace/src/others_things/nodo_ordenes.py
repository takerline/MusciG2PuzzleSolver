#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Int16, Float32
from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import JointState
# from proyecto.msg import FloatPose
from math import pi, tau, dist, fabs, cos
import copy

class GuiaRobot:
    def __init__(
        self,
        nombre_nodo: str = "nodo_global",
        topico_mover_pose: str = "mover_pose",
        topico_mover_configuracion: str = "mover_configuracion",
        topico_trayectoria_cartesiana: str = "trayectoria_cartesiana",
        topico_anadir_obstaculo: str = "anadir_obstaculo",
    ) -> None:
        """
        Inicializa el nodo de ROS y crea los publicadores para los tópicos necesarios.

        Args:
            - nombre_nodo (str): Nombre del nodo de ROS.
            - topico_mover_pose (str): Tópico para publicar la pose del robot.
            - topico_mover_configuracion (str): Tópico para publicar la configuración del robot.
            - topico_trayectoria_cartesiana (str): Tópico para publicar la trayectoria cartesiana del robot.
            - topico_anadir_obstaculo (str): Tópico para publicar la adición de un obstáculo.
        """

        # Inicialización del nodo publicador
        rospy.init_node(nombre_nodo, anonymous=True)

        # Creación de los publicadores
        self.pub_mover_pose = rospy.Publisher(topico_mover_pose, Pose, queue_size=10)
        self.pub_mover_configuracion = rospy.Publisher(
            topico_mover_configuracion, JointState, queue_size=10
        )
        self.pub_trayectoria_cartesiana = rospy.Publisher(
            topico_trayectoria_cartesiana, PoseArray, queue_size=10
        )
        self.pub_anadir_obstaculo = rospy.Publisher(
            topico_anadir_obstaculo, FloatPose, queue_size=10
        )

    def imprimir_menu(self) -> None:
        """
        Imprime un menú en la consola con las opciones disponibles para el control del robot.
        """

        print("#####################################")
        print("       MENU PARA CONTROL ROBOT")
        print("##################################### \n\n")

        print("Seleccione alguna de las siguientes opciones: \n")
        print("1. Añadir obtaculo a la escena de planificacion")
        print(
            "2. Mover el robot a una pose (posición y orientación del extremo del robot)."
        )
        print(
            "3. Mover el robot a una configuración (ángulos de los actuadores del robot)."
        )
        print("4. Mover el extremo del robot por una trayectoria dada.")
        print("5. Detener el programa.")

    def ejecutar_accion(self, opcion) -> None:
        """
        Ejecuta una acción basada en la opción seleccionada por el usuario en el menú.

        Args:
            - opcion (int): La opción seleccionada por el usuario.

            
        """

        # Se valida que el valor ingresado sea numerico
        try:
            opcion = int(opcion)
        except:
            print("\n\nLa opcion elegida no es un numero\n\n")
            return None

        if opcion == 1:
            # Crear y configurar un obstaculo
            objeto = FloatPose()
            objeto.value.data = 0.25
            objeto.pose.position.x += 0.6
            objeto.pose.position.y += 0.8
            objeto.pose.position.z += 0.125

            # Publicar el obstaculo
            self.pub_anadir_obstaculo.publish(objeto)

        elif opcion == 2:
            # Definir y configurar una pose para el robot
            posicion = Pose()
            posicion.position.x = -0.3
            posicion.position.y = 0.3
            posicion.position.z = 0.5

            posicion.orientation.x = -1.0474954199509302e-10
            posicion.orientation.y = -2.1250896069147092e-10
            posicion.orientation.z = -4.429451250231864e-11
            posicion.orientation.w = 1.0

            # Publicar la pose
            self.pub_mover_pose.publish(posicion)

        elif opcion == 3:
            # Configurar y publicar una configuracion de los actuadores del robot
            configuracion = JointState()
            configuracion.name = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
                "finger_joint",
                "left_inner_knuckle_joint",
                "left_inner_finger_joint",
                "right_outer_knuckle_joint",
                "right_inner_knuckle_joint",
                "right_inner_finger_joint",
            ]
            configuracion.position = [0, 0,-pi/2, 0, 1.574, 2.671]

            self.pub_mover_configuracion.publish(configuracion)

        elif opcion == 4:
            # Crear y configurar una trayectoria cartesiana
            array = PoseArray()

            posicion = Pose()
            posicion.position.x = -0.3
            posicion.position.y = 0.3
            posicion.position.z = 0.4

            posicion.orientation.x = -1.0474954199509302e-10
            posicion.orientation.y = -2.1250896069147092e-10
            posicion.orientation.z = -4.429451250231864e-11
            posicion.orientation.w = 1.0

            # Agregar posiciones a la trayectoria
            posicion.position.z += 0.1
            array.poses.append(copy.deepcopy(posicion))

            posicion.position.x += 0.05
            array.poses.append(copy.deepcopy(posicion))

            posicion.position.y += 0.1
            array.poses.append(copy.deepcopy(posicion))

            # Publicar la trayectoria
            self.pub_trayectoria_cartesiana.publish(array)

        elif opcion == 5:
            # Detener el programa
            print("Cerrando programa...")
            self.activo = False

        else:
            print("\n\n[ERROR] La opción elegida no es válida\n\n")

    def correr_programa(self) -> None:
        """
        Ejecuta el bucle principal del programa, mostrando el menú e interactuando con el usuario
        para seleccionar y ejecutar acciones hasta que se detenga el programa o ROS se apague.
        """

        self.activo = True

        while not rospy.is_shutdown() and self.activo:
            self.imprimir_menu()
            opcion = input("\n")

            self.ejecutar_accion(opcion=opcion)

        if rospy.is_shutdown():
            print("El maestro se detuvo")


if __name__ == "__main__":
    nombre_nodo = "menu"

    guia = GuiaRobot(nombre_nodo=nombre_nodo)
    guia.correr_programa()
