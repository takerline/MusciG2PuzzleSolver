#!/usr/bin/python3

import rospy
import math
from geometry_msgs.msg import Pose
from std_msgs.msg import String, Bool

from control_robot import ControlRobot

class RobotController:
    def __init__(self):
        # Variables globales para almacenar datos
        self.data_from_topic = []

        # Configuraciones guardadas.

        # Apartar al robot para tomar fotos.
        self.photo = [-0.9833429495440882, -1.4916654390147706, 1.447803799306051, -1.5284587901881714, -1.564780060444967, 0.6537775993347168]
        
        # COnfiguración del borde del ArUco.
        self.initial_configuration = [1.5948715209960938, -1.8853417835631312, 2.091506306325094, -1.7784701786436976, -1.565052334462301, 0.037766531109809875]
        
        # Cofiguraciones de la posición del tablero de cada uno de los tipos de piezas.
        self.circulo = [0.5798825025558472, -0.9648659390262146, 0.8550356070147913, -1.4623824295452614, -1.564845387135641, -0.9243834654437464]
        self.rectangulo = [0.4818132519721985, -1.1419004660895844, 1.105173412953512, -1.5355650273016472, -1.564841095601217, -1.0594633261310022]
        self.cuadrado = [0.18656985461711884, -1.4657758560827752, 1.4103754202472132, -1.5168303784779091, -1.5647795836078089, -1.3708131949054163]
        self.triangulo = [0.3538753390312195, -1.31035931528125, 1.3065574804889124, -1.568545142920204, -1.5648186842547815, -1.2000106016742151]
        self.pentagono = [-0.01072103181947881, -1.5993906460204066, 1.5812905470477503, -1.554252441307046, -1.5647347609149378, -1.5678437391864222]

        #initial_configuration = [2.1953916549682617, -1.776830335656637, 1.7792037169085901, -1.5747095547118128, -1.5647032896624964, -2.5059061686145228]
        #circulo = [1.4932942390441895, -0.7354418796351929, 0.6956236998187464, -1.5324603256634255, -1.5649130980121058, -3.1704841295825403]
        #rectangulo = [-1.2097694444993685, -2.889007183470147, 0.46590411816550026, -2.2881487199401285, 1.5729671048210363, 0.3440684867251832]
        #cuadrado = [-1.436194765280185, -2.906349252303708, 0.48089900974235, -2.285860498570584, 1.573533708745596, -1.4718585309429348]
        #triangulo = [-1.315338436757223, -3.0407010517516078, 0.6889575163470667, -2.360192438165182, 1.5731886625289917, 1.7895170450210571]
        #pentagono = [-1.5573169807463338, -2.8152436653271917, 0.2507632885660138, -2.1473071059355147, 1.5738987347322055, -1.5475563677325608]

        # Inicializar nodo
        #rospy.init_node('order_node', anonymous=True)

        # Subscripción al topic
        rospy.Subscriber('/piece_data', String, self.piece_data_callback)

        # Publicador para nuevas imágenes
        self.new_image_pub = rospy.Publisher('/NewImages', Bool, queue_size=10)
        self.game_status = rospy.Publisher('/game_status', Bool, queue_size=10)

        # Inicializar control del robot
        self.control = ControlRobot()
        rospy.loginfo("Robot listo. Esperando órdenes...")

    def piece_data_callback(self, msg):
        try:
            data = eval(msg.data) 
            if isinstance(data, list):
                self.data_from_topic = data
                if self.data_from_topic == []:
                    self.game_status.publish(True)
                rospy.loginfo(f"Datos recibidos: {data}")
            else:
                rospy.logwarn("Tipo de dato incorrecto. Se espera una lista.")
        except Exception as e:
            rospy.logerr(f"Error al recibir la información: {e}")

    def handle_pose(self, pose_command, pose_aruco):
        # Pose de referencia, todo se mide respecto a ello y es desde donde el robot se mueve.
        pose = pose_aruco
        try:
            if pose_command.startswith("Pieza"):
                coords_str = pose_command.replace("Pieza(", "").replace(")", "")
                coords = tuple(map(float, coords_str.split(';')))

                # Convertir de cm a m.
                pose.position.x += coords[0] / 100.0
                pose.position.y += coords[1] / 100.0
                self.control.move_to_pose(pose)
                rospy.loginfo(f"Moviendo a las coordenadas ({pose.position.x}, {pose.position.y})")
        except ValueError:
            rospy.logwarn(f"Pose no válida: {pose_command}")

    def handle_rotation(self, rotation_command):
        # Obtener configuraciones actuales para sumarles la rotación correpondiente.
        config = self.control.get_motor_angles()
        try:
            angle = float(rotation_command.replace('Rotate', ''))
            rospy.loginfo(f"Rotando {angle} grados.")
            # Pasar de grados a radianes.
            config[5] += math.radians(angle)
        except ValueError:
            rospy.logwarn(f"Rotación no válida: {rotation_command}")
        return config

    # Método para organizar los pasos a dar.
    def create_orders(self):
        if len(self.data_from_topic) > 0:
            orders = []
            counter = 1
            self.control.move_motors(self.photo)

            for piece_data in self.data_from_topic:
                if counter == 1:
                    # Se crean obstáculos para evitar impactos con trípodes.
                    columna = Pose()
                    techo = Pose()

                    columna.position.x = -0.55
                    columna.position.y = 0.0
                    columna.position.z = 0.2
                    columna.orientation.x = 1.55
                    self.control.add_box_to_planning_scene(columna, "columna", (0.2, 0.2, 0.8))
                    
                    techo.position.x = 0.0
                    techo.position.y = 0.58
                    techo.position.z = 0.7
                    self.control.add_box_to_planning_scene(techo, "techo", (0.5, 0.7, 0.2))
                    # También se añade el suelo.
                    self.control.add_floor()

                    # En primer lugar abrir grip y moverse a configuración inicial.
                    orders.append({'Grip': 'Open'})
                    orders.append({'Conf': 'Initial'})

                coords = piece_data[0]
                angle = piece_data[1]
                piece_type = piece_data[2]

                # Situarse sobre la pieza.
                orders.append({'Pose': f'Pieza({coords[0]};{coords[1]})'})

                # En función del tipo de pieza rotar de la forma necesaria.

                if piece_type == 'pentagonos':
                     orders.append({'Conf': f'Rotate{angle - 180}'})
                elif piece_type == 'triangulos':
                    orders.append({'Conf': f'Rotate{angle - 180}'})
                elif piece_type == 'rectangulos':
                    orders.append({'Conf': f'Rotate{angle - 90}'})
                elif piece_type == 'circulos':
                    orders.append({'Conf': f'Rotate{0}'})
                else:
                    orders.append({'Conf': f'Rotate{angle}'})

                # Bajar a por la pieza.
                orders.append({'Pose': 'Go down'})

                # Tomar la pieza.
                orders.append({'Grip': 'Close'})

                # Subir un poco.
                orders.append({'Pose': 'Go up'})

                # Moverse a la configuración del tablero que corresponda en función del tipo de pieza.
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

                # Sobre el tablero bajar más o menos en función del tamaño.    
                if counter == 1:
                    orders.append({'Pose': 'Go down tablero primera'})
                else:
                    orders.append({'Pose': 'Go down tablero'})

                # Abrir grip para soltar pieza y colocarla.
                orders.append({'Grip': 'Open'})

                # Sumar uno para pasar a la siguiente pieza (si la hay).
                counter += 1
                orders.append({'Conf': 'Initial'})
            orders.append({'Conf': 'Photo'})
            orders.append({'Grip': 'Close'})

            # Una vez guardados los pasos a seguir se procesan las órdenes.
            self.process_orders(orders)

            # Una vez finalizado todo, se espera a que el robot se aparte y se pide una nueva imagen.
            rospy.sleep(5)
            self.new_image_pub.publish(True)
            self.data_from_topic = []

    def process_orders(self, orders):
        for element in orders:
            for order_type, order in element.items():
                # Si la orden es moverse a una conf se procesa de una forma.
                if order_type == 'Conf':
                    self.process_configuration(order)

                # Si se trata de poses de otra forma.
                elif order_type == 'Pose':
                    self.process_pose(order)

                # Y si se trata del grip de otra.
                elif order_type == 'Grip':
                    self.process_grip(order)
                    rospy.sleep(2)
                    pass

                rospy.sleep(2)

    # Método para tratar las configuraciones.
    def process_configuration(self, order):
        # Conf del ArUco.
        if order == 'Initial':
            self.control.move_motors(self.initial_configuration)
            self.pose_inicial = self.control.get_pose()
            rospy.loginfo(f"Moviendo a configuración inicial.")
        # Conf para tomar imágenes.
        elif order == 'Photo':
            self.control.move_motors(self.photo)
            rospy.loginfo(f"Moviendo a configuración segura.")
        # Rotaciones.
        elif order.startswith('Rotate'):
            config = self.handle_rotation(order)
            self.control.move_motors(config)
        # Configuraciones de las piezas en el tablero.
        elif order == 'Triangulos':
            self.control.move_motors(self.triangulo)
            rospy.loginfo(f"Moviendo a posición correspondiente a triángulos en el tablero.")
        elif order == 'Cuadrados':
            self.control.move_motors(self.cuadrado)
            rospy.loginfo(f"Moviendo a posición correspondiente a cuadrados en el tablero.")
        elif order == 'Rectangulos':
            self.control.move_motors(self.rectangulo)
            rospy.loginfo(f"Moviendo a posición correspondiente a rectángulos en el tablero.")
        elif order == 'Pentagonos':
            self.control.move_motors(self.pentagono)
            rospy.loginfo(f"Moviendo a posición correspondiente a pentágonos en el tablero.")
        elif order == 'Circulos':
            self.control.move_motors(self.circulo)
            rospy.loginfo(f"Moviendo a posición correspondiente a círculos en el tablero.")

    # Método para mover a otra pose.
    def process_pose(self, order):
        # Bajar a por las piezas.
        if order == 'Go down':
            pose_act = self.control.get_pose()
            pose_act.position.z = 0.228
            self.control.move_to_pose_tries(pose_act)
            rospy.loginfo(f"Bajando a por pieza.")

        # Bajar a dejar las piezas en el tablero (medianas y pequeñas).
        elif order == 'Go down tablero':
            pose_act = self.control.get_pose()
            pose_act.position.z = 0.257
            self.control.move_to_pose_tries(pose_act)
            rospy.loginfo(f"Posando pieza en tablero.")

        # Bajara a dejar las piezas en el tablero (grandes).
        elif order == 'Go down tablero primera':
            pose_act = self.control.get_pose()
            pose_act.position.z = 0.247
            self.control.move_to_pose_tries(pose_act)
            rospy.loginfo(f"Posando pieza en tablero.")

        # Subir al tomar/dejar piezas.
        elif order == 'Go up':
            pose_act = self.control.get_pose()
            pose_act.position.z += 0.1
            self.control.move_to_pose_tries(pose_act)
            rospy.loginfo(f"Subiendo.")

        # Ir a por una pieza.
        elif order.startswith('Pieza'):
            self.handle_pose(order, self.pose_inicial)

    def process_grip(self, order):
        # Abrir el grip.
        if order == 'Open':
            self.control.mover_pinza(100, 15)
            rospy.loginfo(f"Abriendo pinza.")

        # Cerrar el grip.
        elif order == 'Close':
            self.control.mover_pinza(0, 10)
            rospy.loginfo(f"Cerrando pinza.")

    def run(self):
        while not rospy.is_shutdown():
            self.create_orders()
            rospy.sleep(1)

if __name__ == '__main__':
    try:
        robot_controller = RobotController()
        robot_controller.run()
    except rospy.ROSInterruptException:
        pass
