#!/usr/bin/python3

import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import cv2
from piece_detection import PieceDetection  
from queue import Queue
from cv_bridge import CvBridge

from validate_detection import detection_validation

class PieceDetectionNode:
    def __init__(self):
        # Inicializar el nodo ROS
        rospy.init_node('piece_detection_node', anonymous=True)

        # Configurar atributos
        self.images_data = Queue(maxsize=20)
        self.all_data = []
        self.processed = False

        # Inicializar detección de piezas con archivo de calibración
        self.detection = PieceDetection(calibration_file='/home/laboratorio/ros_workspace/src/proyecto/final/new_calibration.pkl')

        # Inicializar el puente CvBridge
        self.bridge = CvBridge()

        # Configurar el publicador y los suscriptores
        self.piece_data_pub = rospy.Publisher('/piece_data', String, queue_size=10)
        rospy.Subscriber('/cam2/usb_cam_estiven_ASUS_TUF_Gaming_A15_FA506ICB_FA506ICB_383628_2901270562350196763/image_raw', 
                         Image, self.image_callback)
        rospy.Subscriber('/NewImages', Bool, self.new_images_callback)
        rospy.Subscriber('pieces_orders', Bool, self.new_images_callback)


    # Función callback para recibir las imágenes
    def image_callback(self, msg):
        """Recibe y procesa imágenes desde el tópico correspondiente."""
        try:
            # Convertir imagen de ROS a OpenCV
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')  
            
            # Agregar la imagen a la cola solo si no está llena
            if self.images_data.full():
                #rospy.loginfo("Se ha alcanzado el límite de imágenes. No se agregarán más.")
                return None
            self.images_data.put(image)
            rospy.loginfo(f"Imagen {self.images_data.qsize()} recibida y almacenada.")
        except Exception as e:
            rospy.logerr(f"Error: {e}")

    # Función callback para recibir el mensaje True desde /NewImages
    def new_images_callback(self, msg):
        """Recibe True desde /NewImages y reinicia el proceso de toma de imágenes."""
        if msg.data:
            rospy.loginfo("Recibido mensaje 'True' en /NewImages. Vaciar cola e iniciar el nuevo ciclo.")

            # Vaciar la cola de imágenes y la lista de datos procesados
            self.images_data = Queue(maxsize=20)
            self.all_data.clear()

            # Reiniciar la bandera de procesamiento
            self.processed = False

    # Función para procesar la imagen
    def process_images(self):
        if self.images_data.qsize() == 20 and not self.processed:
            rospy.loginfo("Procesando las 20 imágenes...")

            # Procesar todas las imágenes de la cola
            for image in list(self.images_data.queue):
                cv2.imwrite("PRUEBA_FINAL.png",image)
                current_data = self.detection.process_image(image)
                self.all_data.append(current_data)

            # Validar los resultados de la detección
            if len(self.all_data[0]) > 0:   
                final_data = detection_validation(self.all_data)
            else:
                final_data = []

            # Publicar los datos de las piezas
            if final_data:
                data_msg = String()
                data_msg.data = str(final_data)
                self.piece_data_pub.publish(data_msg)
                rospy.loginfo("Datos procesados y publicados en /piece_data")
            else:
                rospy.logwarn("No se detectaron piezas en las imágenes.")

            # Marcar como procesado para evitar más publicaciones
            self.processed = True

    def run(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.process_images()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = PieceDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass