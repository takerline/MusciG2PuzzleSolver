import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImagePublisherNode:
    def __init__(self):
        # Inicializa el nodo con un nombre único
        rospy.init_node('image_publisher_node', anonymous=False)
        
        # Publicador al tema de destino
        self.pub_info = rospy.Publisher('/CurrentImage', Image, queue_size=10)

        self.bridge = CvBridge()

        # Suscriptor al tema de origen (imágenes de la cámara)
        rospy.Subscriber('/cam2/usb_cam_estiven_ASUS_TUF_Gaming_A15_FA506ICB_FA506ICB_451992_1637286455608554950/image_raw', Image, self.image_callback)

        # Variable para almacenar la última imagen recibida
        self.current_image = None
        

    def image_callback(self, msg):
        # Callback que recibe las imágenes
        try:
            self.current_image = msg
            rospy.loginfo('Imagen recibida desde la cámara')
        except Exception as e:
            rospy.logerr(f"Error al obtener la imagen recibida: {e}")

    def run(self):
        rate = rospy.Rate(5)  

        while not rospy.is_shutdown():
            if self.current_image is not None:
                try:
                    # Publica la última imagen recibida
                    self.pub_info.publish(self.current_image)
                    rospy.loginfo('Imagen publicada en /CurrentImage')
                except Exception as e:
                    rospy.logerr(f"Error al publicar la imagen: {e}")

            rate.sleep()

if __name__ == '__main__':
    try:
        # Crea una instancia del nodo y ejecuta el bucle
        image_relay = ImagePublisherNode()
        image_relay.run()
    except rospy.ROSInterruptException:
        pass
