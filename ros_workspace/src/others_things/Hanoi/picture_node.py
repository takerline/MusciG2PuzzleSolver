import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
from std_msgs.msg import String
import pickle
import base64
import cv2

bridge = CvBridge()

class NodoCamara:
    def __init__(self) -> None:
        rospy.init_node('nodo_camara')
        rospy.Subscriber(
            '/cam1/usb_cam_estiven_ASUS_TUF_Gaming_A15_FA506ICB_FA506ICB_9900_7373562842126146576/image_raw',
            Image,
            self.__cb_image
        )

        self.pub_info = rospy.Publisher(
            "CurrentImage", String, queue_size=10
        )

        self.cv_image = None

    def __cb_image(self, image: Image):
        self.cv_image = bridge.imgmsg_to_cv2(image, desired_encoding='passthrough')

    def run(self):
        
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if self.cv_image is not None:
                imagen = deepcopy(self.cv_image)
                # Serializamos la imagen con pickle y codificamos con base64.
                try:
                    imagen = cv2.cvtColor(imagen, cv2.COLOR_RGB2BGR)
                    print(imagen.shape)
                    cv2.imwrite('Original.png',imagen)
                    serialized_image = base64.b64encode(pickle.dumps(imagen)).decode('utf-8')
                    # Publicamos la imagen codificada como String.
                    self.pub_info.publish(serialized_image)
                except Exception as err:
                    rospy.logerr(f"Fallo al publicar: {err}")
                print(imagen.shape)
            rate.sleep()
    
if __name__ == "__main__":
    nodo = NodoCamara()
    nodo.run()
