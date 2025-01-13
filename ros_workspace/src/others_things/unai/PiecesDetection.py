#!/usr/bin/python3

import rospy
from std_msgs.msg import String
import cv2
from piece_detection import PieceDetection  # Importer la classe

import rospy
from std_msgs.msg import String, Int16, Int32

from geometry_msgs.msg import Pose, PoseArray
# from proyecto.msg import FloatPose
from sensor_msgs.msg import JointState
from math import pi, tau, dist, fabs, cos
import copy
import tf.transformations as tf
from std_msgs.msg import Bool
from queue import Queue
import pickle
import json
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from copy import deepcopy
from std_msgs.msg import String
import pickle
import base64


def callback_capture_images(data: String) -> None:
    global image
    """
    Callback para mover el robot a una pose dada.

    Args:
        - data (Pose): La pose a la que se desea mover el robot.
    """


    image_bytes = base64.b64decode(data.data)
    image = pickle.loads(image_bytes)
    #print(image.shape)
    cv2.imwrite("salida.png",image)
    

def main():
    global image
    image = None
    # Initialiser le nœud ROS
    rospy.init_node('piece_detection_node', anonymous=True)
    img_2 = []
    
    rospy.Subscriber('CurrentImage', String, callback_capture_images)

    # Publisher pour envoyer les données des pièces
    piece_data_pub = rospy.Publisher('/piece_data', String, queue_size=10)

    # Créer une instance de la classe PieceDetection
    detection = PieceDetection(calibration_file='new_calibration.pkl')

    # Charger une image à traiter
    # image_path = 'test_pentagonos.png'
    # img = cv2.imread(image_path)

    while image is None:
        pass
    
    if len(img_2) == 0:
        img_2.append(image)
    img = img_2[0]
    if img is None:
        rospy.logerr("Impossible de charger l'image. Vérifiez le chemin.")
        return

    # Traiter l'image pour détecter les pièces
    final_data = detection.process_image(img)

    # Publier les données des pièces
    if final_data:
        # Convertir les données en chaîne JSON pour les publier
        data_msg = String()
        data_msg.data = str(final_data)  # Utiliser JSON si besoin
        piece_data_pub.publish(data_msg)
        rospy.loginfo("Données des pièces publiées sur /piece_data")
    else:
        rospy.logwarn("Aucune pièce détectée. Rien à publier.")

    # Garder le nœud actif
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass