#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArUcoDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/video_source/raw', Image, self.image_callback)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(e)
            return
        
        # Convertir imagen a escala de grises
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Crear un diccionario de marcadores ArUco
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        
        # Detectar marcadores ArUco en la imagen
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        
        if ids is not None:
            # Dibujar los marcadores detectados en la imagen
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
        
        # Mostrar la imagen resultante
        cv2.imshow('ArUco Detection', cv_image)
        cv2.waitKey(1)
        
def main():
    rospy.init_node('aruco_detection_node')
    node = ArUcoDetectionNode()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass