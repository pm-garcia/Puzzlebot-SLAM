#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import numpy as np

class ArUcoDetectionNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.pose_pub = rospy.Publisher('/aruco_pose', Pose2D, queue_size=10)

        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            rospy.logerr(e)
            return
        
        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Convertir imagen a escala de grises
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Crear un diccionario de marcadores ArUco
            aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            aruco_params = cv2.aruco.DetectorParameters_create()
            
            # Detectar marcadores ArUco en la imagen
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
            
            if len(corners) > 0:
                distances = []
                for corner in corners:

                    rvecs, tvecs= cv2.aruco.estimatePoseSingleMarkers(corner, 0.40, self.camera_matrix, self.dist_coeffs)
                    tvec = tvecs[0]
                    distance = np.linalg.norm(tvec)
                    distances.append(distance)
                
                closest_marker_idx = np.argmin(distances)

                closest_corner = corners[closest_marker_idx]
                rvecs, tvecs= cv2.aruco.estimatePoseSingleMarkers(closest_corner, 0.40, self.camera_matrix, self.dist_coeffs)
                rvec = rvecs[0]
                tvec = tvecs[0]
                #print(tvecs)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                pose = np.concatenate((rotation_matrix, tvec.T), axis=1)

                x = tvec[0][2]
                y = tvec[0][0]

                pose_msg = Pose2D()
                pose_msg.x = x
                pose_msg.y = y
                self.pose_pub.publish(pose_msg)
            
            if ids is not None:
                # Dibujar los marcadores detectados en la imagen
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs, tvecs, 0.01)
            
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
