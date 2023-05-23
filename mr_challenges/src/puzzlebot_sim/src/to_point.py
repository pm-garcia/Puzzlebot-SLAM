#!/usr/bin/env python
import rospy, tf
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
import numpy as np

class Control():
    
    def __init__(self):
        # Inicializar nodo
        rospy.init_node("to_point")
        self.rate = rospy.Rate(90)

        # Ganancias proporcionales
        self.kp_v = 0.1
        self.kp_w = 0.4

        # Contantes auxiliares
        self.stop_v, self.stop_w = 1, 1

        # Subscriptor a la posicion 
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        self.th_R = None
        self.x_R = None
        self.y_R = None

        # Publicadores para el theta_error, distance_error y cmd_vel
        self.eth_pub = rospy.Publisher("/error/theta", Float32, queue_size = 1)
        self.e_theta = 0.0

        self.edis_pub = rospy.Publisher("/error/distance", Float32, queue_size = 1)
        self.e_distance = 0.0

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.robot_cmd = Twist()

        rospy.on_shutdown(self.endCallback)

    # Callback para la posicion
    def _odom_callback(self, msg):
        self.x_R = msg.pose.pose.position.x
        self.y_R = msg.pose.pose.position.y
        self.q = msg.pose.pose.orientation
        _, _, self.th_R = tf.transformations.euler_from_quaternion([self.q.x, self.q.y, self.q.z, self.q.w])

    # Callback para detener el robot
    def endCallback(self):
        self.robot_cmd.linear.x = 0.0
        self.robot_cmd.angular.z = 0.0
        self.cmd_pub.publish(self.robot_cmd)
    
    def error(self, x_T, y_T):
        if (self.th_R is not None) and (self.x_R is not None) and (self.y_R is not None):
            
            # Calculo del error en theta
            self.e_theta = np.arctan2(y_T - self.y_R, x_T - self.x_R) - self.th_R
    
            # Calculo del error en la distancia
            self.e_distance = np.sqrt((x_T - self.x_R)**2 + (y_T - self.y_R)**2)
            
            # Pub de los errores 
            self.eth_pub.publish(self.e_theta)
            self.edis_pub.publish(self.e_distance)
            #self.rate.sleep()

    def controller(self):
        # Calculo de la velocidad linear y angular
        v = self.kp_v*self.e_distance

        if abs(self.kp_w*self.e_theta) > 0.5:
            w = 0.5
        else:
            w = self.kp_w*self.e_theta

        # Rango de exito para detenerse
        if (abs(self.e_distance) < 0.1):
            v = 0.0
        if (abs(self.e_theta) < 0.045):
            w = 0.0

        # Pub de las velocidades
        self.robot_cmd.linear.x = v
        self.robot_cmd.angular.z = w
        print("Velocidad lineal = " + str(v))
        print("Velocidad angular = " + str(w))
        print("Error de distancia = " + str(self.e_distance))
        print("Error de angulo = " + str(self.e_theta))
        self.cmd_pub.publish(self.robot_cmd)
        #self.rate.sleep()

    def main(self):
        while not rospy.is_shutdown():  
            self.rate.sleep()
            self.error(-1, 1)
            self.controller()

if __name__ == '__main__':

    try:
        control = Control()
        control.main()
    except rospy.ROSInterruptException:
        pass
