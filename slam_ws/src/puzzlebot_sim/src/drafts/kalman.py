#!/usr/bin/env python

""" Implement a node that calculates the odometry for the puzzlebot robot.
"""

import numpy as np
import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovariance,TwistWithCovariance, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Int32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Pose2D

R, L = 0.05, 0.18 # radius and distance between wheels
FS = 90 # rate
T = 1.0/FS # frequency

STD_DEV = 0.001

# Pose covariance gains
Kl = 0.8
Kr = 0.95

class LocalisationNode():
    def __init__(self):

        rospy.init_node("localisation_deadreckoning")

        #Subscribe to arucos pose topic
        rospy.Subscriber("/aruco_id", Int32, self._aruco_callback)
        self.aruco_id = None
        self.m = {  18:{'x': 1, 'y': 2},
                    12:{'x': 2, 'y': 5},
                    5: {'x': -1, 'y': 9}}
        self.M = []

        rospy.Subscriber("/aruco_pose", Pose2D, self._aruco_pose_callback)
        self.aruco_x = None
        self.aruco_y = None
        self.aruco_th


        # Subscribe to wheel angular velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        self.wl = None
        self.wr = None

        # Subscribe to velocities topic
        # TODO: Revisar si es necesario, sino, quitarlo. 
        rospy.Subscriber("/cmd_vel", Twist, self._vel_callback)
        # self.v = 0.0
        # self.w = 0.0
                
        # Publish to the odometry topic
        self.pOdom = rospy.Publisher("/odom", Odometry, queue_size=10)

        # Publish to the joint states topic
        self.pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
       
        # Service for reset robot state values
        ser = rospy.Service("/reset", Empty, self._ser_callback)

        # States of the mobile robot
        self.miu = np.array([[0.0], [0.0], [0.0]]) # Best estimate of the pose (initial conditions)
        self.s = np.array([[0.0], [0.0], [0.0]]) # Robot pose (initial conditions)
        self.model_pose = PoseWithCovariance()
        self.model_twist = TwistWithCovariance()

        # Covariance matrices
        self.sigma = np.zeros((3, 3)) # Initial covariance matrix
        self.pose_cov = np.zeros((6, 6)) # Pose covariance for Odometry message
        self.twist_cov = np.zeros((6, 6)) # Twist covariance for Odometry message
              
        # For broadcasting transform from base_link to map
        self.tb = tf.TransformBroadcaster()

        # Rate    
        self.rate = rospy.Rate(FS)

        self.contador = 0
        
    def _wl_callback(self, msg):
        self.wl = msg.data
        
    def _wr_callback(self, msg):
        self.wr = msg.data

    def _vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def _ser_callback(self, req):
        self.x = 0.0
        self.y = 0.0
        self.q = 0.0
        return EmptyResponse()
    
    def _aruco_callback(self, msg):
        self.arucos_id = msg.data
        if self.aruco_id not in self.M:
            self.M.append(self.arucos_id)
    
    # def _EKF(self, M, miu_k, sigma_k, z_i, R_k):

    #     s_k = miu_k + H_k * (s_k_1 - )

    #     if z_i in M:
    #         z_i = 
    #         G_K = np.array([])
    #         Z_k = np.dot(np.dot(G_K, sigma_est), G_K.T) + R_k
    #         K_k = np.dot(np.dot(sigma_est, G_K.T), np.linalg.inv(Z_k))
    #         miu_k = miu_est + np.dot(K_k, (zi_k - z_est))
    #         sigma_k = np.dot(I - np.dot(K_k, G_K), sigma_est)

    #     return miu_k, sigma_k
          
    def main(self):

        t0 = rospy.Time.now()
        f1 = R/L # To save some calculations
        f2 = 0.5*R # To save some calculations
        v = 0 # Linear velocity
        w = 0 # Angular velocity

        while not rospy.is_shutdown():

            # TODO: Cambiar el robot_pose por lo nuevo (Es con el resultado del filtro?)
            # Pose parameters
            self.model_pose.pose.position.x = self.miu[0][0]
            self.model_pose.pose.position.y = self.miu[1][0]
            # Quaternion
            qRota = tf.transformations.quaternion_from_euler(0, 0, self.miu[2][0])
            self.model_pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
            # Pose covariance
            self.model_pose.covariance[:] = np.ravel(self.pose_cov)

            # Twist parameters
            self.model_twist.twist.linear.x = v
            self.model_twist.twist.angular.z = w
            # Twist covariance
            self.model_twist.covariance[:] = np.ravel(self.twist_cov)

            # Publish the transform
            t = TransformStamped()
            self.tb.sendTransform([self.miu[0][0], self.miu[1][0], 0], qRota, rospy.Time.now(), "base_link", "map")

            # Publish the joint state
            if (self.wl is not None) and (self.wr is not None):
                js = JointState()
                js.name = ['base_to_right_w', 'base_to_left_w']
                t = rospy.Time.now() - t0
                js.position = [self.wr*t.to_sec(), self.wl*t.to_sec()]
                js.header.stamp = rospy.Time.now()
                self.pJS.publish(js)

            # Publish the odometry
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "map"
            odom.child_frame_id = "base_link"
            odom.pose = self.model_pose
            odom.twist = self.model_twist
            self.pOdom.publish(odom)

            if (self.wl is not None) and (self.wr is not None):
                
                w = (self.wr-self.wl) * f1 # Compute of linear velocity
                v = (self.wr+self.wl) * f2 # Compute of angular velocity
                
                dd = T*v # To save some calculations
                dth = T*w # To save some calculations
                
                miu_k_1 = self.miu # best estimation (k - 1)
                s_k_1 = self.s # robot pose (k - 1)

                # ----- Prediction step -----

                # TODO: Preguntar si los caluclos se hacen con miu o con s
                # Best estimation update
                displacement_vector = np.array([[dd * np.cos(miu_k_1[2][0])],
                                                [dd * np.sin(miu_k_1[2][0])],
                                                [dth]])
                self.miu = miu_k_1 + displacement_vector
                
                # 3x3 Linear model Jacobian of the robot
                H = np.array([[1.0, 0.0, -dd * np.sin(miu_k_1[2][0])],
                              [0.0, 1.0, dd * np.cos(miu_k_1[2][0])],
                              [0.0, 0.0, 1.0]])
                
                # Jacobian for covariance calculation
                j_w = 0.5*R*T * np.array([[np.cos(miu_k_1[2][0]), np.cos(miu_k_1[2][0])],
                                          [np.sin(miu_k_1[2][0]), np.sin(miu_k_1[2][0])],
                                          [2/L, -2/L]])
                
                # TODO: Preguntar si esl ruido de la covarianza tambien se agrega en el modelo real
                # Noise (tunned according to the test of mini challenge 1)
                sigma_j = np.array([[Kr*np.abs(self.wr), 0],
                                    [0, Kl*np.abs(self.wl)]]) 
                
                # Pose covariance matrix Q_k
                Q = np.dot(np.dot(j_w, sigma_j), j_w.T)
                #Q =(j_w * sigma_j) * j_w.T

                # Update covariance matrix
                self.sigma = np.dot(np.dot(H, self.sigma), H.T) + Q
                
                #self.sigma = (H * self.sigma * H.T) + Q
                
                # Assignment of covariance values
                self.pose_cov[0][0] = self.sigma[0][0]
                self.pose_cov[0][1] = self.sigma[0][1]
                self.pose_cov[1][0] = self.sigma[1][0]
                self.pose_cov[1][1] = self.sigma[1][1]
                self.pose_cov[0][5] = self.sigma[0][2]
                self.pose_cov[1][5] = self.sigma[1][2]
                self.pose_cov[5][0] = self.sigma[2][0]
                self.pose_cov[5][1] = self.sigma[2][1]
                self.pose_cov[5][5] = self.sigma[2][2]

                self.s = self.miu + H*(s_k_1 - miu_k_1)

                # TODO: Implementar funcion para el filtro de Kalman
                # ----- Correction step -----
                for M in self.M:
                    if M in self.m:
                        print("Mx",self.m[M]['x'])
                        print("My",self.m[M]['y'])

                        print("miuX",self.miu[0][0])
                        print("miuY",self.miu[1][0])

                        dx = self.m[M]['x'] - self.miu[0][0]
                        dy = self.m[M]['y'] - self.miu[1][0]
                        p = dx**2 + dy**2

                        z_rho = np.sqrt((dx)**2 + (dy)**2)
                        z_alpha = np.arctan2(dy, dx) - self.miu[2][0]



                        #print(b1)
                        #print(b2)

                        #print(z_alpha)
                        #print(z_rho)

                        

                        #print(z_alpha)
                        #print(z_rho)

                        ez = np.array([[z_rho], [z_alpha]])


                        #print(ez.shape)
                        #print(p)

                        G = np.array([[-(dx/np.sqrt(p)), -(dy/np.sqrt(p)), 0.0], 
                                    [dy/p, -(dx/p), -1.0]])
                        
                        # TODO: Preugntar de donde sale la R = ?
                        RR = np.array([[0.1, 0],[0, 0.02]])
                        Z = np.dot(np.dot(G, self.sigma), G.T) + RR
                        # Z = np.dot(G, self.sigma)*(G.T) + RR
                        # print("sigma", self.sigma)
                        # print("Sigma det",np.linalg.det(self.sigma))
                        # print("G.T", G.T)
                        # print("G", G)
                        # print("Z", Z)
                        #if np.linalg.det(Z) > 0:
                        K = np.dot(np.dot(self.sigma, G.T), np.linalg.inv(Z))

                        # K = self.sigma*(G.T)*np.linalg.inv(Z)

                        z = ez + np.dot(G, (self.s - self.miu))

                        #z = ez + G*(self.s - self.miu)
                        # print(self.miu.shape)
                        # print(z.shape)
                        # print(ez.shape)
                        # print(K.shape)

                        self.miu = self.miu + np.dot(K,(z - ez))
                        #self.miu = self.miu + K*(z - ez)

                        I = np.eye(H.shape[0])
                        
                        self.sigma = np.dot((I - np.dot(K, G)),self.sigma)
                        #self.sigma = (I - (K*G)*self.sigma)
                    



            self.rate.sleep()
                    
if __name__ == '__main__':

    try:
        aux = LocalisationNode()
        aux.main()

    except rospy.ROSInterruptException:
        pass