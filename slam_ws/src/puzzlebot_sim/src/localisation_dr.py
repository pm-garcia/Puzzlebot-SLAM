#!/usr/bin/env python

""" Implement a node that calculates the odometry for the puzzlebot robot.
"""

import numpy as np
import rospy, tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Quaternion, PoseWithCovariance,TwistWithCovariance, TransformStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse

R, L = 0.05, 0.18
FS = 90
T = 1.0/FS
STD_DEV = 0.001
Kl = 0.8
Kr = 0.4

class LocalisationNode():
    def __init__(self):

        rospy.init_node("localisation_deadreckoning")

        # Subscribe to the wheel velocity topics
        rospy.Subscriber("/wl", Float32, self._wl_callback)
        rospy.Subscriber("/wr", Float32, self._wr_callback)
        self.wl = None
        self.wr = None

        # Subscribe to the twist topic
        rospy.Subscriber("/cmd_vel", Twist, self._vel_callback)
        # self.v = 0.0
        # self.w = 0.0
                
        # Publish to the odometry topic
        self.pOdom = rospy.Publisher("/odom", Odometry, queue_size=10)
        # self.odom = Odometry()

        # Publish to the joint states topic ()
        self.pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
        # self.js = JointState()

        # Service for reset state values
        ser = rospy.Service("/reset", Empty, self._ser_callback)

        # States of the mobile robot
        self.x = 0.0
        self.y = 0.0
        self.q = 0.0
        self.robot_pose = np.array([[0.0], [0.0], [0.0]]) # Robot initial position
        self.model_pose = PoseWithCovariance()
        self.model_twist = TwistWithCovariance()

        self.sigma = np.zeros((3, 3)) # Initial covariance matrix
        self.pose_cov = np.zeros((6, 6)) 
        self.twist_cov = np.zeros((6, 6)) 
              
        # For broadcasting transform from base_link to odom 
        self.tb = tf.TransformBroadcaster()
        
        self.rate = rospy.Rate(FS)
        
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
                
    def main(self):

        t0 = rospy.Time.now()
        f1 = R/L #To save some calculations
        f2 = 0.5*R #To save some calculations
        v = 0
        w = 0

        while not rospy.is_shutdown():

            # Pose parameters
            self.model_pose.pose.position.x = self.robot_pose[0][0]
            self.model_pose.pose.position.y = self.robot_pose[1][0]
            # Quaternion
            qRota = tf.transformations.quaternion_from_euler(0, 0, self.robot_pose[2][0])
            self.model_pose.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
            # Pose covariance
            self.model_pose.covariance[:] = np.ravel(self.pose_cov)

            # Twist parameters
            self.model_twist.twist.linear.x = v
            self.model_twist.twist.angular.z = w
            self.model_twist.covariance[:] = np.ravel(self.twist_cov)

            # Publish the transform
            t = TransformStamped()
            self.tb.sendTransform([self.robot_pose[0][0], self.robot_pose[1][0], 0], qRota, rospy.Time.now(), "base_link", "map")

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
            # Publish the message
            self.pOdom.publish(odom)

            if (self.wl is not None) and (self.wr is not None):
                
                w = (self.wr-self.wl) * f1
                v = (self.wr+self.wl) * f2
                dd = T*v
                dth = T*w
                s_th_k_1 = self.robot_pose[2][0]

                # Robot state update
                displacement_vector = np.array([[dd * np.cos(s_th_k_1)],
                                                [dd * np.sin(s_th_k_1)],
                                                [dth]])
                self.robot_pose = self.robot_pose + displacement_vector
                
                # 3x3 Linear model Jacobian of the robot
                H = np.array([[1.0, 0.0, -dd * np.sin(s_th_k_1)],
                                [0.0, 1.0, dd * np.cos(s_th_k_1)],
                                [0.0, 0.0, 1.0]])
                
                # Matrix 
                delta_w = 0.5*R*T * np.array([[np.cos(s_th_k_1), np.cos(s_th_k_1)],
                                                [np.sin(s_th_k_1), np.sin(s_th_k_1)],
                                                [2/L, -2/L]])
                
                # Noise (tunned according to the test of mini challenge 1)
                sigma_delta = np.array([[Kr*np.abs(self.wr), 0],
                                        [0, Kl*np.abs(self.wl)]]) 
                
                # Pose covariance matrix
                Q = np.dot(np.dot(delta_w, sigma_delta), delta_w.T)

                # Update covariance matrix
                self.sigma = np.dot(np.dot(H, self.sigma), H.T) + Q

                self.pose_cov[0][0] = self.sigma[0][0]
                self.pose_cov[0][1] = self.sigma[0][1]
                self.pose_cov[1][0] = self.sigma[1][0]
                self.pose_cov[1][1] = self.sigma[1][1]
                self.pose_cov[0][5] = self.sigma[0][2]
                self.pose_cov[1][5] = self.sigma[1][2]
                self.pose_cov[5][0] = self.sigma[2][0]
                self.pose_cov[5][1] = self.sigma[2][1]
                self.pose_cov[5][5] = self.sigma[2][2]          

            self.rate.sleep()
                    
if __name__ == '__main__':

    try:
        aux = LocalisationNode()
        aux.main()

    except rospy.ROSInterruptException:
        pass