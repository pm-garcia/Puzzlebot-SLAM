#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import quaternion_from_euler

class LocalizationNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('localization_node', anonymous=True)
        
        # Initialize ROS subscribers
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/wheel_odom', Odometry, self.odometry_callback)
        
        # Initialize ROS publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        
        # Initialize robot parameters
        self.robot_width = 0.2 # meters
        self.wheel_radius = 0.05 # meters
        self.pose = Pose()
        self.velocity = Vector3()
        self.imu_data = Imu()
        self.prev_time = rospy.Time.now()
        self.prev_odom = Odometry()
        
        # Initialize ROS rate
        self.rate = rospy.Rate(10) # Hz
        
    def run(self):
        while not rospy.is_shutdown():
            # Compute time delta
            current_time = rospy.Time.now()
            delta_time = (current_time - self.prev_time).to_sec()
            
            # Compute linear and angular velocity from wheel odometry
            delta_left = self.odom.pose.pose.position.x - self.prev_odom.pose.pose.position.x
            delta_right = self.odom.pose.pose.position.y - self.prev_odom.pose.pose.position.y
            linear_velocity = (delta_left + delta_right) / (2 * delta_time)
            angular_velocity = (delta_right - delta_left) / (self.robot_width * delta_time)
            
            # Integrate velocity to update robot pose
            self.pose.position.x += linear_velocity * delta_time * cos(self.pose.orientation.z)
            self.pose.position.y += linear_velocity * delta_time * sin(self.pose.orientation.z)
            self.pose.orientation.z += angular_velocity * delta_time
            
            # Publish odometry message
            self.publish_odometry(linear_velocity, angular_velocity)
            
            # Update previous time and odometry
            self.prev_time = current_time
            self.prev_odom = self.odom
            
            # Sleep until next iteration
            self.rate.sleep()
            
    def velocity_callback(self, msg):
        self.velocity = msg.linear
        
    def imu_callback(self, msg):
        self.imu_data = msg
        
    def odometry_callback(self, msg):
        self.odom = msg
        
    def publish_odometry(self, linear_velocity, angular_velocity):
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose = self.pose
        odom_msg.twist.twist.linear = linear_velocity
        odom_msg.twist.twist.angular = angular_velocity
        
        # Publish odometry message
        self.odom_pub.publish(odom_msg)

if __name__ == '__main__':
    try:
        node = LocalizationNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
