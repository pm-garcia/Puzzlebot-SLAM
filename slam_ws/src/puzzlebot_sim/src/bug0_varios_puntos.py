#!/usr/bin/env python

import rospy, sys
import numpy as np
from tf import transformations
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

class Bug0_varios_puntos():

    def __init__(self):

        # Initialize node
        rospy.init_node("Bug0")
        self.rate = rospy.Rate(10)
        # Target point
        self.targets_x = [ 0,  -6, -4, 0, 0]
        self.targets_y = [ 9, 6, -1, -3, 0]
        self.contador = 0
        self.tx = self.targets_x[self.contador]
        self.ty = self.targets_y[self.contador]
        
        print("Target: ", self.tx, self.ty)

        # Odometry subscriber
        rospy.Subscriber("/odom", Odometry, self._odom_callback)
        self.yaw = None
        self.cx = None
        self.cy = None

        # Sensor (LIDAR) subscriber
        rospy.Subscriber("/scan", LaserScan, self._scan_callback)
        self.regions = None

        # Velocity publisher
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0

        # TODO: Agregar variables, tolerancias, etc. 
        self.state = "go-to-goal"
        self.MAX_VELOCITY = 0.5
        self.MIN_VELOCITY = 0.3
        self.MAX_ANGULAR_VELOCITY = 0.5
        self.follow_distance = 0.5
        # TODO: Agregar ganancias
        self.K0 = 0.3 # Ganancia velocidad angular
        self.K1 = 0.5 # Ganancia velocidad lineal
        self.K2 = 0.2 # Ganancia velocidad angular (Follow wall)
        self.K3 = 0.2 # Distance wall stabalizer 
        self.K4 = 0.2
        
        # Stop robot at finish
        rospy.on_shutdown(self._end_callback)

    # End callback (Stop robot)
    def _end_callback(self):
        self.cmd = Twist()
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)

    # Odometry callback
    def _odom_callback(self, msg):
        position = msg.pose.pose.position
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w, 
        )
        t = transformations.euler_from_quaternion(quaternion)
        self.yaw = t[2]
        self.cx = position.x
        self.cy = position.y
    
    # Sensor (LIDAR) callback
    def _scan_callback(self, msg):
        front_index = msg.ranges[0:71] + msg.ranges[1066:1146]
        
        self.regions = {
            "front":    min(min(front_index), 10),
            "fleft":    min(min(msg.ranges[72:213]), 10),
            "left":     min(min(msg.ranges[214:355]), 10),
            "lback":    min(min(msg.ranges[356:497]), 10),
            "back":     min(min(msg.ranges[498:639]), 10),
            "rback":    min(min(msg.ranges[640:781]), 10),
            "right":    min(min(msg.ranges[782:923]), 10),
            "fright":   min(min(msg.ranges[924:1065]), 10),
        }
    
    def go_to_goal(self):
        
        self.angle_to_goal = np.arctan2(self.ty-self.cy, self.tx-self.cx) - self.yaw
        if self.angle_to_goal > np.pi:
            self.angle_to_goal -= 2 * np.pi
        elif self.angle_to_goal < -np.pi:
            self.angle_to_goal += 2 * np.pi
        angle_limit = 0.1
        
        self.distance_to_goal = np.sqrt((self.ty - self.cy)**2+(self.tx - self.cx)**2)
        if abs(self.distance_to_goal*self.K1) < self.MAX_VELOCITY:
            linear_velocity = self.distance_to_goal * self.K1
        elif abs(self.distance_to_goal*self.K1) >= self.MAX_VELOCITY:
            linear_velocity = self.MAX_VELOCITY

        if abs(self.angle_to_goal) < angle_limit:
            self.cmd.linear.x = linear_velocity
        elif abs(self.angle_to_goal) >= angle_limit:
            self.cmd.linear.x = 0.0

        angular_velocity = self.angle_to_goal * self.K0
        self.cmd.angular.z = angular_velocity
        self.cmd_pub.publish(self.cmd)

    def follow_wall(self):

        self.cmd.angular.z = -self.MAX_ANGULAR_VELOCITY
        self.cmd.linear.x = 0.0

        print("Turning")

        if self.regions['front'] >= 0.5:
            print("Following wall")
            angular_velocity = (self.regions['fleft'] - self.regions['lback']) * self.K2
            distance_stabilizer = (self.follow_distance - self.regions['fleft']) * self.K3
            angular_velocity = (angular_velocity + distance_stabilizer)

            if abs(angular_velocity) < self.MAX_ANGULAR_VELOCITY:
                self.cmd.angular.z = angular_velocity
            elif abs(angular_velocity) >= self.MAX_ANGULAR_VELOCITY:
                self.cmd.angular.z = self.MAX_ANGULAR_VELOCITY

            angle_error = abs(self.regions['fleft'] - self.regions['lback']) * self.K4
            if angle_error < self.MIN_VELOCITY:
                self.cmd.linear.x = self.MAX_VELOCITY - angle_error
            if angle_error >= self.MIN_VELOCITY:
                self.cmd.linear.x = self.MIN_VELOCITY

        self.cmd_pub.publish(self.cmd)
                
    def obstacle_close(self):
        if self.regions["front"] < 0.5 or self.regions["fright"] < 0.5 or self.regions["fleft"] < 0.5:
            return True
        
    # def obstacle_close_turn(self):
    #     if self.regions["front"] < 0.25 or self.regions["fleft"] < 0.25 or self.regions["front"] < 0.25 or self.regions["left"] < 0.25:
    #         return True
    #     elif self.regions["lback"] > 0.25 and self.regions["front"] > 0.25 or self.regions["fleft"] > 0.25 or self.regions["front"] > 0.25:
    #         return False
    def reached_goal(self):
        if self.distance_to_goal < .10:
            return True
        else:
            return False

    def main(self):
        while not rospy.is_shutdown():
            if self.regions is not None and self.cx is not None and self.cy is not None and self.yaw is not None:
                if self.state == "go-to-goal":
                    #print("Going to goal")
                    self.go_to_goal()
                    if self.reached_goal():
                         print("Goal reached!")
                         self.contador += 1
                         if self.contador < len(self.targets_x):
                            self.tx = self.targets_x[self.contador]
                            self.ty = self.targets_y[self.contador]
                            self.state = "go-to-goal"
                         else:
                             self.state = "goal-reached"

                    elif self.obstacle_close():
                        print("Obstacle detected!")
                        self.state = "follow-wall"
                elif self.state == "follow-wall":
                     #print("Following wall")
                     self.follow_wall()
                    #  self.tiempo_inicial = rospy.get_time()
                     if not self.obstacle_close():
                        #  t0 = rospy.get_rostime().to_sec()
                        #  while (rospy.get_rostime().to_sec() - t0 <= 5):
                        #     #print(rospy.get_rostime().to_sec() - t0)
                        #     self.cmd.linear.x = 0.03
                        #     self.cmd_pub.publish(self.cmd)
                        #     self.rate.sleep()
                        #  self.cmd.linear.x = 0.0
                        #  self.cmd_pub.publish(self.cmd)
                         self.state = "go-to-goal"
                elif self.state == "goal-reached":
                    self._end_callback()

            self.rate.sleep()

                    
if __name__ == "__main__":
    try: 
        bug = Bug0_varios_puntos()
        bug.main()
    
    except rospy.ROSInterruptException:
        pass
