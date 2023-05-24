#!/usr/bin/env python

import rospy, sys
import numpy as np
from tf import transformations
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Bug2():

    def __init__(self, tx, ty):

        # Initialize node
        rospy.init_node("Bug2")
        self.rate = rospy.Rate(10)
        
        # Target point
        self.tx = tx
        self.ty = ty
        print("Target: ", tx, ty)

        # Odometry subscriber
        rospy.Subscriber("/true_odometry", Odometry, self._odom_callback)
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
        self.point_lstate = "leave"
        self.point_cstate = "leave"
        self.MAX_VELOCITY = 0.3
        self.MIN_VELOCITY = 0.15
        self.MAX_ANGULAR_VELOCITY = 0.2
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

        # index = [i for i, x in enumerate(msg.ranges) if x != float('inf')]
        # print(index)

        # Front: 0:52 y 1097:1146 - 101
        # FRight: 955:1065 - 110
        # FLeft: 95:196 - 101
        # Right: 788:922 - 134
        # Left: 225:358 - 133
        # Back: 497:651 - 154
        # RBack: 645:768 - 123 
        # LBack: 382:502 - 120

        front_index = msg.ranges[0:60] + msg.ranges[1086:1146]
        
        # Min or mean?
        self.regions = {
            "front":    min(min(front_index), 10),
            "fleft":    min(min(msg.ranges[96:216]), 10),
            "left":     min(min(msg.ranges[226:346]), 10),
            "lback":    min(min(msg.ranges[386:466]), 10),
            "back":     min(min(msg.ranges[496:616]), 10),
            "rback":    min(min(msg.ranges[646:766]), 10),
            "right":    min(min(msg.ranges[786:906]), 10),
            "fright":   min(min(msg.ranges[956:1016]), 10),
        }

    def go_to_goal(self):
        
        self.angle_to_goal = np.arctan2(self.ty-self.cy, self.tx-self.cx) - self.yaw
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

        # print("Front: ", self.regions["front"])
        # print("Right Front: ", self.regions["fright"])
        # print("Right: ", self.regions["right"])
        # print("Left: ", self.regions["left"])
        # print("Left Front: ", self.regions["fleft"])
        # print("Back: ", self.regions["back"])
        # print("Right Back: ", self.regions["rback"])
        # print("Left Back: ", self.regions["lback"])

        self.cmd.angular.z = -self.MAX_ANGULAR_VELOCITY
        self.cmd.linear.x = 0.0

        # print("Turning")

        if self.regions['front'] >= 0.5:
            # print("Following wall")
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

        if self.regions["front"] > 0.5 and self.regions["lback"] < 0.6 and self.regions["left"] > 0.5:
            print("Turning left")
            self.point_cstate = "leave"
            print("estado actual: ", self.point_cstate)
            self.cmd.angular.z = self.MAX_ANGULAR_VELOCITY 
            self.cmd.linear.x = 0.0

        # print("front: ",self.regions["front"] > 0.5)
        # print("lback ",self.regions["lback"] < 0.6)
        # print("left",self.regions["left"] > 0.5)   
        self.cmd_pub.publish(self.cmd)

    def leave_obstacle(self):
        if self.regions['front'] > 5:
            return True
                
    def obstacle_close(self):
        # if self.regions["front"] < 0.5 or self.regions["fright"] < 0.5 or self.regions["fleft"] < 0.5:
        if self.regions["front"] < 0.5:
            return True
            
    def left_obstacle_close(self):
         if self.regions["left"] < 0.5 and self.regions["fleft"] < 0.5 and self.regions['lback'] < 0.5:
            return True

    def reached_goal(self):
        if self.distance_to_goal < .20:
            return True
        else:
            return False
        
    def distance_to_line(self):

        up_eq = np.fabs((self.ty - 0) * self.cx - (self.tx - 0) * self.cy + (self.tx * 0) - (self.ty * 0))
        lo_eq = np.sqrt((self.ty - 0)**2 + (self.tx - 0)**2)
        distance = up_eq / lo_eq
        return distance

    # def normalize_angle(angle):
    #     if(np.fabs(angle) > np.pi):
    #         angle = angle - (2 * np.pi * angle) / (np.fabs(angle))
    #     return angle

    def main(self):
        while not rospy.is_shutdown():
            if self.regions is not None and self.cx is not None and self.cy is not None and self.yaw is not None:
                distance_position_to_line = self.distance_to_line()
                #print("estado actual: ", self.point_cstate)
                if self.state == "go-to-goal":
                    # print("Going to goal")
                    print("Going to goal")
                    self.go_to_goal()
                    if self.reached_goal():
                         rospy.loginfo("Goal reached!")
                         break
                    elif self.obstacle_close():
                        # print("Obstacle detected!")
                        self.state = "follow-wall"
                        self.point_cstate = "hit"
                        #print("estado actual: ", self.point_cstate)
                elif self.state == "follow-wall":
                    #print("Following wall")
                    self.follow_wall()
                    # if self.leave_obstacle():
                    #      print("Going to goal")
                    print("Distancia de la linea: ", distance_position_to_line)
                    if  self.point_cstate == "leave" and distance_position_to_line < 0.1:
                         print("Entro al bucle")
                         self.state = "go-to-goal"
                    

                    

if __name__ == "__main__":
    try: 
        bug2 = Bug2(float(sys.argv[1]), float(sys.argv[2]))
        bug2.main()
    
    except rospy.ROSInterruptException:
        pass
