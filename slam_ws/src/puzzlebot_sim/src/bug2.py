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
        self.goal = (self.tx, self.ty)
        print("Target: ", tx, ty)

        # Odometry subscriber
        rospy.Subscriber("/odometry", Odometry, self._odom_callback)
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
        self.MAX_VELOCITY = 0.2
        self.MIN_VELOCITY = 0.1
        self.MAX_ANGULAR_VELOCITY = 0.3
        self.d_wall = 0.38
        self.follow_distance = 0.5
        self.state2 = ''
        self.angle_to_goal = 0
        self.hit_point = 0
        self.leave_point = 0
        self.distance_to_goal = 1000
        self.tiempo_transcurrido = 0
        # TODO: Agregar ganancias
        self.K0 = 0.5 # Ganancia velocidad angular
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
        
        # Min or mean?
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
        self.state2 = "turning"

        self.angle_to_goal = np.arctan2(self.ty-self.cy, self.tx-self.cx) - self.yaw
        angle_limit = 0.15
        
        self.distance_to_goal = np.sqrt((self.ty - self.cy)**2+(self.tx - self.cx)**2)
        if abs(self.distance_to_goal*self.K1) < self.MAX_VELOCITY:
            linear_velocity = self.distance_to_goal * self.K1
        elif abs(self.distance_to_goal*self.K1) >= self.MAX_VELOCITY:
            linear_velocity = self.MAX_VELOCITY

        if abs(self.angle_to_goal) < angle_limit:
            self.cmd.linear.x = linear_velocity
        elif abs(self.angle_to_goal) >= angle_limit:
            self.cmd.linear.x = 0.0

         #angular_velocity = self.angle_to_goal * self.K0

        # if self.angle_to_goal > np.pi:
        #     self.angle_to_goal -= 2 * np.pi
        # elif self.angle_to_goal < -np.pi:
        #     self.angle_to_goal += 2 * np.pi
        
        # print(self.angle_to_goal)

        # if abs(self.K0*self.angle_to_goal) >= self.MAX_ANGULAR_VELOCITY:
        #     angular_velocity = self.MAX_ANGULAR_VELOCITY
        # else:
        #     angular_velocity = self.K0 * self.angle_to_goal
                
        # if (abs(self.angle_to_goal) < 0.045):
        #     self.cmd.angular.z = 0.0
        # elif (abs(self.angle_to_goal) >= 0.045):
        #     self.cmd.angular.z = angular_velocity
            
        # self.cmd_pub.publish(self.cmd)
        # Ajustar el angulo para que este entre -pi y pi
        if self.angle_to_goal > np.pi:
            self.angle_to_goal -= 2 * np.pi
        elif self.angle_to_goal < -np.pi:
            self.angle_to_goal += 2 * np.pi

        # Calcular la velocidad angular del robot
        
        if abs(self.angle_to_goal) < np.pi/2:
            angular_velocity = self.K0 * self.angle_to_goal
        else:
            angular_velocity = self.K0 * np.sign(self.angle_to_goal) * np.pi/2

        # Limitar la velocidad angular maxima
        if abs(angular_velocity) > self.MAX_ANGULAR_VELOCITY:
            angular_velocity = np.sign(angular_velocity) * self.MAX_ANGULAR_VELOCITY

        # Publicar la velocidad angular
        self.cmd.angular.z = angular_velocity
        self.cmd_pub.publish(self.cmd)

        # Detener el giro si el angulo al objetivo es menor que el limite
        if abs(self.angle_to_goal) < angle_limit:
            self.state2 = 'no-turn'
            self.cmd.angular.z = 0.0
        
        self.cmd_pub.publish(self.cmd)

    def follow_wall(self):
        #0.25
        #0.6
        alpha = 0.22
        beta = 0.65
        
        p1 = np.array([[self.regions['left'] * np.cos(90)],
              [self.regions['left'] * np.sin(90)]])
        
        p2 = np.array([[self.regions['fleft'] * np.cos(45)],
              [self.regions['fleft'] * np.sin(45)]])
        
        u_tan = p2 - p1
        uu_tan = u_tan/np.linalg.norm(u_tan)
        
        tmp = np.dot(p1.T, uu_tan) * uu_tan
        u_per = p1 - tmp.reshape(-1, 1)
        
        uu_per = u_per - self.d_wall * (u_per/np.linalg.norm(u_per))

        angle_vel = beta*np.arctan2(uu_tan[1], uu_tan[0]) + alpha*np.arctan2(uu_per[1], uu_per[0])
        
        self.cmd.linear.x = 0.08
        self.cmd.angular.z = angle_vel
        self.cmd_pub.publish(self.cmd)

    def leave_obstacle(self):
        if self.regions['front'] > 5:
            return True
                
    def obstacle_close(self):
        # if self.regions["front"] < 0.8 or self.regions["fright"] < 0.8 or self.regions["fleft"] < 0.8:
        if self.regions["front"] < 1 or self.regions["front"] < 0.15:
            return True 
            
    def reached_goal(self):
        if self.distance_to_goal < .20:
            return True
        else:
            return False
    def distancia_entre_puntos(self,punto1, punto2):
        x1, y1 = punto1
        x2, y2 = punto2
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        
    def distance_to_line(self):

        up_eq = np.fabs((self.ty - 0) * self.cx - (self.tx - 0) * self.cy + (self.tx * 0) - (self.ty * 0))
        lo_eq = np.sqrt((self.ty - 0)**2 + (self.tx - 0)**2)
        distance = up_eq / lo_eq
        return distance

    def main(self):
        while not rospy.is_shutdown():
            if self.regions is not None and self.cx is not None and self.cy is not None and self.yaw is not None:
                distance_position_to_line = self.distance_to_line()
                while self.reached_goal() == False:
                    self.go_to_goal()
                    if self.obstacle_close():
                        self.state2 = "hit"
                        self.hit_point = (self.cx, self.cy)
                        self.tiempo_inicial = rospy.get_time()
                        while self.distance_to_line() > .05:
                            while self.tiempo_transcurrido < 5:
                                self.tiempo_actual = rospy.get_time()
                                self.tiempo_transcurrido = self.tiempo_actual - self.tiempo_inicial
                                self.follow_wall()
                            self.tiempo_actual = rospy.get_time()
                            self.tiempo_transcurrido = self.tiempo_actual - self.tiempo_inicial
                            self.follow_wall()
                        self.leave_point = (self.cx, self.cy)

                        self.distancia_a_goal_hit = self.distancia_entre_puntos(self.goal, self.hit_point)
                        self.distancia_a_goal_leave = self.distancia_entre_puntos(self.goal, self.leave_point)

                        if self.distancia_a_goal_leave < self.distancia_a_goal_hit:
                            break
                        else:
                            continue
                # if self.reached_goal() == True:
                #     print("Goal reached!")
                #     break
                
if __name__ == "__main__":
    try: 
        bug2 = Bug2(float(sys.argv[1]), float(sys.argv[2]))
        bug2.main()
    
    except rospy.ROSInterruptException:
        pass
