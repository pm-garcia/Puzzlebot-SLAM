#!/usr/bin/env python
""" Implements a "ground truth" odometry using the model state information
from Gazebo.
"""
import sys
import numpy as np
import rospy
from geometry_msgs.msg import Twist

class MoveInCircleCommander():

    def __init__(self):
        rospy.init_node('MoveInCircle')
        # Publish to the /cmd_vel topic
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(90)

    def main(self, radie=5, linvel=0.1):
        # If there's an object attached to the ee we want it to follow its trajectory
        while not rospy.is_shutdown():
            self.rate.sleep()
            self._move_in_circle(radie, linvel)

    def _move_in_circle(self, radie, linvel):
        tw = Twist()
        #----------------------------------------------------------------------------
        angvel = linvel/radie
        tw.linear.x = linvel
        tw.angular.z = -angvel
        #----------------------------------------------------------------------------
        self.pub.publish(tw)

if __name__ == '__main__':
    try:
        node = MoveInCircleCommander()
        node.main()
    except rospy.ROSInterruptException:
        pass