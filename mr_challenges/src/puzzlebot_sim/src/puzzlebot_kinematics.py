#!/usr/bin/env python
import tf, rospy
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import Empty, EmptyResponse
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion

x, y, q = 0.0, 0.0, 0.0 # Initial Conditions
r, l = 0.05, 0.188
v, w = 0.0, 0.0
fs = 60
mu = 0
sigma = 0.5

def callback_top(msg):
    global v, w
    v = msg.linear.x
    w = msg.angular.z

def callback_ser(req):
    global x, y, q
    x, y, q = 0.0, 0.0, 0.0
    return EmptyResponse()

def node():
    global x, y, q
    T = 1.0/fs
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        a1 = np.random.normal(mu, sigma)       
        a2 = np.random.normal(mu, sigma)  
        x += T*(v+a1)*np.cos(q)
        y += T*(v+a1)*np.sin(q)
        q += T*(w+a2)
        robotLocation = PoseStamped()
        robotLocation.pose.position= Point(x, y, 0)
        qRota = tf.transformations.quaternion_from_euler(0, 0, q)
        robotLocation.pose.orientation = Quaternion(qRota[0], qRota[1], qRota[2], qRota[3])
        cTime = rospy.Time.now()
        robotLocation.header.stamp = cTime
        robotLocation.header.frame_id = "base_link"
        pPose.publish(robotLocation)
        wr, wl = (v + 0.5*l*w)/r, (v - 0.5*l*w)/r
        pWl.publish(wl)
        pWr.publish(wr)
        tb.sendTransform([x,y,0], qRota, cTime, "base_link", "map")
        js = JointState()
        js.name = ['right_wheel_joint', 'left_wheel_joint']
        t = cTime - t0
        js.position = [wr*t.to_sec(), wl*t.to_sec()]
        js.header.stamp = cTime
        pJS.publish(js)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('puzzlebot_kinematic_model')
        ser = rospy.Service("/reset", Empty, callback_ser)
        tb = tf.TransformBroadcaster()
        pPose = rospy.Publisher('/pose', PoseStamped, queue_size=10)
        pWl   = rospy.Publisher('/wl', Float32, queue_size=10)
        pWr   = rospy.Publisher('/wr', Float32, queue_size=10)
        pJS = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber("/cmd_vel", Twist, callback_top)
        rate = rospy.Rate(fs)
        node()
    except rospy.ROSInterruptException:
    	pass