#!/usr/bin/env python

import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

video_capture = cv2.VideoCapture(0)
bridge = CvBridge()

def end_callback():
	cv2.destroyAllWindows()

rospy.init_node("webimage_node_publisher")
image_pub = rospy.Publisher("/video_source/raw", Image, queue_size=10)
rospy.on_shutdown(end_callback)
rate = rospy.Rate(10)

def main(args):
	try:
		if not video_capture.isOpened():
			return
		while not rospy.is_shutdown():
			ret, frame = video_capture.read()
			image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
			rate.sleep()
	except rospy.ROSInterruptException:
			rospy.logerr("Ros Interrupt Exception")
if __name__=="__main__":
	if len(sys.argv) == 1:
		main(sys.argv)
	else:

		print("Incorrect")
