#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class Odometry():
	
	def __init__(self):
		# Inicializar nodo
		rospy.init_node("localisation")
		self.rate = rospy.Rate(90)

		#self.r = 0.055
		#self.l = 0.185
		self.r = 0.0505
		self.l = 0.183
		
		# Subscriptores a wl y wr
		rospy.Subscriber("/wl", Float32, self.wlCallback)
		rospy.Subscriber("/wr", Float32, self.wrCallback)
		self.wl = None
		self.wr = None

		# Publicador de la posicion
		self.odom_pub = rospy.Publisher("/odometry", Pose2D, queue_size = 1)
		self.odom = Pose2D()
		
		self.model_state = np.array([0.0, 0.0, 0.0])

		# Calculo del tiempo pasado
		self.current_time = rospy.get_time()
		self.last_time = self.current_time

	# Callbacks para wl y wr
	def wlCallback(self, msg):
		self.wl = msg.data

	def wrCallback(self, msg):
		self.wr = msg.data

	def main(self):
		while not rospy.is_shutdown():  
			self.rate.sleep()

			self.current_time = rospy.get_time()
			dt  = self.current_time - self.last_time
			self.last_time = self.current_time
			
			th = self.model_state[0]
			x = self.model_state[1]
			y  = self.model_state[2]

			if (self.wl is not None) and (self.wr is not None):
				# Ecuaciones para la obtencion de theta, x, y
				th += self.r * ((self.wr - self.wl)/self.l) * dt
				x += self.r * ((self.wr + self.wl)/2) * dt * np.cos(th)
				y += self.r * ((self.wr + self.wl)/2) * dt * np.sin(th)
				# Ajuste del rango de theta
				#if (th >= 2*np.pi or th <= -2*np.pi):
				#	th = 0.0
				if (th > np.pi):
					th = -np.pi
				elif (th <= -np.pi):
					th = np.pi
				# Asignacion de valores y pub al topico
				self.model_state = np.array([th, x, y])

			self.odom.theta = th
			self.odom.x = x
			self.odom.y = y
			self.odom_pub.publish(self.odom)

if __name__ == '__main__':

	try:
		odometry = Odometry()
		odometry.main()
	except rospy.ROSInterruptException:
		pass
