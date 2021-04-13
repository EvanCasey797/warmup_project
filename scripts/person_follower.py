#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from numpy import inf
import math

class SendTwistData(object):
	""" This node sends ROS messages containing Twist data, either to drive in a straight line or to turn left 90 degrees """
	#initializes the node
	def __init__(self):
		rospy.init_node('send_velocity')
		
		#initialize twist publisher and lidar subscriber
		self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
		
		#initialize values for velocities to publish
		self.my_straight_vel = Twist(linear=Vector3(0.2,0,0), angular=Vector3(0,0,0))
		self.my_backwards_vel = Twist(linear=Vector3(-0.2,0,0), angular=Vector3(0,0,0))
		self.my_turn_vel = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
		self.my_corner_vel = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,math.pi/2))
		self.my_stop_vel = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,0))
		
		#initialize local copy of lidar data
		self.lidar = []
		
		#initialize tolerance for angles
		self.deg_tol = 1
		
		#initialize count for number of turn cycles
		self.count = 0
		
	#function to find the closest object to the robot
	#uses the local copy of lidar data
	def getClosestObject(self):
		min_distance = self.lidar[0]
		min_angle = 0
		for i in range(0,360):
			if self.lidar[i] < min_distance:
				min_distance = self.lidar[i]
				min_angle = i
		return min_angle
	
	#callback function to set local lidar to the new sensor values
	def callback(self, msg):
		self.lidar = msg.ranges
			
	#turns the robot until the current value (usually given as the closest object) is equal to the goal
	#attempts to always turn the fastest direction (ie. < 180 degrees)
	def turnToAngle(self, curr, goal):
		#determines the angle to turn between -180 and 180
		if goal - curr > 180:
			curr = goal - (curr + 360)
		else:
			curr = goal - curr
		if curr < -180:
			curr = curr + 360
		if curr > 180:
			curr = curr - 360
		#publishes the vel to turn the correct way
		if curr > 0:
			self.my_turn_vel.angular.z = -0.3 * (curr/60)
		elif curr < 0:
			self.my_turn_vel.angular.z = 0.3 * -1 * (curr/60)
		else:
			self.my_turn_vel.angular.z = 0
		self.twist_pub.publish(self.my_turn_vel)
		

	#run function that keeps track of what the robot state is to turn the correct directions
	def run(self):
		#Give the turtle time to initialize
		r = rospy.Rate(1)
		r.sleep()
		#Set the frequency of sends that we actually want to use
		r = rospy.Rate(10)
		while not rospy.is_shutdown():
			nearest_object = self.getClosestObject()
			if abs(nearest_object) > self.deg_tol:
				self.turnToAngle(nearest_object,0)
			elif self.lidar[0] > 2:
				self.twist_pub.publish(self.my_straight_vel)
			elif self.lidar[0] < 1.5:
				self.twist_pub.publish(self.my_stop_vel)				


#main method to trigger the run function
if __name__ == '__main__':
	node = SendTwistData()
	node.run()
