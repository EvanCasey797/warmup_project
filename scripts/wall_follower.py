#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from numpy import inf
import math

class SendTwistData(object):
	""" This node sends ROS messages containing Twist data. The angle is determined by which angle the closesst wall is """
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
		
		#initialize boolean states
		self.perp_wall = False
		self.in_corner = False
		self.wall_found = False
		self.near_wall = False
		
		#initialize local copy of lidar data
		self.lidar = []
		
		#initialize tolerance for angles
		self.deg_tol = 1
		
		#initialize count for number of turn cycles
		self.count = 0
	
	#function to find the closest wall to the robot
	#uses the local copy of lidar data
	def getClosestWall(self):
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
	
	
	#turns the robot until the current value (usually given as the closest wall) is equal to the goal
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
			self.my_turn_vel.angular.z = -0.2 * (curr/60)
		elif curr < 0:
			self.my_turn_vel.angular.z = 0.2 * -1 * (curr/60)
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
			#find the closest wall
			min_angle = self.getClosestWall()
			#see how far away the wall directly in front of the robot is
			forward_wall = self.lidar[0]
			#checks to see if robot is facing the closest wall
			#if not, it rotates towards the closest wall
			if not self.wall_found:
				if min_angle < self.deg_tol or min_angle > 360 - self.deg_tol:
						self.wall_found = True
						self.twist_pub.publish(self.my_stop_vel)
				else:
					self.turnToAngle(min_angle, 0)
			#checks to see if the wall the robot is facing is too close
			#if not, it moves towards it to the correct distance
			elif not self.near_wall:
				if forward_wall > 0.5:
					self.twist_pub.publish(self.my_straight_vel)
				elif forward_wall < 0.3:
					self.twist_pub.publish(self.my_backwards_vel)
				else:
					self.twist_pub.publish(self.my_stop_vel)
					self.near_wall = True
			#checks to see if the robot is facing the right way to follow the wall
			#if not, it tries to rotate so that 270 degrees is the closest angle to the wall
			elif not self.perp_wall:
				if abs(min_angle - 270) < self.deg_tol:
						self.perp_wall = True
						self.twist_pub.publish(self.my_stop_vel)
				else:
					self.turnToAngle(min_angle, 270)
			#checks to see if the robot is in a corner
			#if not, it moves forward along the wall, if it gets too far away it tries to correct
			elif not self.in_corner:
				if self.lidar[min_angle] < 0.49:
					self.my_straight_vel.angular.z = 0.02
				elif self.lidar[min_angle] > 0.51:
					self.my_straight_vel.angular.z = -0.02
				else:
					self.my_straight_vel.angular.z = 0
				if forward_wall > 0.5:
					self.twist_pub.publish(self.my_straight_vel)
				elif forward_wall < 0.3:
					self.twist_pub.publish(self.my_backwards_vel)
				else:
					self.twist_pub.publish(self.my_stop_vel)
					
					self.in_corner = True
			#the robot must now be in the corner facing the wall it should be following
			#thus we rotate 90 degrees counterclockwise
			else:
				if self.count <= 10:
					self.twist_pub.publish(self.my_corner_vel)
					r.sleep()
					self.count+=1
				else:
					self.count = 0
					self.in_corner = False
					


#main method to trigger the run function
if __name__ == '__main__':
	node = SendTwistData()
	node.run()
