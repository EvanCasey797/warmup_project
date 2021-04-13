#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
import math

class SendTwistData(object):
    """ This node sends ROS messages containing Twist data, either to drive in a straight line or to turn left 90 degrees """
    
    #initializes the node and publisher
    def __init__(self):
        rospy.init_node('send_velocity')
        self.twist_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    #sends the 
    def run(self):
        #default velocity values
        #go straight at 0.3 speed
        #turn at pi/2 radians/s
        my_straight_vel = Twist(linear=Vector3(0.3,0,0), angular=Vector3(0,0,0))
        my_turn_vel = Twist(linear=Vector3(0,0,0), angular=Vector3(0,0,math.pi/2))
        
        #Defines starting values. Start moving straight with count of 0
        straight = True
        count = 0

        #Give the turtle time to initialize
        r = rospy.Rate(1)
        r.sleep()
        #Set the frequency of sends that we actually want to use
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            #if we are supposed to go straight
            if straight:
                self.twist_pub.publish(my_straight_vel)
            #if we are supposed to turn
            if not straight:
                self.twist_pub.publish(my_turn_vel)
            #turn for 1 second or 10 times through the loop
            while not straight and count < 10:
                #keep the same velocity but increment count
                r.sleep()
                count += 1
            #go straight for 5 seconds or 50 times through the loop
            while straight and count < 50:
                #keep the same velocity but increment count
                r.sleep()
                count += 1
            #switch from straight to turn or vice versa and reset the count
            straight = not straight
            count = 0

#main method to trigger the run function
if __name__ == '__main__':
    node = SendTwistData()
    node.run()
