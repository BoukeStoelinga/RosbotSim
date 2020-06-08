#!/usr/bin/env python
#keep this first line

#import relevant libraries
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import sys
import time

#default when script is launched is /first/rosbot unless otherwise mentioned with the argument
if len(sys.argv) == 1:
    namespace = "/first"
    namespace = 0.2
else:
    namespace = sys.argv[1]
    max_speed = float(sys.argv[2])


class CruiseNode:
    def __init__(self, namespace, max_speed):
        self.max_speed = max_speed
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'cruise_node', anonymous=True) #initialise node
        self.twist = Twist() #Define what type of veriable self.twist is (Twist() is for velocities)
        self.pub = rospy.Publisher(self.namespace+'/cruising', Twist, queue_size=10) #Defines where to publish according to what rosbot the script is used for

    def speed(self):
        """
        Set all velocities to 0 except 0.35 m/s driving forwards
        Also publishes those velocities
        """
        self.twist.linear.x = max_speed
        self.twist.linear.y = 0
        self.twist.linear.z = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.pub.publish(self.twist)


rosbot = CruiseNode(namespace, max_speed) #initialise the class for that namespace (defined in lines 11-14)
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rosbot.speed() #do speed function for that namespace
    rate.sleep()
