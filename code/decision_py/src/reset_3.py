#!/usr/bin/env python
#keep thing above

#import relevant libraries
import rospy
import numpy as np
from geometry_msgs.msg import Twist
import time
import sys
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState

#default when script is launched is /first/rosbot unless otherwise mentioned with the argument
if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]

class ResetNode:
    def __init__(self, namespace):
        self.namespace = namespace
        self.twist = Twist()
        self.pub_vel = rospy.Publisher(self.namespace+'/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.status = ModelState()

    def name(self):
        """
        defines to what model_name it publishes, depended on namespace
        """
        self.status.model_name = "rosbot_"+self.namespace[1] #will not work for 4 and 5
        self.pub.publish(self.status)

    def stop(self):
        """
        makes all ROSbots stop
        """
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.y = 0
        self.twist.angular.x = 0
        self.twist.angular.y = 0
        self.twist.angular.z = 0
        self.pub_vel.publish(self.twist)

    def set_rosbot(self, x, y):
        """
        sets orientation to forwards and position (x,y) dependend on input of function + publishes
        """
        self.status.pose.orientation.x = 0
        self.status.pose.orientation.y = 0
        self.status.pose.orientation.z = 0
        self.status.pose.orientation.w = 0
        self.status.pose.position.x = x
        self.status.pose.position.y = y
        self.status.pose.position.z = 0.044
        self.pub.publish(self.status)

#one node to reset all of them
rospy.init_node('reset_node', anonymous=True)
rosbot_f = ResetNode("/first")
rosbot_s = ResetNode("/second")
rosbot_t = ResetNode("/third")

rate = rospy.Rate(10) #runs at 10hz
while not rospy.is_shutdown(): #loop necessary for it to work, press ctrl+c when the ROSbots are in correct place
    rosbot_f.name()
    rosbot_f.stop()
    rosbot_f.set_rosbot(-9.7,0)
    rosbot_s.name()
    rosbot_s.stop()
    rosbot_s.set_rosbot(-8.6,0)
    rosbot_t.name()
    rosbot_t.stop()
    rosbot_t.set_rosbot(-9.7,0.28)
    rate.sleep()
