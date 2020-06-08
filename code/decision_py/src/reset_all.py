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
    rosbot_x = sys.argv[2]
    rosbot_y = sys.argv[3]

class ResetNode:
    def __init__(self, namespace):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_"+'reset_node', anonymous=True)
        self.twist = Twist()
        self.pub_vel = rospy.Publisher(self.namespace+'/cmd_vel', Twist, queue_size=10)
        self.pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
        self.status = ModelState()

    def name(self):
        """
        defines to what model_name it publishes, depended on namespace
        """
        self.status.model_name = "rosbot_"+ self.getmodelname()
        self.pub.publish(self.status)

    def stop(self):
        """
        makes all ROSbots stop
        """
        self.twist.linear.x = 0
        self.twist.linear.y = 0
        self.twist.linear.z = 0
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

    def getmodelname(self):
        '''
        Very ugly solution, but it works...
        '''
        if self.namespace== "/first":
            mn = "f"
        elif self.namespace== "/second":
            mn = "s"
        elif self.namespace== "/third":
            mn = "t"
        elif self.namespace== "/fourth":
            mn = "fo"
        elif self.namespace== "/fifth":
            mn = "fi"

        return mn


rosbot = ResetNode(namespace)
rate = rospy.Rate(10) #runs at 10hz
i = 0
while not rospy.is_shutdown(): #loop necessary for it to work, press ctrl+c when the ROSbots are in correct place
    rosbot.name()
    rosbot.stop()
    rosbot.set_rosbot(float(rosbot_x), float(rosbot_y))
    rate.sleep()
    print(i)
    i +=1
    if i >4:
        break
