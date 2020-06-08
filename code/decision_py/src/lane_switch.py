#!/usr/bin/env python
#keep line above

#import relevant libraries
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray
import sys

#default when script is launched is /first/rosbot unless otherwise mentioned with the argument
if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]


class LaneswitchNode:
    def __init__(self, namespace):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'lane_switch_node', anonymous=True) #initialise node
        self.pub = rospy.Publisher(self.namespace+'/lane_switch_bool', UInt8MultiArray, queue_size=10)# publish to topic that decision.py is subscribed to
        self.bool = UInt8MultiArray()
    def check_lane_switch(self):
        pass

    def publisher(self):

        self.pub.publish(self.bool)


rosbot = LaneswitchNode(namespace)
while not rospy.is_shutdown():
    """
    When given true it will tell decision.py to move a lane to the left
    """
    try:
        inp = [0,0]
        print("give True/False for doing it")
        inp[0] = bool(input())

        print("give True/False for direction (Left = True)")
        inp[1] = bool(input())
        print(inp)

        rosbot.bool.data = inp

        rosbot.publisher()
        print("input succesfull")
    except Exception as e:
        print("invalid input")
        print(e)
