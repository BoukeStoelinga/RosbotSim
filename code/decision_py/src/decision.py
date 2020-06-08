#!/usr/bin/env python
#keep this first line

#import relevant libraries
import rospy
import numpy as np
from std_msgs.msg import Float64

from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray
import sys
from std_msgs.msg import Bool
import time

#default when script is launched is /first/rosbot unless otherwise mentioned with the argument
if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]


class DecisionNode:
    def __init__(self,namespace):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'decision_node', anonymous=True)
        self.lane_correction = Float64()
        self.twist = Twist()
        self.pub = rospy.Publisher(self.namespace+'/cmd_vel', Twist, queue_size=10)
        self.lane_keep = True
        self.pub_done_lane_switch=rospy.Publisher(self.namespace+'/switch_done', Bool, queue_size=10) #indicate if a ROSbot is switching lanes
        self.red_lane_vel_factor = 1

        done_with_laneswitch = Bool()
        done_with_laneswitch.data =True
        self.pub_done_lane_switch.publish(done_with_laneswitch)
        #self.pub_done_lane_switch_right=rospy.Publisher(self.namespace+'/right_switch_done', Bool, queue_size=10)

    def subscribers(self):
        """
        subscriber to topics
        publishes twist
        """
        rospy.Subscriber(self.namespace+'/lane_keep_ang_vel', Float64, self.callback_ang_vel)
        rospy.Subscriber(self.namespace+'/platoon_linear_vel', Float64, self.callback_lin_vel)
        rospy.Subscriber(self.namespace+"/lane_switch_bool", UInt8MultiArray, self.callback_switch_lanes)
        rospy.Subscriber(self.namespace+"/cruising", Twist, self.callback_cruise)
        rospy.Subscriber(self.namespace+"/red_lane_vel", Float64, self.callback_red_lane_vel)
        rospy.Subscriber("/killswitch", Bool, self.callback_killswitch)
        #rospy.Subscriber(self.namespace+"/lane_merging_decrease_vel", Float64, self.callback_lane_merging_decrease)
        # rospy.Subscriber(self.namespace+"/cmd_vel", Twist, self.callback_speed)


        rospy.spin()
    def callback_killswitch(self,data):
        if data.data:
            sys.exit()

    # def callback_lane_merging_decrease(self, lin_vel):
    #     if lane_merging_decrease_vel>0
    #         self.twist.linear.x=lane_merging_decrease_vel
    #
    # def callback_lin_vel(self,lin_vel):
    #     self.twist.linear.x = lin_vel.data
    #
    # def choose_lin_vel(self, lin_vel):
    #     self.twist.linear.x = min(callback_lin_vel(), callback_lane_merging_decrease())
    #     self.pub.publish(self.twist)

    def callback_lin_vel(self,lin_vel):
        self.twist.linear.x = lin_vel.data
        self.pub.publish(self.twist)

    def callback_ang_vel(self,ang_vel):
        if self.lane_keep:
            self.twist.angular.z = ang_vel.data
            self.pub.publish(self.twist)

    def callback_switch_lanes(self,lane_switch_bool):
        lane_switch_bool.data = list(map(ord, lane_switch_bool.data))
        self.lane_keep = not (lane_switch_bool.data[0]==1) #do it or not and set lane keeping on and off at the same time
        self.direction = (lane_switch_bool.data[1]==1) #define what direction true = left
        if not self.lane_keep:
            done_with_laneswitch = Bool()
            done_with_laneswitch.data =False
            self.pub_done_lane_switch.publish(done_with_laneswitch)

            self.switch_lanes(self.direction)

    def callback_cruise(self,cruise):
        self.twist.linear.x = cruise.linear.x
        self.pub.publish(self.twist)

    def callback_red_lane_vel(self, data):
        self.twist.linear.x = data.data
        self.pub.publish(self.twist)

    def _publisher(self):
        publ_lane_keep_ang_vel= Float64()
        publ_lane_keep_ang_vel.data = self._last_output
        self.pub.publish(publ_lane_keep_ang_vel)

    def switch_lanes(self,direction=True):
        # self.pub_done_lane_switch.publish(False)
        #self.pub_done_lane_switch_right.publish(False)
        current_speed = min(self.twist.linear.x,0.1)
        self.twist.angular.z = 0

        start = rospy.get_time()
        now = start
        while now-start < 0.1: #loop without a purpose to make the other loops work
            now = rospy.get_time()

        print("start turn")
        straight_time = 0.28/(current_speed*4) #current speed with a gain #0.23/(current_speed*4
        angle_time = 1

        self._switch(angle_time, direction)

        print("going straight")
        self._switch(straight_time,ang_vel=0)

        print("leveling out")
        self._switch(angle_time-0.15,not direction,ang_vel=2) #different angle_time because glitchy gazebo effect
        print("turn complete")


        self.twist.angular.z = 0
        self.debug_publisher()

        self.lane_keep = True
        done_with_laneswitch = Bool()
        done_with_laneswitch.data =True
        self.pub_done_lane_switch.publish(done_with_laneswitch)
        ##add also a right is finished publisher so overtaking script knows when overtaking is completed
        #self.pub_done_lane_switch_right.publish(True)



    def _switch(self,time,left=True,ang_vel=2):
        start = rospy.get_time()
        now = start
        if not left:
            ang_vel = ang_vel *(-1)
        while now-start < time: #in seconds
            self.twist.angular.z = ang_vel
            self.debug_publisher()
            now = rospy.get_time()

    def debug_publisher(self):
        self.pub.publish(self.twist)


rosbot = DecisionNode(namespace)
rosbot.subscribers()
