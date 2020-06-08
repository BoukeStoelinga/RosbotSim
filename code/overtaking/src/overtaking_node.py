#!/usr/bin/env python

"""
What still can be added:
- Implement right publisher that publishes to the topic that turns on lane switching
- If there is a car infront when 'you' are already driving in the most left lane then don't start to overtake
- Check if you passed the car, now the assumption is made that it is not safe to switch to the right when the car is not passed yet
- Overtake multiple cars
"""

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import sys
import numpy as np

if len(sys.argv) == 1:
    namespace = "/first"
    max_speed = 0.3 #works at 0.3 and the slower car 0.2, others are also possible
else:
    try:
        namespace = sys.argv[1]
        max_speed = float(sys.argv[2])
    except:
        namespace = "/first"
        max_speed = 0.3

class SensorValues():
    def __init__(self, namespace,max_speed, kp,offset):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'overtaking_node', anonymous=True)
        # platooning part:
        self.kp = kp
        self.offset = offset
        self.max_speed = max_speed
        self.own_vel = Float64()
        self.lin_vel_pub =rospy.Publisher(self.namespace+'/platoon_linear_vel', Float64, queue_size=10)

        self.current_time = rospy.get_time()
        self.bool=Bool()
        self.overtaking = False
        # self.previous_time =rospy.get_time()
        self.fl=0
        self.fr=0
        self.ddistance=0
        self.rel_vel_fl=0
        self.rel_vel_fr=0
        self.distance = []
        self.array_fl = []
        self.array_fr = []
        self.d_min = 0.15
        self.d_max = 0.375
        self.r_max = 0.9
        self.d_overtake=0.25            #distance from car in front at which the car starts the overtaking manoeuvre
        self.d_overtake_time=3          #time inbetween car and car in front
        # self.own_speed = Float64()      #define speed that the car works towards
        self.acceptable_range = 0.5     #when the car in front drives slower than 90% of own speed, start overtaking
             #initialize node
        self.pub = rospy.Publisher(self.namespace+'/lane_switch_bool', UInt8MultiArray, queue_size=10)
        self.switch_done = False
        #don't forget to publish this to your own made topic instead of cmd_vel:
        self.twist = Twist()
        # self.twist.linear.x=self.own_speed
        # self.pub = rospy.Publisher(self.namespace+'/cmd_vel', Twist, queue_size=10)
        # self.pub.publish(self.twist)

    # def overtaking_node():
            #overtaking_publisher = rospy.Publisher('/overtaking', Bool, queue_size=10)
    def distance_to_front(self):
        return min(self.fl,self.fr)
    def callback_killswitch(self,data):
        if data.data:
            sys.exit()

    def callback_fl(self,data):     #callback to read fl and set variable
        """
        Callback to read fl and set variable
        """
        self.calculate_own_vel()
        self.fl = data.range
    # def callback_fr_2(self,data):     #callback to read fl and set variable
    #     """
    #     Callback to read fr and set variable
    #     """
    #     print("fr2")
    #     self.fr = data.range

    def callback_fr(self,data):     #callback to read fr and set variable (value in meter)
        """
        Callback to read fr and set variable
        """
        self.fr = data.range
    def callback_rl(self,data):
        # print("executed")

        if self.overtaking:
            self.overtaking_manoeuvre()

    def callback_safety(self,data):
        """
        Callback to read the array with [left safe, right safe]
        """

        self.safety = list(map(ord, data.data))
        self.overtaking_check()


    def callback_switch_done(self,data):
        """
        Callback to read if the lane switching manoeuvre to the left lane is finished
        """
        self.switch_done = data.data

    def d_time(self):
        """
        Update time difference
        """
        self.previous_time = self.current_time
        self.current_time = rospy.get_time()
        return self.current_time - self.previous_time

    def calculate_rel_vel(self):
        """
        the relative velocity of the car in front is calculated.
        """
        #fills an array with the last 2 sensor values
        self.array_fl.append(self.fl)
        self.array_fr.append(self.fr)
        if len(self.array_fl) == 3:
            del self.array_fl[0]
            del self.array_fr[0]

        #determine relative speed using the change in distance and time difference between the measurements
        d_time = self.d_time()
        if len(self.array_fl) == 2:
            self.ddistance_fl = self.array_fl[1]-self.array_fl[0]
            self.ddistance_fr = self.array_fr[1]-self.array_fr[0]
            self.rel_vel_fl=self.ddistance_fl/d_time
            self.rel_vel_fr=self.ddistance_fr/d_time
            #self.rel_vel_fl=self.ddistance_fl/(self.d_time)
            #self.rel_vel_fr=self.ddistance_fr/(self.d_time)

    def calculate_own_vel(self):
        """
        Define speed based on distance to car in front
        """

        #Check if object too close, set vel to 0
        if self.fl < self.d_min or self.fr < self.d_min:
            self.own_vel.data= 0

        #check if the car is before the maximal allowed distance in platoon
        elif self.fl < self.d_max or self.fr < self.d_max:
            #set speed according to remaining distance to car in front
            #do it for either left or right, according to witch is closer

            self.own_vel.data = (min(self.fl,self.fr) * self.kp) - self.offset

        else:
            self.own_vel.data = self.max_speed
        # print(self.fl,self.fr)
        # print(self.own_vel.data)
        self.lin_vel_pub.publish(self.own_vel)

    def overtaking_manoeuvre(self):
        published = False
        published_2 = False
        while self.overtaking:
            if not self.switch_done and not published:
                # print("OVERTAKING: SWITCHING LEFT")
                lane_switch_commando = UInt8MultiArray()
                lane_switch_commando.data = [1,1]
                self.pub.publish(lane_switch_commando)
                rospy.get_time()
                self.passing_start_time = rospy.get_time()
                published = True

            if published == True and not published_2:
                # print(self.safety)
                if rospy.get_time() - self.passing_start_time >self.own_vel.data*20 and self.safety[1] == 1:
                    # print("OVERTAKING: SWITCHING RIGHT")
                    lane_switch_commando.data = [1,0]
                    self.pub.publish(lane_switch_commando)
                    published_2 = True
                    self.switchback_time = rospy.get_time()
            # second turn command given
            if published_2:
                # 5 seconds later
                if rospy.get_time()-self.switchback_time >5:
                    # end overtaking manoeuvre
                    self.overtaking = False




    def overtaking_check(self):
        """
        First is checked if the car in front is close enough to start, dependent on a set distance or the distance determined based on time between the cars, the overtaking manoeuvre.
        Second it is checked whether the car in front is driving too slow, if checking wether it's safe to go to the left is started.
        If this is not safe the car adapts it's speed if necessary so it does not crash into the car in front.
        If this is safe the lane switching manoeuvre to the left lane is performed.
        When this is done the car starts checking whether or not it's safe to switch back to the right lane.
        If it is safe the lane switching manoeuvre is performed.
        """

        """
        Input (needed data):
        own speed
        Front left and Front right sensor
        Relative velocity of car in Front


        """
        self.own_speed=self.own_vel.data
        # self.min_rel_vel_fl=(-1)*self.rel_vel_fl

        self.d_overtake = 0.5

        conditions = []
        # not currently overtaking
        conditions.append(not self.overtaking)
        # print(self.distance_to_front())
        # close enough to overtake
        conditions.append(self.distance_to_front()<=self.d_overtake)


        # car in front is slower
        self.calculate_rel_vel()
        # conditions.append(self.rel_vel_fl<0 or self.rel_vel_fr<0)

        # print(self.rel_vel_fl,self.rel_vel_fr)
        # # speed of car in front is within acceptable Range
        # conditions.append(self.rel_vel_fl <-self.acceptable_range and \
        #                   self.rel_vel_fr <-self.acceptable_range)

        # check if safe to switch left
        conditions.append(self.safety[0] == 1)
        # print(conditions)

        # all conditions are met. do overtaking manoeuvre
        rospy.get_time()
        if all(conditions):
            self.overtaking = True
            self.overtaking_start_time = rospy.get_time()



    def subscribers(self):
        while not rospy.is_shutdown():
            rospy.Subscriber(self.namespace+ "/range/rl", Range, self.callback_rl)
            rospy.Subscriber(self.namespace+ "/range/fl", Range, self.callback_fl)
            rospy.Subscriber(self.namespace+ "/range/fr", Range, self.callback_fr)
            rospy.Subscriber(self.namespace+ "/lane_switching_flag", UInt8MultiArray, self.callback_safety)  #niet zeker of die Float64MultiArray klopt
            rospy.Subscriber(self.namespace+ "/switch_done", Bool, self.callback_switch_done)
            rospy.Subscriber("/killswitch", Bool, self.callback_killswitch)
            rospy.spin()

            #subscribe to speed of car infront of you
d_min = 0.1 #min allowed range to car in front can also be 0.1
d_max = 0.375 #max prefered range to car in front
r_max = 0.9 #max range of range sensor
#o
kp = max_speed/(d_max-d_min)
offset = kp*d_min
rosbot = SensorValues(namespace,max_speed,kp,offset)
rosbot.subscribers()
