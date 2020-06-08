#!/usr/bin/env python

"""
What still needs to be added:
- Now ROSbot on red lane will decrease speed when it cannot lane change
    Improvement options: make an option to increase speed and or make the decrease dependant on relative velocity or the other ROSbot.
"""

import rospy
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import  UInt8
from pprint import pprint
import sys
import numpy as np

if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]

class MergingController():
    def __init__(self, namespace, kp, offset, max_speed):
        self.namespace = namespace
        #put your variables and constants here
        rospy.init_node(self.namespace[1:]+"_" + 'Lane_merging_node', anonymous=True)
        self.pub_left_switch=UInt8MultiArray()
        #self.pub_right_switch=Bool()
        self.own_vel = Float64()
        self.pub_left_switch = rospy.Publisher(self.namespace+'/lane_switch_bool', UInt8MultiArray, queue_size=10)
        #self.pub_red_lin_vel = rospy.Publisher(self.namespace+'/red_lin_vel', Float64, queue_size=10)
        self.lin_vel_pub =rospy.Publisher(self.namespace+'/red_lane_vel', Float64, queue_size=10)
        self.length_red = 6         #length of the red stroke in meters
        self.red_stop_fraction=0.8    #fraction of how far on red track untill end
        self.d_min = 0.1 #min allowed range to car in front can also be 0.1
        self.d_max = 0.375 #max prefered range to car in front
        self.r_max = 0.9 #max range of range sensor
        self.fl=0
        self.fr=0
        self.kp = kp
        self.offset = offset
        self.max_speed = max_speed
        self.safety = [False, False]
        self.initial_vel = 0
        self.distance_red_strip = 0
        self.current_time = rospy.get_time()
        self.switch_in_progress = False

    def callback_fl(self,data):     #callback to read fl and set variable
        """
        Callback to read fl and set variable
        """
        self.fl = data.range

    def callback_fr(self,data):     #callback to read fr and set variable (value in meter)
        """
        Callback to read fr and set variable
        """
        self.fr = data.range

    def callback_rgbd_colour(self, data):
        """
        Read out which colour the lane is in which the ROSbot is driving.
        Start the part of the script that nees to be used based on the colour.
        """
        self.rgbd_colour = data.data
        if self.rgbd_colour == 0:        #then you are driving in the red lane
            self.red_lane()             #turn on red lane function
            # print("DEBUG:" + self.namespace + ":\t lane is red")
        elif self.rgbd_colour == 1:      #then you are driving in the green lane
            self.green_lane()           #turn on green lane fucntion
            # print("DEBUG:" + self.namespace + ":\t lane is green")
        elif self.rgbd_colour == 2:      #you are now driving in the black lane or not on the track...
            self.calculate_own_vel()    #turn on platooning function
            # print("DEBUG:" + self.namespace + ":\t lane is black")
        else:
            print("DEBUG" + self.namespace+ "\t no red/green/black registerd")

    def callback_safety(self,data):
        """
        Callback to read the array with [left safe, right safe]
        """
        self.safety = list(map(ord, data.data))

    def callback_detected_objects(self, data):
        detected_objects_flat = data.data           #1D array
        N_detected = int(len(detected_objects_flat)/4)
        detected_objects = np.zeros((N_detected, 4))

        for k in range(N_detected):
            detected_objects[k] = np.array(detected_objects_flat[4*k:4*k+4])

        self.detected_objects = detected_objects

    def callback_switch_done(self, data):
        self.switch_in_progress = not data.data

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
        self.lin_vel_pub.publish(self.own_vel)


    def red_lane(self):
        """
        Define what the ROSbot needs to do while driving on the red lane.
        The red lane is the lane for which it is necessary to lane merge to the left.
        As soon as it is safe to lane merge lane switch to left.
        When it is not safe to lane merge, adapt, if necessary, your speed to a car infront or because the red stroke is ending.
        """

        if self.safety[0] == True and not self.switch_in_progress:
            #safe to merge to left lane
            self._switch_left() #[switch yes/no, left/right]
        elif self.safety[0] == False and not self.switch_in_progress:      #decrease speed decrease is dependant on how far it is on the insertion strip
            #estimate how far you are at red strip
            self.distance_red_strip += self.d_time()*self.own_vel.data
            if self.distance_red_strip >= self.red_stop_fraction*self.length_red:
                if self.safety[0] == False:
                    self.own_vel.data=0
                    print("DEBUG" +  self.namespace + "set speed to 0")
                    self.lin_vel_pub.publish(self.own_vel)
                else:
                    #safe to merge to left lane
                    self._switch_left()
            else:
                self.calculate_own_vel()
                print("DEBUG" + self.namespace +"calculate own velocity red")
            # self.vel_array.append(self.inital_vel, self.red_lin_vel)      #fill an array with all the velocities the rosbot had since getting on the red lane
            # self.gem_vel = np.mean(self.vel_array)
            # self.d_time=rospy.get_time()-self.start_red_time
            # self.distance_red_strip=self.gem_vel*self.time_red_strip
            # #decrease speed dependant on how much space is left
            # self.red_lin_vel=((self.length_red-self.distance_red_strip)/self.length_red)*self.initial_vel


    def _switch_left(self):
        lane_switch_commando = UInt8MultiArray()
        lane_switch_commando.data = [1,1]
        self.pub_left_switch.publish(lane_switch_commando)
        rospy.sleep(0.01)
        lane_switch_commando.data = [0,0]
        self.pub_left_switch.publish(lane_switch_commando)
        self.distance_red_strip = 0

    def _switch_right(self):
        lane_switch_commando = UInt8MultiArray()
        lane_switch_commando.data = [1,0]
        self.pub_left_switch.publish(lane_switch_commando)      #deze moet LEFT blijven heten ivm publisher
        rospy.sleep(0.01)
        lane_switch_commando.data = [0,0]
        self.pub_left_switch.publish(lane_switch_commando)

    def d_time(self):
        """
        Update time difference
        """
        self.previous_time = self.current_time
        self.current_time = rospy.get_time()
        return self.current_time - self.previous_time


    def green_lane(self):
        """
        Define what the ROSbot needs to do while driving on the green lane.
        When driving on the green lane try to make space for the merging cars from the right lane.
        Do this by lane merging to the left is necessary. If this is not possible keep driving and keeping enough distance from the car in front.
        """
        if self.safety[1]== True and not self.switch_in_progress:      #no ROSbot drives on your right
            self.calculate_own_vel()
        elif self.safety[0] == True and not self.switch_in_progress:          #safe to merge to left lane
            self._switch_left()    #if this is done you will switch to calculate_own_vel() function [switch, left/right]
        elif not self.switch_in_progress:     #platooning/cruising
            self.calculate_own_vel()



    def subscribers(self):
        rospy.Subscriber(self.namespace+ "/range/fl", Range, self.callback_fl)
        rospy.Subscriber(self.namespace+ "/range/fr", Range, self.callback_fr)
        rospy.Subscriber(self.namespace+ "/lane_colour", UInt8, self.callback_rgbd_colour)     #0 is red, 1 is green, 2 is blue
        rospy.Subscriber(self.namespace+ "/lane_switching_flag", UInt8MultiArray, self.callback_safety)
        rospy.Subscriber(self.namespace+'/switch_done', Bool, self.callback_switch_done)
        rospy.Subscriber(self.namespace+ "/detected_objects", Float64MultiArray, self.callback_detected_objects)

#        rospy.subscriber(self.namespace+ "/cmd_vel", Twist, self.callback_cmd_vel)
        rospy.spin()        #makes sure thatit keeps subscribing




d_min = 0.1 #min allowed range to car in front can also be 0.1
d_max = 0.375 #max prefered range to car in front
r_max = 0.9 #max range of range sensor
max_speed = 0.3 #only for second and third
kp = max_speed/(d_max-d_min) #gain factor
offset = kp*d_min
#first rosbot has a slower max_speed so that the other rosbots can catch up to it
if namespace == "/first":
    max_speed = 0.3
if namespace == "/third":
    max_speed = 0.3
rosbot = MergingController(namespace,kp,offset,max_speed)
rosbot.subscribers()
