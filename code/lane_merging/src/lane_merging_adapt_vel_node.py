#!/usr/bin/env python

"""
What still needs to be added:
- Hij lijkt een beetje te stuntellen wat betreft op rood versnellen.
- 2e op rood die rechts achter eentje op groen zit, begint met lane switchen maar eindigd halverwege de switch.
    Omdat ie te dicht op de volgende auto zit en gaat dan lane volgen op tov de rode laan, dus weer terug.

    snelheid na accelleratie blijven vasthouden
    stopstreep stoppen
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

        self.red_stop_fraction=1.3    #fraction of how far on red track untill end
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
        self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
        self.accel_history = np.zeros((20, 1))
        self.wait = False

        self.side_threshold = 0.07
        self.accel_factor = 0.8


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
            self.red_looking_green()             #turn on red lane function
            #print("DEBUG:" + self.namespace + ":\t lane is red")
        elif self.rgbd_colour == 1:      #then you are driving in the green lane
            self.green_looking_red()           #turn on green lane fucntion
            # print("DEBUG:" + self.namespace + ":\t lane is green")
        elif self.rgbd_colour == 2:      #you are now driving in the black lane or not on the track...
            self.calculate_own_vel(1)    #turn on platooning function
            # print("DEBUG:" + self.namespace + ":\t lane is black")
        # else:
        #     print("DEBUG" + self.namespace+ "\t no red/green/black registerd")

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

    def callback_killswitch(self,data):
        if data.data:
            sys.exit()

    def calculate_own_vel(self, accel_factor=1):
        """
        Define speed based on distanc e to car in front
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
            self.own_vel.data = accel_factor*self.max_speed
        self.lin_vel_pub.publish(self.own_vel)
        if accel_factor != 1 and self.safety[0] == False:
            # self.event.wait(self.safety[0] == True)
            self.wait = True
        else:
            self.wait = False

            #nog een if met zorgen dat ie stopt met speed decreasen op groen als er geen auto meer op rechts zit

    def red_looking_green(self):
        """
        Check if safe to lane merge, if so lane merge.
        If not safe, maak een matrix met alleen de waarden y waarden van laan linker baan
        kijk naar welke absolute waarde van x het kleinst is (1e kolom)
        gebruik de x waarde om te bepalen of die achter, zij of voor zit (1e waarde uit die rij negatief, rond 0 of positief)
        relatieve snelheid auto die het dichtst bij zit (pak de 3e kolom uit de rij met laagste x)
        voer bijbehorende actie uit (zie drive spreadsheat)
        """
        if self.safety[0] == True and not self.switch_in_progress:
            #safe to merge to left lane
            self._switch_left() #[switch yes/no, left/right]
            self.accel = 0
            self.accel_history = np.zeros((20, 1))

        elif len(np.unique(self.accel_history)) == 1:
            if not self.wait:
                nearest_car_index = np.argmin(self.detected_objects[:,0])
                nearest_car_x = self.detected_objects[nearest_car_index, 0]
                nearest_car_xd = self.detected_objects[nearest_car_index, 2] #+ self.accel * self.own_vel.data

                vel_threshold = 0.1

                if nearest_car_x > self.side_threshold: #front
                    if nearest_car_xd > vel_threshold: #faster
                        self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                    elif nearest_car_xd < -1* vel_threshold: #slower
                        self.accel = 0
                    else:   #same speed
                        self.accel = 1

                elif nearest_car_x < -1*self.side_threshold: #back
                    if nearest_car_xd > vel_threshold: #faster
                        self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                    elif nearest_car_xd < -1* vel_threshold: #slower
                        self.accel = 0
                    else:   #same speed
                        self.accel = 1

                else:   #to the side
                    if nearest_car_xd > vel_threshold: #faster
                        self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                    elif nearest_car_xd < -1* vel_threshold: #slower
                        self.accel = 0
                    else:   #same speed
                        self.accel = 1

            accel_factor = 1 + self.accel*self.accel_factor
            # print(self.namespace + str(accel_factor) + "\t" + str(nearest_car_xd))
            self.red_lane(accel_factor)

        self.accel_history = np.roll(self.accel_history, -1)
        self.accel_history[-1] = self.accel

    def green_looking_red(self):
        """
        Check if car drives on green.
        Check if not safe to lane merge.
        Determine speed of car closest to you in red lane.
        Determine speed dependant on rel vel of of this car in "def green lane".
        """
        self.accel = 0
        self.accel_history = np.zeros((20, 1))

        if self.safety[1]== True and not self.switch_in_progress:               #no ROSbot drives on your right
            self.calculate_own_vel()
        elif self.safety[0] == True and not self.switch_in_progress:            #el -> so ROSbot drives on your right and safe to merge to left lane
            self._switch_left()
        elif self.safety[0] == False and not self.switch_in_progress:           #is car in red lane but not safe for green to switch to black, so adapt speed
            if self.fl <= self.d_min or self.fr <= self.d_min:                  #when there is a car driving close in front of you it is not interesting to speed up so just keep driving
                self.calculate_own_vel()
            elif len(np.unique(self.accel_history)) == 1:                       # if it had the same value for a while
                try:
                    #filter out all vectors with positive y value (aka only look at rosbots in the right lane)
                    cars_right_index = np.where(self.detected_objects[:, 1] < 0)
                    only_cars_right=self.detected_objects[cars_right_index, :]
                    nearest_car_index = np.argmin(only_cars_right[:,0])       #determine minimum value of the array on the first axis (aka find rosbot closest to you)                                                                           #so the car it's measuring is on the right
                    nearest_car_x = only_cars_right[nearest_car_index, 0]
                    nearest_car_xd = only_cars_right[nearest_car_index, 2]
                    #IndexError: index 1 is out of bounds for axis 0 with size 1

                    vel_threshold = 0.1
                    if nearest_car_x > self.side_threshold: #front
                        if nearest_car_xd > vel_threshold: #faster
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        elif nearest_car_xd < -1 * vel_threshold: #slower
                            self.accel = 1 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        else:   #same speed
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor

                    elif nearest_car_x < -1*self.side_threshold: #back
                        if nearest_car_xd > vel_threshold: #faster
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        elif nearest_car_xd < -1 * vel_threshold: #slower
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        else:   #same speed
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor

                    else:   #to the side
                        if nearest_car_xd > vel_threshold: #faster
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        elif nearest_car_xd < -1 * vel_threshold: #slower
                            self.accel = 1# -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                        else:   #same speed
                            self.accel = 0 # -1: decelerating, 0: cruising, 1:accelating
                            accel_factor = 1 + self.accel*self.accel_factor
                    # print(self.namespace + str(accel_factor) + "\t" + str(nearest_car_xd))
                    self.calculate_own_vel(accel_factor)
                except Exception as  e:
                    # print(e)
                    pass
            else:
                self.calculate_own_vel()

        self.accel_history = np.roll(self.accel_history, -1)
        self.accel_history[-1] = self.accel

        # self.green_lane()

    def red_lane(self, accel_factor):
        """
        Define what the ROSbot needs to do while driving on the red lane.
        The red lane is the lane for which it is necessary to lane merge to the left.
        As soon as it is safe to lane merge lane switch to left.
        When it is not safe to lane merge, adapt, if necessary, your speed to a car infront or because the red stroke is ending.
        """
        #decrease speed decrease is dependant on how far it is on the insertion strip
        #estimate how far you are at red strip
        self.distance_red_strip += self.d_time()*self.own_vel.data         #TODO self.own_vel.data zorgt ervoor dat de rosbot weer gaat rijden na even bij de stop stil te hebben gestaan, zorg dat ie alleen weer gaat rijden als hij kan lane switchen
#        print("DEBUG" + self.namespace + " distance red strip: " + str(self.distance_red_strip))
#        if self.distance_red_strip >= self.red_stop_fraction*self.length_red: #False:
#            self.own_vel.data=0
#            print("DEBUG" +  self.namespace + "set speed to 0")
#            self.lin_vel_pub.publish(self.own_vel)
#            while self.safety[0] != True:       #make sure the ROSbot only starts moving after the stop line for lane switching to the green lane
#                rospy.sleep(0.01)               #niet zeker of dit klopt
#                print("DEBUG" + self.namespace + "waiting after stop")
        if self.safety[0] == True and not self.switch_in_progress:
            self._switch_left()

        else:
            self.calculate_own_vel(accel_factor)
#            print("DEBUG" + self.namespace +"calculate own velocity red")



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
#        print("DEBUG d_time is:" + self.namespace + self.current_time-self.previous_time)




    # def green_lane(self):
    #     """
    #     Define what the ROSbot needs to do while driving on the green lane.
    #     When driving on the green lane try to make space for the merging cars from the right lane.
    #     Do this by lane merging to the left is necessary. If this is not possible keep driving and keeping enough distance from the car in front.
    #     """
    #     if self.safety[1]== True and not self.switch_in_progress:      #no ROSbot drives on your right
    #         self.calculate_own_vel()
    #     elif self.safety[0] == True and not self.switch_in_progress:          #safe to merge to left lane
    #         self._switch_left()    #if this is done you will switch to calculate_own_vel() function [switch, left/right]
    #     elif not self.switch_in_progress:     #platooning/cruising
    #         self.calculate_own_vel()



    def subscribers(self):
        rospy.Subscriber(self.namespace+ "/range/fl", Range, self.callback_fl)
        rospy.Subscriber(self.namespace+ "/range/fr", Range, self.callback_fr)
        rospy.Subscriber(self.namespace+ "/lane_colour", UInt8, self.callback_rgbd_colour)     #0 is red, 1 is green, 2 is blue
        rospy.Subscriber(self.namespace+ "/lane_switching_flag", UInt8MultiArray, self.callback_safety)
        rospy.Subscriber(self.namespace+'/switch_done', Bool, self.callback_switch_done)
        rospy.Subscriber(self.namespace+ "/detected_objects", Float64MultiArray, self.callback_detected_objects)
        rospy.Subscriber("/killswitch", Bool, self.callback_killswitch)

#        rospy.subscriber(self.namespace+ "/cmd_vel", Twist, self.callback_cmd_vel)
        rospy.spin()        #makes sure thatit keeps subscribing




d_min = 0.15 #min allowed range to car in front can also be 0.1
d_max = 0.375 #max prefered range to car in front
r_max = 0.9 #max range of range sensor
max_speed = 0.25 #only for second and third
kp = max_speed/(d_max-d_min) #gain factor
offset = kp*d_min

rosbot = MergingController(namespace,kp,offset,max_speed)
rosbot.subscribers()
