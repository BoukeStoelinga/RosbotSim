#!/usr/bin/env python
#above thing has to stay

#import relevant libraries and message types
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from pprint import pprint
from std_msgs.msg import Float64
from std_msgs.msg import Bool
import sys

#script is concerning the first rosbot unless differently mentioned when launchin the script

if len(sys.argv) == 1:
    namespace = "/first"
    max_speed = 0.3
else:
    try:
        namespace = sys.argv[1]
        max_speed = float(sys.argv[2])
    except:
        namespace = "/first"
        max_speed = 0.3

class SensorValues():
    """
    Object for reading sensor data of fl and fr
    """
    def __init__(self,namespace,kp,offset,max_speed,front=False):
        self.front = front
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'platooning_node', anonymous=True) #initialise node
        self.fl = 0
        self.fr = 0
        self.kp = kp
        self.offset = offset
        self.max_speed = max_speed
        self.d_min = 0.15 #range in which the ROSbots should follow each other
        self.d_max = 0.375
        self.r_max = 0.9 #maximum rnage that the sensor can detect
        self.pub =rospy.Publisher(self.namespace+'/platoon_linear_vel', Float64, queue_size=10) #publish to a topic that decision.py will subscribe to
        self.vel = Float64()

        self.starttime = 1000

        if namespace == "/first":
            self.kick = False
            self.token = False
        else:
            self.kick = True
            self.token = True

    def callback_fl(self,data):
        """
        Callback to read fl and set variable
        """
        self.fl = data.range
        # print(self.pub,"fl",self.fl)
    def callback_killswitch(self,data):
        if data.data:
            print("KILL_PLATOON"+"-"*40)
            sys.exit()

    def callback_starttime(self, data):
        if data.linear.x > 0.05 and not self.token:
            self.starttime = rospy.get_time()
            self.token = True

    def callback_fr(self,data):
        """
        Callback to read fr and set variable
        """
        self.fr = data.range
        self.calculate_vel()
        self.publish_vel()

    def calculate_vel(self):
        """
        Define speed based on distance to car in front
        """
        #Check if object too close, set vel to 0
        if self.fl < self.d_min or self.fr < self.d_min:
            self.vel.data= 0

        #check if the car is before the maximal allowed distance in platoon
        elif self.fl < self.d_max or self.fr < self.d_max:
            #set speed according to remaining distance to car in front
            #do it for either left or right, according to witch is closer

            self.vel.data = (min(self.fl,self.fr) * self.kp) - self.offset

        #speed up to speed_max to catch up to platoon
        else:
            self.vel.data = self.max_speed

    def publish_vel(self):
        """
        publish own velocity to own publisher
        """

        if rospy.get_time() - self.starttime > 10 and not self.kick1:
            self.vel.data += self.max_speed
            self.kick1 = True
            self.pub.publish(self.vel)
            print("\033[92m\033[1m GOD HAS KICKED /first\033[0m")
            rospy.sleep(1)

        if rospy.get_time() - self.starttime > 20 and not self.kick2:
            self.vel.data -= (self.max_speed/2)
            self.kick2 = True
            self.pub.publish(self.vel)
            print("\033[92m\033[1m GOD HAS KICKED /first\033[0m")
            rospy.sleep(1)

        else:
            self.pub.publish(self.vel)

    def subscribers(self):
        rate = rospy.Rate(10) #set frequency of loop to 10hz
        while not rospy.is_shutdown():
            rospy.Subscriber(self.namespace+ "/range/fl", Range, self.callback_fl)
            rospy.Subscriber(self.namespace+ "/range/fr", Range, self.callback_fr)
            rospy.Subscriber("/killswitch", Bool, self.callback_killswitch)
            rospy.Subscriber(self.namespace+'/cmd_vel', Twist, self.callback_starttime)
            rate.sleep()


d_min = 0.1 #min allowed range to car in front can also be 0.1
d_max = 0.375 #max prefered range to car in front
r_max = 0.9 #max range of range sensor

kp = max_speed/(d_max-d_min) #gain factor
offset = kp*d_min

rosbot = SensorValues(namespace,kp,offset,max_speed)
rosbot.subscribers()
