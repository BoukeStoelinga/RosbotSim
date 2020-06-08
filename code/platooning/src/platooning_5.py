#!/usr/bin/env python
#above thing has to stay

#import relevant libraries and message types
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from pprint import pprint

class SensorValues():
    """
    Object for reading sensor data of fl and fr
    """
    def __init__(self,kp,offset,max_speed,pub):
        self.fl = 0
        self.fr = 0
        self.kp = kp
        self.offset = offset
        self.max_speed = max_speed
        self.d_min = 0.15
        self.d_max = 0.375
        self.r_max = 0.9
        self.pub = pub
        self.vel = Twist()

    def callback_fl(self,data):
        """
        Callback to read fl and set variable
        """
        self.fl = data.range
        print(self.pub,"fl",self.fl)


    def callback_fr(self,data):
        """
        Callback to read fr and set variable
        """
        self.fr = data.range

    def calculate_vel(self):
        """
        Define speed based on distance to car in front
        """
        print("speed calc")
        #Check if object too close, set vel to 0
        if self.fl < self.d_min or self.fr < self.d_min:
            self.vel.linear.x = 0

        #check if the car is before the maximal allowed distance in platoon
        elif self.fl < self.d_max or self.fr < self.d_max:
            #set speed according to remaining distance to car in front
            #do it for either left or right, according to witch is closer

            self.vel.linear.x = (min(self.fl,self.fr) * self.kp) - self.offset
            # if self.fr < self.fl:
            #     self.vel.linear.x = ((self.fr * self.kp) - self.offset)
            # elif self.fl < self.fr:
            #     self.vel.linear.x = ((self.fr * self.kp) - self.offset)

        #speed up to speed_max to catch up to platoon
        else:
            self.vel.linear.x = self.max_speed

    def publish_vel(self):
        """
        publish own velocity to own publisher
        """
        self.pub.publish(self.vel)


def read_values(first_values,second_values,third_values,fourth_values,fifth_values):
    """
    subcribes to relevant range sensors and executes the callback function when a value is received
    """
    rospy.Subscriber("/second/range/fl", Range, second_values.callback_fl)
    rospy.Subscriber("/second/range/fr", Range, second_values.callback_fr)
    rospy.Subscriber("/third/range/fl", Range, third_values.callback_fl)
    rospy.Subscriber("/third/range/fr", Range, third_values.callback_fr)
    rospy.Subscriber("/fourth/range/fr", Range, fourth_values.callback_fr)
    rospy.Subscriber("/fourth/range/fl", Range, fourth_values.callback_fl)
    rospy.Subscriber("/fifth/range/fr", Range, fifth_values.callback_fr)
    rospy.Subscriber("/fifth/range/fl", Range, fifth_values.callback_fl)



def main(pub_f,pub_s,pub_t,pub_fo,pub_fi):
    d_min = 0.15 #min allowed range to car in front can also be 0.1
    d_max = 0.375 #max prefered range to car in front
    r_max = 0.9 #max range of range sensor
    max_speed = 0.325 #only for second and third
    kp = max_speed/(d_max-d_min) #gain factor
    offset = kp*d_min

    #define what first_values, second_values and third_values are and give them certain values (kp, offset, max_speed, pub)
    first_values = SensorValues(kp,offset,0.3,pub_f) # kp,offset,max_speed,pub
    second_values = SensorValues(kp,offset,max_speed,pub_s)
    third_values = SensorValues(kp,offset,max_speed,pub_t)
    fourth_values = SensorValues(kp,offset,max_speed,pub_fo)
    fifth_values = SensorValues(kp,offset, max_speed,pub_fi)

    rate = rospy.Rate(10) #set frequency of loop to 10hz
    while not rospy.is_shutdown():
        read_values(first_values,second_values,third_values,fourth_values,fifth_values) #read sensor values
        first_values.vel.linear.x = first_values.max_speed #first leads platoon and will not require sensor values

        #calculate the new velocities of the following cars
        second_values.calculate_vel()
        third_values.calculate_vel()
        fourth_values.calculate_vel()
        fifth_values.calculate_vel()

        #Publish all velocities
        first_values.publish_vel()
        second_values.publish_vel()
        third_values.publish_vel()
        fourth_values.publish_vel()
        fifth_values.publish_vel()

        rate.sleep() #wait to match loop freqency

def init():
    """
    intitialises the script by defining the node and the publishers, returns the variables for the publishers
    """
    pub_f = rospy.Publisher('first/cmd_vel', Twist, queue_size=10)
    pub_s = rospy.Publisher('second/cmd_vel', Twist, queue_size=10)
    pub_t = rospy.Publisher('third/cmd_vel', Twist, queue_size=10)
    pub_fo = rospy.Publisher('fourth/cmd_vel', Twist, queue_size=10)
    pub_fi = rospy.Publisher('fifth/cmd_vel', Twist, queue_size=10)

    rospy.init_node('platooning_node', anonymous=True)
    return pub_f,pub_s,pub_t, pub_fo, pub_fi

if __name__ == '__main__':
    try:
        pub_f,pub_s,pub_t,pub_fo,pub_fi=init()
        main(pub_f,pub_s,pub_t,pub_fo,pub_fi)
    except rospy.ROSInterruptException:
        pass
