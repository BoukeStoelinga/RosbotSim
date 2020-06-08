#!/usr/bin/env python
#thing above has to stay

#import relevant libraries
import rospy
from geometry_msgs.msg import Twist



def main():

    vel = Twist()

    #set all velocities to 0
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    # publish these velocities for the first second and third rosbot
    pub_f = rospy.Publisher('first/cmd_vel', Twist, queue_size=10)
    pub_s = rospy.Publisher('second/cmd_vel', Twist, queue_size=10)
    pub_t = rospy.Publisher('third/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) #set frequency of loop to 1hz
    while not rospy.is_shutdown(): #loop is needed for the ROSbots to react to it, without loop they don't stop, stop the script with ctrl+c
        pub_f.publish(vel)
        pub_s.publish(vel)
        pub_t.publish(vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('everybody_stooopsss', anonymous=True) #initialise node
        main()
    except rospy.ROSInterruptException:
        pass
