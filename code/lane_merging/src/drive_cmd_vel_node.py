#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
def drive():
    rospy.init_node('drive_cmd_vel_node', anonymous=True) #definieren hoe de node heet + anonymouis zodat we hem meerdere keren apart zouden kunnen launchen
    pub_first = rospy.Publisher('/first/cmd_vel', Twist, queue_size=10)
    pub_second = rospy.Publisher('/second/cmd_vel', Twist, queue_size=10)
    pub_third = rospy.Publisher('/third/cmd_vel', Twist, queue_size=10)
    set_vel1 = Twist()
    set_vel2 = Twist()
    set_vel3 = Twist()
    set_vel1.angular.z = 1
    set_vel2.angular.z = -1
    set_vel3.angular.z = 2
    while not rospy.is_shutdown():
        pub_first.publish(set_vel1)
        pub_second.publish(set_vel2)
        pub_third.publish(set_vel3)

if __name__=="__main__":
    drive()
