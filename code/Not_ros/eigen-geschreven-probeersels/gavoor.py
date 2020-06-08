#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist

def move():
    # Starts a new node
    rospy.init_node('auto_roundabout', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("start")
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    vel_msg.linear.x = 1

    rospy.loginfo(vel_msg)
    velocity_publisher.publish(vel_msg)


    time.sleep(5)
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    rospy.loginfo(vel_msg)
    velocity_publisher.publish(vel_msg)




if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException:
        pass
