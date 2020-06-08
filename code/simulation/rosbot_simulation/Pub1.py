#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Twist

# from std_msgs.msg import String

def straight_line():
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = 0

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.init_node('ikbenpython', anonymous=True)
    rate = rospy.Rate(2) # 10hz
    x = 1
    while not rospy.is_shutdown():
        vel.linear.x = x
        rospy.loginfo(vel)
        pub.publish(vel)
        x += 1
        if x > 10:
            x = 0
        rate.sleep()


if __name__ == '__main__':
    try:
        straight_line()
    except rospy.ROSInterruptException:
        pass
