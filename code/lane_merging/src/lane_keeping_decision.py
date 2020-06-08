#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


def callback(data,args):
    pub = args[0]
    set_vel = args[1]
    set_vel.angular.z = float(data.data)
    print(set_vel)
    pub.publish(set_vel)
def drive():
    rospy.init_node('lane_keeping_python', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    set_vel = Twist()
    set_vel.linear.x = 0.1

    rospy.Subscriber("/ang_vel",Float64,callback,callback_args=(pub,set_vel))
    rospy.spin()
if __name__ == "__main__":

    drive()
    # debug_link_states()
