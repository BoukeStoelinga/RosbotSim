#!/usr/bin/env python
# Houd die eerste regel er in, fixt soms dingen
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

"""
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
"""
def setzero(set_vel):
    set_vel.linear.x = 0
    set_vel.linear.y = 0
    set_vel.linear.z = 0
    set_vel.angular.x = 0
    set_vel.angular.y = 0
    set_vel.angular.z = 0


def return_data(data,pub):
    print(data.range)
    vel_cmd = Twist()
    setzero(vel_cmd)
    if data.range < 0.20:
        pub.publish(vel_cmd)
    else:
        vel_cmd.linear.x = 10
        pub.publish(vel_cmd)


    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('ikbenpython', anonymous=True)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # print("1")
    rospy.Subscriber("/range/fl", Range, return_data,callback_args=pub)

    # print("3")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
