#!/usr/bin/env python
# Houd die eerste regel er in, fixt soms dingen
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from pprint import pprint

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

def return_data(data):
    print("2")
    return data
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    print("1")
    var = rospy.Subscriber("/Range/fl", Range,var2)
    pprint(vars(var))
    print("3")
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()

def stop():
    pass


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
