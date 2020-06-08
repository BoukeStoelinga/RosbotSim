#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan


def callback(data):
    rhos = data.ranges
    thetas = np.linspace(-1*np.pi, np.pi, len(rhos))
    plt.clf()
    plt.polar(thetas, rhos, 'x')
    plt.pause(.1)
    # plt.close('all')

if __name__ == '__main__':
    while not rospy.is_shutdown():
        rospy.init_node('plotter', anonymous=True)
        rospy.Subscriber('second/scan', LaserScan, callback)
        rospy.spin()
        plt.show()
    pass
