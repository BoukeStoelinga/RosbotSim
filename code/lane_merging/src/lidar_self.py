#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import sys
import numpy as np
import time

"""
Header header            # timestamp in the header is the acquisition time of
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis

float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array
"""

class LidarValues():

    def __init__(self,namespace):
        self.ranges = 0
        self.intensities = 0
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_"+'lidar_reading_node', anonymous=True)
        self._index = -1
        self.x_coordinate2 = 0
        self.x_coordinate = 0

    def callback(self, scan):
        self.intensities = np.array(scan.intensities)
        self.ranges = scan.ranges
        self.read_x()


    def read_x(self):
        #find indices of relevant values (values0 that are not ) of intensity array
        location_relevant_values = np.where(self.intensities != 0)[0]
        # print(self.intensities)

        #find where there is a gap in the ranges and split into arrays accordingly
        dif = np.diff(location_relevant_values) #take the difference of the relevant values
        breakpoints = np.where(dif > 10)[0] + 1 #what index the gap between the arrays is bigger than 10 (where there is gaps bigger than 10 in the intensities) add 1 for next step
        location_relevant_arrays = np.array(np.array_split(location_relevant_values, breakpoints)) #split the relevant_values array in arrays that belong together

        # take the ranges with correct indices
        np_ranges = np.array(self.ranges)                # make np array of the range arrays
        object_ranges = []
        # print(self.location_relevant_arrays)                        # make a dynamic array (a List)
        for entry in location_relevant_arrays:
            # print(entry)    # make sure you do this for every entry in the location_relevant_arrays
            object_ranges.append([self.ranges[np.min(entry):np.max(entry)]])     #make range depend on the amount of entries in the location_relevant_arrays

        # print(object_ranges)
        np_object_ranges = np.array(object_ranges)  # make np array of the object ranges array
        self.minimums = np.zeros(len(np_object_ranges))
        self.locations = np.zeros(len(np_object_ranges))
        self.angles = np.zeros(len(np_object_ranges))
        self.x_coordinate = np.zeros(len(np_object_ranges))
        self.y_coordinate = np.zeros(len(np_object_ranges))

        # print(self.np_object_ranges.shape)
        for j in range(len(self.minimums)):
            self.minimums[j] = np.amin(np_object_ranges[j,0])
            self.locations[j] = np.where(np_ranges == self.minimums[j])[0]
            self.angles[j] = self.locations[j]/(len(np_ranges))*2*np.pi #better version would be angle_minimum + locations * angle_increment
            self.x_coordinate[j] = np.cos(self.angles[j])*self.minimums[j]
            self.y_coordinate[j] = np.sin(self.angles[j])*self.minimums[j]

        # self.print_lidar()
        # self.rel_vel_x()
        self._rel_vel_x()
        self.save()

    # def previous_x(self):
    #     self._index += 1
    #     if self.index >= 0:
    #         self.x_coordinate2 = self.x_coordinate
    #     else:
    #         self.x_coordinate2 = 0


    def save(self):
        self.x_coordinate2 = self.x_coordinate

        # self._index -= 1
        # if self._index < 0:
        #     self._index = len(self.x_coordinate)-1
        # return self.x_coordinate[self._index]
    def _rel_vel_x(self):
        self.rel_vel = np.subtract(self.x_coordinate, self.x_coordinate2)
        if self.rel_vel[0] != 0:
            print(self.namespace[1:]+"rel_vel =" + str(self.rel_vel))


    # def rel_vel_x(self):
    #     self.x_coordinate_previous = self.previous_x()
    #     print(self.namespace[1:]+"vorige x",self.x_coordinate_previous)
    #     # self.rel_vel_x = self.x_coordinate - self.x_coordinate_previous
    #     # print(self.namespace[1:]+ "rel_vel_x",self.rel_vel_x)


    def print_lidar(self):
        print(self.namespace[1:]+ " x=",self.x_coordinate)
        # print(self.namespace[1:]+ " y=",self.y_coordinate)

    def subscribers(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            rospy.Subscriber(self.namespace+ "/scan", LaserScan, self.callback)
            rate.sleep()


# def read_lidar(first_lidar, second_lidar, third_lidar):
#         rospy.Subscriber("/first/scan", LaserScan, first_lidar.callback)
#         rospy.Subscriber("/second/scan", LaserScan, second_lidar.callback)
#         rospy.Subscriber("/third/scan", LaserScan, third_lidar.callback)


def main():
    lidar = LidarValues("/first")
    lidar.subscribers()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
