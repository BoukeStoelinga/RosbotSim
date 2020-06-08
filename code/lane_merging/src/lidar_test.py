#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import sys
import numpy as np



def callback(scan, x_coordinate2):
    #find indices of relevant values (values0 that are not ) of intensity array
    intensities = np.array(scan.intensities)
    location_relevant_values = np.where(intensities != 0)[0]

    #find where there is a gap in the ranges and split into arrays accordingly
    dif = np.diff(location_relevant_values) #take the difference of the relevant values
    breakpoints = np.where(dif > 10)[0] + 1 #what index the gap between the arrays is bigger than 10 (where there is gaps bigger than 10 in the intensities) add 1 for next step
    location_relevant_arrays = np.array(np.array_split(location_relevant_values, breakpoints)) #split the relevant_values array in arrays that belong together

    # take the ranges with correct indices
    ranges = scan.ranges                        # array of the ranges
    np_ranges = np.array(ranges)                # make np array of the range arrays
    object_ranges = []                          # make a dynamic array (a List)
    for entry in location_relevant_arrays:      # make sure you do this for every entry in the location_relevant_arrays
        object_ranges.append([ranges[np.min(entry):np.max(entry)]])     #make range depend on the amount of entries in the location_relevant_arrays

    # print(object_ranges)
    np_object_ranges = np.array(object_ranges)  # make np array of the object ranges array
    minimums = np.zeros(len(np_object_ranges))
    locations = np.zeros(len(np_object_ranges))
    angles = np.zeros(len(np_object_ranges))
    x_coordinate = np.zeros(len(np_object_ranges))
    y_coordinate = np.zeros(len(np_object_ranges))
    for j in range(len(minimums)):
        minimums[j] = np.amin(np_object_ranges[j,0])
        locations[j] = np.where(np_ranges == minimums[j])[0]
        angles[j] = locations[j]/(len(np_ranges))*2*np.pi #better version would be angle_minimum + locations * angle_increment
        x_coordinate[j] = np.cos(angles[j])*minimums[j]
        y_coordinate[j] = np.sin(angles[j])*minimums[j]
        #x_rel_vel[j] = x_coordinate[j]-x_coordinate[j-1]
    print("x=",x_coordinate)
    # print("y=",y_coordinate)

    # x_coordinate2 = []*len(np_object_ranges)
    # rel_speed_x = x_coordinate - x_coordinate2
    # print(rel_speed_x)
    # x_coordinate = x_coordinate2


#calculate relative speed on x a xis, is positive if the other car comes closer
def rel_speed(x_coordinate, x_coordinate2):
    rel_speed_x = x_coordinate - x_coordinate2
    print(rel_speed_x)

def save(x_coordinate):
    x_coordinate = x_coordinate2
    return x_coordinate2


             #this goes wrong sometimes, implement a try pass thing



#         result = function();
# try:
#     for r in result:
#         #process items
# except TypeError:
#     pass;


def read_lidar(x_coordinate2):
        rospy.Subscriber("/first/scan", LaserScan,callback, x_coordinate2)

def test1():
    x = np.arange(8.0) #relevant_values
    z = [3,5,6,10] #met z de breakpoints2
    y = np.split(x, z) #split cmd
    print(x)
    print(y)

def main():
    # x_coordinate2 = []
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        read_lidar(x_coordinate2)
        # print(x_coordinate)
        # rel_speed(x_coordinate, x_coordinate2)
        # x_coordinate2 = save(x_coordinate)
        rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node('lidar_reading_node', anonymous=True)
        main()
        # test1()
    except rospy.ROSInterruptException:
        pass
