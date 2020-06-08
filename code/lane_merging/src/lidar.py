#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import sys
import numpy as np
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

def callback(scan):
    # print(scan.header)
    # print(scan.angle_increment)
    # print(scan.range_min)
    # print(scan.ranges)
    intensities = np.array(scan.intensities) #array of the intensities
    location_relevant_values = np.where(intensities != 0)[0]  #relevant values are values that are not 0
    # use location_relevant_values = np.nonzero( intensities) to try to improve speed
    dif = np.diff(location_relevant_values) #take the difference of the relevant values
    breakpoints = np.where(dif > 10)[0] + 1 #what index the gap between the arrays is bigger than 10 (where there is gaps bigger than 10 in the intensities) add 1 for next step

    location_relevant_arrays = np.array(np.array_split(location_relevant_values, breakpoints)) #split the relevant_values array in arrays that belong together



    ranges = scan.ranges                        # array of the ranges
    np_ranges = np.array(ranges)                # make np array of the range arrays
    object_ranges = []                          # make a dynamic array (a List)
    for entry in location_relevant_arrays:      # make sure you do this for every entry in the location_relevant_arrays
        object_ranges.append([ranges[np.min(entry):np.max(entry)]])     #make range depend on the amount of entries in the location_relevant_arrays

    # print(object_ranges)
    np_object_ranges = np.array(object_ranges)  # make np array of the object ranges array
    # print("ranges", np_object_ranges)
    # print("ranges1",np_object_ranges[0,0])
    # print("ranges2",np_object_ranges[1,0])
    # print("ranges3", np_object_ranges[2,0])

    # print("lentgh", len(np_object_ranges))



    minimums = np.zeros(len(np_object_ranges))
        for j in range(len(minimums)):
            minimums[j] = np.amin(np_object_ranges[j,0])
            locations[j] = np.where(np_ranges == minimums[j])
        print("mins=",minimums)
        print("locations",locations)

             #this goes wrong sometimes, implement a try pass thing


#         result = function();
# try:
#     for r in result:
#         #process items
# except TypeError:
#     pass;



    # for en in np_object_ranges:
    #     min_object_ranges[en]=np.amin(np_object_ranges[en])  # determine minimum value in all range arrays
    #     location_min_ranges[en]=np.where(min_object_ranges[en]) #determine index of minimum value range arrays
    #
    # print("minimums =",min_object_ranges)
    # print("locations",location_min_ranges)

# ('minimums =', (0.9343463778495789, 0.9543922543525696, 0.9478777050971985, 0.9397414326667786, 0.9294841289520264, 0.9111793041229248, 0.9285654425621033, 0.9503731727600098, 0.9303084015846252, 0.9283803701400757, 0.9268149733543396, 0.9212576150894165, 0.9233800768852234, 0.9291691184043884, 0.9192265868186951, 0.9235718846321106, 0.9100285172462463, 0.9043178558349609, 0.9194201827049255, 0.9150504469871521, 0.9261188507080078, 0.8962540626525879, 0.917637050151825, inf, 0.9359669089317322, 0.9293578267097473, 0.9072414040565491, 0.9241660833358765))
# ('locations', (array([ 0,  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16,
#        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27]),))









    # for i in range(len(location_relevant_arrays)-1):
    #     relevant_ranges[i,1] = np.zeros(len(location_relevant_arrays[i])-1)
    #     relevant_ranges[i,1] = np_ranges[location_relevant_arrays[i]]
    #
    # print("ranges =", relevant_ranges)
    # print(location_relevant_arrays.shape)

    # plt.plot(scan.ranges)
    # plt.show()
    # if rospy.is_shutdown():
    #     sys.exit()

def read_lidar():
    rospy.init_node('lidar_reading_node', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("/first/scan", LaserScan,callback)
        rate.sleep()

def test1():
    x = np.arange(8.0) #relevant_values
    z = [3,5,6,10] #met z de breakpoints2
    y = np.split(x, z) #split cmd
    print(x)
    print(y)


if __name__ == "__main__":
    try:
        read_lidar()
        # test1()
    except rospy.ROSInterruptException:
        pass
