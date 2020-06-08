#!/usr/bin/env python

#DEPENDENCIES
#pip install opencv-python

import roslib
import rospy
import cv2
import numpy as np
from std_msgs.msg import UInt8
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from cv_bridge import CvBridge, CvBridgeError
import sys

if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = str(sys.argv[1])

class ColourProcess:
    def __init__(self,namespace):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'rgbd_colour_node', anonymous=True)
        self.pub_lane_colour = rospy.Publisher(self.namespace+'/lane_colour', UInt8, queue_size=10) #pub the lane colour
        self.bridge = CvBridge() #Initialise transfer ROSimage to openCV image


    def callback(self,data):

        self.colour = UInt8()

        # Convert image to cv2 format (np array)
        cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        # cv_imgae = (hoogte,breedte,color channels) for channels: 0 = red, 1 = green, 2 = blue

        cv_image_red = cv_image[::-20,280:380,0]    #size of this array is (24, 100)
        cv_average_red = np.average(cv_image_red)
        cv_image_green = cv_image[::-20,280:380,1]    #size of this array is (24, 100)
        cv_average_green = np.average(cv_image_green)

        if cv_average_red > 120 and cv_average_green < 60:
            self.colour = 0 #meaning the colour is red

        elif cv_average_green > 120 and cv_average_red < 60:
            self.colour = 1 #meaning the colour is green

        else:
            self.colour = 2 #meaning its another colour

        # print(colour)
        self.pub_lane_colour.publish(self.colour)

    def subscribers(self):
        """
        Excecute subscribers
        """
        rospy.Subscriber(self.namespace+'/camera/rgb/image_raw', Image, self.callback)
        rospy.spin()


camera = ColourProcess(namespace)
camera.subscribers()
