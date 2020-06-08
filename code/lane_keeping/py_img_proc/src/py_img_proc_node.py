#!/usr/bin/env python

#DEPENDENCIES
#pip install opencv-python


import roslib

import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = str(sys.argv[1])
class ImageProcess:
    def __init__(self,namespace,debug=False):
        self.namespace = namespace
        rospy.init_node(self.namespace[1:]+"_" + 'python_image_proc_node', anonymous=True)
        self.pub_angular_error = rospy.Publisher(self.namespace+'/angular_error', Float64, queue_size=10) #pub the angular error a ROSbot has
        self.pub_edge_image = rospy.Publisher(self.namespace+'/edge_camera', Image, queue_size=10) #pub what the ROSbot sees in rviz
        self.debug = debug
        self.grey =  4
        self.edges = None
        self.half = 0
        self.standard_error = 0
        self.bridge = CvBridge() #Initialise transfer ROSimage to openCV image
        if self.debug:
            self.debug_image = None


    def callback(self,data):
        edges_left, edges_right = self.convert_image(data)
        rho_l ,theta_l, rho_r, theta_r = self.get_houglines(edges_left,edges_right)
        error = self.get_angular_error(rho_l ,theta_l, rho_r, theta_r)
        self.publish_error(error)

        if self.debug:
            self.publish_image(True, rho_l ,theta_l, rho_r, theta_r)



    def subscribers(self):
        """
        Excecute subscribers
        """
        rospy.Subscriber(self.namespace+'/camera/rgb/image_raw', Image, self.callback)
        rospy.spin()

    def convert_image(self,image):
        """
        convert image from sensor_msgs.image to cv
        convert to grayscale
        detect edges
        split in left and right
        """
        # Convert image to cv2 format (np array)
        cv_image = self.bridge.imgmsg_to_cv2(image, "rgb8")

        #Cutoff the upper part of the image
        cv_image = cv_image[int(len(cv_image)*2/3):-1][:]
        if self.debug:
            self.debug_image= cv_image

        # Convert to grayscale and use edge detection
        gray = cv2.cvtColor(cv_image,cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 0.95, 0.7)
        self.edges = edges



        #split image into right and left (half)
        self.half = int(len(edges[0])/2)
        edges_left = edges[:,:self.half]
        edges_right = edges[:,self.half:]
        return edges_left, edges_right

    def publish_image(self, Hough = False, rho_l=0 ,theta_l=0, rho_r=0, theta_r=0):
        if Hough:
            self.draw_houglines_image(rho_l ,theta_l, rho_r, theta_r)

        publ_image = self.bridge.cv2_to_imgmsg(self.debug_image, encoding="passthrough")

        self.pub_edge_image.publish(publ_image)


    def get_houglines(self, edges_left, edges_right):
        """
        Recognise the lines of the roadsides
        """
        # Draw houghlines in left and right image
        # Get the top two houghlines:
        try:
            top_two_houghlines = cv2.HoughLines(edges_left, 1, np.pi/180, 40)[0:2]
            if top_two_houghlines[0][0][0] < top_two_houghlines[1][0][0]:
                lines_left = top_two_houghlines[1][0]
            else:
                lines_left = top_two_houghlines[0][0]
        # if there's only 1
        except IndexError:
            lines_left = cv2.HoughLines(edges_left, 1, np.pi/180, 40)[0][0]
        # if there's 0
        except:
            lines_left = (0,0)
        if lines_left[1] >1.3:
            lines_left = (0,0)

        # same for right half, seperated because we use errors (errors in left give
        # different result in right)
        try:
            top_two_houghlines = cv2.HoughLines(edges_right, 1, np.pi/180, 40)[0:2]
            if top_two_houghlines[0][0][0] < top_two_houghlines[1][0][0]:
                lines_right = top_two_houghlines[1][0]
            else:
                lines_right = top_two_houghlines[0][0]

        except IndexError:
            lines_right = cv2.HoughLines(edges_right, 1, np.pi/180, 40)[0][0]
        except:
            lines_right = (len(edges_right[0])-1,0)
        if lines_right[1]<2:
            lines_right = (len(edges_right[0])-1,0)

        #Split Houglines into vars
        rho_l,theta_l = lines_left
        rho_r_one,theta_r = lines_right
        rho_r = rho_r_one +np.cos(theta_r)*self.half
        # print("rho_r, theta_r: "+str(rho_r) +" "+str(theta_r))
        
        return rho_l ,theta_l, rho_r, theta_r


    def cross_lines(self,theta_a,rho_a):
        return rho_a/(np.cos(theta_a))-np.tan(theta_a)*(len(self.edges)//2)
    def cross_lines_right(self,theta_a,rho_a):
        return abs(rho_a)/(np.cos(np.pi-theta_a))+ np.tan(np.pi-theta_a)*(len(self.edges)//2)

    def get_angular_error(self, rho_l ,theta_l, rho_r, theta_r):
        # Get intersection of both houghlines, matrix to solve and vector

        #Determine the angular error with respect to the road
        try:
            try:
                x_left = self.cross_lines(theta_l,rho_l)
                x_right = self.cross_lines_right(theta_r,rho_r)#
            except:
                pass
            if theta_l == 0:
                x_left =  0
            if theta_r == 0:
                x_right = len(self.edges[0])
            pixel_scale = (x_right-x_left)/0.28
            self.middle_pixels = (x_left+x_right)/2

            self.x_left = x_left
            self.x_right = x_right
            middle_of_track_diff_pixels = (len(self.edges[0])/2)-(x_left+x_right)/2
            middle_track_error =-middle_of_track_diff_pixels/pixel_scale
        except Exception as e:
            # print(e)
            # print(rospy.get_time())
            middle_track_error = self.standard_error

        if self.debug:
            pass
            # print(angular_error)
        return middle_track_error


    def publish_error(self,angular_error):
        """
        Publish the angular error in the namespace
        """
        publ_error = Float64()
        publ_error.data = angular_error

        self.pub_angular_error.publish(publ_error)

    def draw_houglines_image(self,rho_l ,theta_l, rho_r, theta_r):
        lines = [[rho_l,theta_l],[rho_r,theta_r],[len(self.edges)//2,np.pi/2]]
        for rho,theta in lines:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            cv2.line(self.debug_image,(x1,y1),(x2,y2),(255,0,0),2)
        try:
            cv2.line(self.debug_image,(int(self.middle_pixels),len(self.edges)//2),(int(self.middle_pixels),len(self.edges)//2-20),(255,155,10),2)
            cv2.line(self.debug_image,(int(self.x_left),len(self.edges)//2),(int(self.x_left),len(self.edges)//2-20),(255,155,10),2)
            cv2.line(self.debug_image,(int(self.x_right),len(self.edges)//2),(int(self.x_right),len(self.edges)//2-20),(255,155,10),2)
        except:
            pass




first_camera = ImageProcess(namespace,debug=True)
first_camera.subscribers()
