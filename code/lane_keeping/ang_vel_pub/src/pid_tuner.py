#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import sys
import time
import os



if len(sys.argv) == 1:
    namespace = "/first"
else:
    namespace = sys.argv[1]


class PIDTuner:
    def __init__(self,namespace,debug=False):
        self.namespace = namespace
        rospy.init_node('python_PID_tuner', anonymous=True)
        self.pub= rospy.Publisher(self.namespace+'/pid_tunings', Float64MultiArray, queue_size=10)
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.filename = __file__[:-12] + namespace[1:]+"_pid_tunings.txt"

    def write_pid_tunings(self):
        with open(self.filename,"w+") as f:
            f.write("{},{},{}".format(self.Kp,self.Ki,self.Kd))

    def read_pid_tunings(self):
        with open(self.filename,"r") as f:
            contents = f.read()
        tunings_as_string = contents.split(",")
        try:
            self.Kp = float(tunings_as_string[0])
            self.Ki = float(tunings_as_string[1])
            self.Kd = float(tunings_as_string[2])
        except Exception as e:
            print(e)

    def publish_tunings(self):
        tunings = Float64MultiArray()
        tunings.data = [self.Kp,self.Ki,self.Kd]
        self.pub.publish(tunings)

    def __repr__(self):
        return "Kp = " + str(self.Kp) +" Ki = " + str(self.Ki) + " Kd = " + str(self.Kd)


    def menu(self):
        self.read_pid_tunings()
        while not rospy.is_shutdown() or kp == "exit" or ki == "exit" or kd == "exit":
            print("-"*30)
            print("Current tunings are:")
            print(self)

            try:
                kp = input("Kp = ")
                kp = float(kp)
                kp_valid = True
            except:
                print("invalid, keeping previous kp")
                kp_valid = False
            # print("What value of ki? (type exit for exit, invalid number doesn't change Ki)")
            try:
                ki = input("Ki = ")
                ki = float(ki)
                ki_valid = True
            except:
                print("invalid, keeping previous ki")
                ki_valid = False
            # print("What value of kd? (type exit for exit, invalid number doesn't change Kd)")
            try:
                kd = input("Kd = ")
                kd = float(kd)
                kd_valid = True
            except:
                print("invalid, keeping previous kd")
                kd_valid = False
            if kp_valid:
                self.Kp = kp
            if ki_valid:
                self.Ki = ki
            if kd_valid:
                self.Kd = kd
            print("Publishing and saving tunings...")
            self.publish_tunings()
            self.write_pid_tunings()
        exit()





pid = PIDTuner(namespace)
pid.menu()
