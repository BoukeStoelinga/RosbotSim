#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import sys
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates


if len(sys.argv) == 1:
    model = "rosbot_f"
    runID = "1"
else:
    model = str(sys.argv[1])
    runID = str(sys.argv[2])


class Recorder:
    def __init__(self, model_name, runID):
        '''
        create file and write header
        '''
        self.model_name = model_name
        self.namespace = self.getnamespace()
        self.runID = runID
        rospy.init_node('pos_recorder', anonymous=True)
        self.status = ModelState()
        # self.doc = str(__file__[:-14]+ "/measurements/" + self.runID + "_" + self.model_name + ".csv")
        self.doc = str(__file__[:-27]+ "/measurements_lane_merging/" + self.runID + ".csv")
        # row = "Timestamp, x, y, yaw"
        row = "Timestamp, x_f, y_f, yaw_f, x_se, y_se, yaw_se, x_th, y_th, yaw_th, x_fo, y_fo, yaw_fo, x_fi, y_fi, yaw_fi"
        with open(self.doc,'w') as d:
            d.write(row + "\n")

    def callback(self,data):
        self.config(data)
        self.get_poses()
        self.write_pose()
        rospy.sleep(0.01)

    def config(self, data):
        '''
        determine the model to record from the received topic
        '''
        names = ["rosbot_f","rosbot_s","rosbot_t","rosbot_fo","rosbot_fi"]
        indices = [data.name.index(name) for name in names if name in data.name]
        print(data.pose[1])
        self.statuses = [data.pose[index] for index in indices]
        # print(self.statuses)
        # self.statuses = data.pose[index]

    def get_poses(self):
        '''
        extract measurements and trandfer from quaternion to yaw
        '''
        self.current_poses = []
        for status in self.statuses:
            cposX = status.position.x
            cposY = status.position.y
            q = status.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = np.arctan(siny_cosp/cosy_cosp)
            self.current_poses.append([cposX,cposY,yaw])

    def write_pose(self):
        '''
        write measuremnts to file
        '''
        t = rospy.get_rostime()
        row_string = str(t.secs) +"."+ str(t.nsecs)
        # row_string = ""
        print(len(self.current_poses))
        for pose in self.current_poses:
            # print(pose)
            pose  = map(str,pose)
            row_string += ","+",".join(pose)
        # print(row_string)
        # row = str(t.secs) + ". " + str(t.nsecs) + ", "  + str(self.cposX) + ", " + str(self.cposY) + ", " + str(self.yaw)
        with open(self.doc,'a') as d:
            d.write(row_string + "\n")

    def subscribers(self):
        """
        Excecute subscribers
        """
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        rospy.spin()

    def getnamespace(self):
        '''
        Very ugly solution, but it works...
        '''
        if self.model_name == "rosbot_f":
            ns = "/first"
        elif self.model_name == "rosbot_s":
            ns = "/second"
        elif self.model_name == "rosbot_t":
            ns = "/third"
        elif self.model_name == "rosbot_fo":
            ns = "/fourth"
        elif self.model_name == "rosbot_fi":
            ns = "/fifth"

        return ns

Recorder = Recorder(model, runID)
Recorder.subscribers()
