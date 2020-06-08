#!/usr/bin/env python

import roslib
import rospy
import numpy as np
import sys
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool


if len(sys.argv) == 1:
    filename = "1"
else:
    filename = sys.argv[1]


class Recorder:
    def __init__(self, runID):
        '''
        create file and write header
        '''
        # self.model_name = model_name
        self.runID = runID
        rospy.init_node('pos_recorder', anonymous=True)
        self.status = ModelState()
        self.prev_time = 0

        # self.doc = str(__file__[:-14]+ "/measurements/" + self.runID + "_" + self.model_name + ".csv")
        self.doc = str(__file__[:-14] + "measurements/" + self.runID + ".csv")
        self.d = open(self.doc, 'a')
        # row = "Timestamp, x, y, yaw"
        row = "Timestamp, x_f, y_f, yaw_f, x_se, y_se, yaw_se, x_th, y_th, yaw_th, x_fo, y_fo, yaw_fo, x_fi, y_fi, yaw_fi"
        with open(self.doc, 'w') as d:
            d.write(row + "\n")

    def callback(self, data):
        current_time = rospy.get_time()
        # print(current_time)
        # print(current_time.secs+(0.001*current_time.nsecs)-self.prev_time.secs-(0.001*self.prev_time.nsecs))
        if current_time - self.prev_time >=0.05:
            self.config(data)
            self.get_poses()
            self.write_pose()
            self.prev_time = current_time
        # rospy.sleep(0.01)

    def callback_killswitch(self, data):
        if data.data:
            sys.exit()

    def config(self, data):
        '''
        determine the model to record from the received topic
        '''
        names = ["rosbot_f", "rosbot_s", "rosbot_t", "rosbot_fo", "rosbot_fi"]
        indices = [data.name.index(name)
                   for name in names if name in data.name]

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
            yaw = np.arctan(siny_cosp / cosy_cosp)
            self.current_poses.append([cposX, cposY, yaw])

    def write_pose(self):
        '''
        write measuremnts to file
        '''
        t = rospy.get_rostime()
        row_string = str(t.secs) + "." + str(t.nsecs)
        # row_string = ""

        for pose in self.current_poses:

            pose = map(str, pose)
            row_string += "," + ",".join(pose)
        # print("real_x_pos: " + str(self.current_poses[0][0]))
        # print(row_string)
        # row = str(t.secs) + ". " + str(t.nsecs) + ", "  + str(self.cposX) + ", " + str(self.cposY) + ", " + str(self.yaw)
        # if float(row_string[0:row_string.find(',')])- self.prev_time >= 0.005:
        #     print(row_string)

        self.d.write(row_string + "\n")
        # self.prev_time = float(row_string[0:row_string.find(',')])

    def subscribers(self):
        """
        Excecute subscribers
        """
        rate = rospy.Rate(100)
        #set frequency of loop to 10hz
        while not rospy.is_shutdown():
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback,queue_size=3)
            rospy.Subscriber("/killswitch", Bool, self.callback_killswitch)
            rate.sleep()



Recorder = Recorder(filename)
Recorder.subscribers()
