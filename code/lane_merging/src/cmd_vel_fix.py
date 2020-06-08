#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates

import numpy as np
import time


# http://docs.ros.org/jade/api/tf/html/python/transformations.html
def drive():
    while True:
        rospy.init_node('lane_merging_node', anonymous=True) #definieren hoe de node heet + anonymouis zodat we hem meerdere keren apart zouden kunnen launchen
        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10) #zeggen waar hij moet gaan publishen
        rospy.Subscriber("/gazebo/model_states", ModelStates,callback,callback_args=pub)
            #zeggen waar hij moet subscriben + elke keer als hij iets leest doet hij callback pub moet hij meenemen omdat hij in die functie ook moet weten waar hij moet publishen
        rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    # rospy.spin() #blijf dit uitvoeren, soort loop

def callback(data,pub):   #de functie die hij elke keer uitvoert als er iets wordt gelezen

    # create ModelState objects of the rosbots (position velocity etc)
    rosbot_f = copystate(data,"rosbot_f")
    print(rosbot_f.pose)
    # transform your velocities you want
    transform_vel(1,rosbot_f)

    #publish to gazebo
    pub.publish(rosbot_f)
    pub.publish(rosbot_s)


def copystate(data,rosbot_name):
    """
    data = LinkState
    rosbot_name = string
    output: current data of specified rosbot
    """
    rosbot_name = rosbot_name
    copy = ModelState()
    index = data.name.index(rosbot_name) #definieer waar hij moet gaan zoeken voor de specifieke rosbot
    copy.model_name = data.name[index] #copy de naam van het model (soort check om te kijken of je wel de goede hebt)
    copy.pose = data.pose[index] #pose is waar hij staat in de gazebo wereld
    copy.twist = data.twist[index]
    return copy

def setzero(twist):
    """
    Receives twist type
    sets all inputs to 0
    usefull for stopping
    """
    if not type(twist) == type(Twist()):
        raise Exception
    twist.linear.x = 0
    twist.linear.y = 0
    twist.linear.y = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = 0

def transform_ref_frame(vec,rosbot_state):
    # Oude functie, waarschijnlijk langzamer
    """
    vec = numpy array ([x,0,0])
    rosbot_state = ModelStates
    return update rosbot_state to proper velocity
    """
    # read in quaternion
    qi = rosbot_state.pose.orientation.x
    qj = rosbot_state.pose.orientation.y
    qk = rosbot_state.pose.orientation.z
    qr = rosbot_state.pose.orientation.w
    # transform to rotation matrix
    new_vel = vec*quat_to_rotmatrix(qi,qj,qk,qr)
    # update rosbot_state
    rosbot_state.twist.linear.x = new_vel[0][0]
    rosbot_state.twist.linear.y = new_vel[1][0]
    rosbot_state.twist.linear.z = new_vel[2][0]

def transform_vel(v_x,rosbot_state):
    """
    vec = numpy array ([x,0,0])
    rosbot_state = ModelStates
    return update rosbot_state to proper velocity
    """
    # read in quaternion
    qi = rosbot_state.pose.orientation.x
    qj = rosbot_state.pose.orientation.y
    qk = rosbot_state.pose.orientation.z
    qr = rosbot_state.pose.orientation.w
    # transform to rotation matrix optimized
    new_vel = rotate_vector(qi,qj,qk,qr,v_x)
    # update rosbot_state
    rosbot_state.twist.linear.x = new_vel[0]
    rosbot_state.twist.linear.y = new_vel[1]
    rosbot_state.twist.linear.z = new_vel[2]

def rotate_vector(qi,qj,qk,qr,v_x):
    """
    v_x = value of x in [x,0,0]
    qi,qj,qk,qr = quaternion x,y,z,w
    returns new rotated velocity vector
    """
    vec = np.array([
    (1-2*qj**2-2*qk**2)*v_x,
    (2*(qi*qj+qk*qr))*v_x,
    (2*(qi*qk-qj*qr))*v_x
    ])
    return vec

def quat_to_rotmatrix(qi,qj,qk,qr):
    # oude functie waarschijnlijk langzamer
    """
    source: https://en.wikipedia.org/wiki/Quaternion
    turns quaternion into rotation matrix
    """
    mat = np.array([\
    [1-2*qj**2-2*qk**2, 2*(qi*qj-qk-qr),2*(qi*qk+qj*qr)],\
    [2*(qi*qj+qk*qr),1-2*qi**2-2*qk**2,2*(qj*qk-qi*qr)],\
    [2*(qi*qk-qj*qr),2*(qi*qr+qj*qk),1-2*qi**2-2*qj**2]
    ]) #TODO shrink matrix to only work with [x,0,0] vec
    return mat

def transform_ref_frame(vec,rosbot_state):
    # Oude functie, waarschijnlijk langzamer
    """
    vec = numpy array ([x,0,0])
    rosbot_state = ModelStates
    return update rosbot_state to proper velocity
    """
    # read in quaternion
    qi = rosbot_state.pose.orientation.x
    qj = rosbot_state.pose.orientation.y
    qk = rosbot_state.pose.orientation.z
    qr = rosbot_state.pose.orientation.w
    # transform to rotation matrix
    new_vel = vec*quat_to_rotmatrix(qi,qj,qk,qr)
    # update rosbot_state
    rosbot_state.twist.linear.x = new_vel[0][0]
    rosbot_state.twist.linear.y = new_vel[1][0]
    rosbot_state.twist.linear.z = new_vel[2][0]






# only execute this file when it's callled, so not when you import it from somewhere
if __name__ == "__main__":
    drive()
    # debug_link_states()
