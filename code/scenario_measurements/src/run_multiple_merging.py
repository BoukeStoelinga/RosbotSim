#!/usr/bin/env python
import os
import signal
import subprocess
import time
import rospy
from std_msgs.msg import Bool
import signal
import numpy as np
from itertools import combinations


rospy.init_node('scenario_master', anonymous=True)
pub = rospy.Publisher('/killswitch', Bool, queue_size=10)

#CHANGE START PARAMETERS HERE
#NOT USING A ROSBOT? PUT IT ASIDE WITH VELOCITY 0
# position_dict = dict()
# position_dict["x_first"] = -8.2
# position_dict["y_first"] = 0
# position_dict["x_second"] = -8.6
# position_dict["y_second"] = 0
# position_dict["x_third"] = -9
# position_dict["y_third"] = 0
# position_dict["x_fourth"] = -9.4
# position_dict["y_fourth"] = 0
# position_dict["x_fifth"] = -9.8
# position_dict["y_fifth"] = 0
velocity_dict = dict()
velocity_dict["max_vel_first"] = 0.1
velocity_dict["max_vel_second"] = 0.1
velocity_dict["max_vel_third"] = 0.1
velocity_dict["max_vel_fourth"] = 0.1
velocity_dict["max_vel_fifth"] = 0.1

## Make arrays with the possible starting positions
start_positions_black = np.zeros((6, 2))
start_positions_red = np.zeros((3, 2))
token = 0
for i in [0.3, 0.0]:
    for k in [-0.7, -1.2, -1.7]:
        start_positions_black[token] = [k, i]
        token += 1

start_positions_black = np.delete(start_positions_black, 1, axis = 0)

token = 0
for i in [-0.7, -1.2, -1.7]:
    start_positions_red[token] = [i, -0.3]
    token += 1

## make an array with all starting positions per sim
token = 0
sim_start_pos = np.zeros((45, 5, 2))

#1 bot in red lane
black_start_pos_1Red = combinations(range(5), 4)
black_start_pos_1Red = [list(entry) for entry in black_start_pos_1Red]

for i in range(3):
    for subset in black_start_pos_1Red:
        sim_start_pos[token, 0, :] = start_positions_red[i]
        sim_start_pos[token, 1:, :] = start_positions_black[subset]
        token += 1

#2 bots in red lane
black_start_pos_2Red = combinations(range(5), 3)
black_start_pos_2Red = [list(entry2) for entry2 in black_start_pos_2Red]
red_start_pos_2Red = combinations(range(3), 2)
red_start_pos_2Red = [list(entry3) for entry3 in red_start_pos_2Red]

for i in red_start_pos_2Red:
    for subset in black_start_pos_2Red:
        sim_start_pos[token, :2, :] = start_positions_red[i]
        sim_start_pos[token, 2:, :] = start_positions_black[subset]
        token += 1

def convert_to_dict(starting_positions):
    rosbot_names = ["first","second","third","fourth","fifth"]
    # iterate for 45 simulations
    position_dictionaries = []
    for simulation_setup in starting_positions:
        position_dict = dict()
        # for every rosbot in a simulation
        for i in range(len(rosbot_names)):
            x_rosbot = simulation_setup[i][0]
            y_rosbot = simulation_setup[i][1]
            dict_key_x = "x_" + rosbot_names[i]
            dict_key_y = "y_" + rosbot_names[i]
            position_dict[dict_key_x] = x_rosbot
            position_dict[dict_key_y] = y_rosbot
        position_dictionaries.append(position_dict)

    return position_dictionaries


#convert to dictionary
all_starting_positions = convert_to_dict(sim_start_pos)




# launch lane keeping and lane switching sefety
subprocess.Popen(["roslaunch ang_vel_pub lane_keeping_5.launch"],
                 shell=True, preexec_fn=os.setsid)
subprocess.Popen(["roslaunch lane_switching_safety lane_switching_safety.launch"],
                 shell=True, preexec_fn=os.setsid)
subprocess.Popen(["roslaunch rgbd colour_detect.launch"],
                 shell=True, preexec_fn=os.setsid)



for ID in range(30, 45):

    position_dict = all_starting_positions[ID]
    filename_ident = str(ID)
    for car in sim_start_pos[ID]:
        filename_ident += str(car[0])+"_"
    filename_ident = filename_ident[:-1]
    # change the filename of the current situation, make sure every sim has a different
    # name and doesn't get overwritten (for example car_vel_first_1,2,3 etc)
    position_dict["filename"] = "merging_measurements/5_cars_{}".format(filename_ident)

    # reset the positions to specified in position_dict
    # updates to the position dict should be given above this line
    namespaces = ["/first", "/second", "/third", "/fourth", "/fifth"]
    for namespace in namespaces:
        if "x_" + namespace[1:] in position_dict or "y_" + namespace[1:] in position_dict:
            python_reset_cmd_string = "rosrun decision_py reset_all.py"
            python_reset_cmd_string += " " + namespace
            if position_dict.get("x_" + namespace[1:]) != None:
                python_reset_cmd_string += " {}".format(
                    position_dict.get("x_" + namespace[1:]))
            else:
                python_reset_cmd_string += " 0"
            if position_dict.get("y_" + namespace[1:]) != None:
                python_reset_cmd_string += " {}".format(
                    position_dict.get("y_" + namespace[1:]))
            else:
                python_reset_cmd_string += " 0"
            print(python_reset_cmd_string)

            subprocess.Popen([python_reset_cmd_string],
                             shell=True, preexec_fn=os.setsid)
    # wait for reset to finish (real time 10 seconds)
    time.sleep(10)
    starttime = rospy.get_time()
    starttime = rospy.get_time() #2x for a good start

    # start lane_merge nodes
    # MAKE SURE "exec" is in fromt of the command, otherwise it's not killable
    lane_merge = []
    for namespace in namespaces:
        cmd_lanemerge = "exec rosrun lane_merging lane_merging_adapt_vel_node.py"
        try:
            cmd_lanemerge += " " + namespace + " " + \
                str(velocity_dict["max_vel_" + namespace[1:]])
        except:
            cmd_lanemerge += " " + namespace + " 0"

        lane_merge.append(subprocess.Popen(
            [cmd_lanemerge], shell=True, preexec_fn=os.setsid))

    # wait (2 real) seconds for it to start
    time.sleep(2)

    # start the recorder, record to position_dict["filename"]
    recorder = subprocess.Popen(["exec rosrun scenario_measurements record_data.py " +
                                 position_dict["filename"]], shell=True, preexec_fn=os.setsid)
    time.sleep(2)
    # wait for (2 real) seconds
    # start decision 5x
    decisions = []
    for namespace in namespaces:
        decisions.append(subprocess.Popen(
            ['exec rosrun decision_py decision.py ' + namespace], shell=True, preexec_fn=os.setsid))

    # simulate for 24 (simulation) seconds, will take a few minutes
    print("simulating {}".format(str(ID)))
    rospy.sleep(14)
    print("simulation {} halfway".format(str(ID)))
    rospy.sleep(14)


    # time.sleep(5*60)  # time to simulate
    print("killing everything")
    # publish to killswitch topic (just leave this here)
    pub.publish(Bool(data=True))
    pub.publish(Bool(data=True))

    # kill lane_merge and decision, both .kill() and os.killpg are neccesary
    a = [lane_merge_instance.kill() for lane_merge_instance in lane_merge]
    a = [os.killpg(os.getpgid(lane_merge_instance.pid), signal.SIGKILL)
         for lane_merge_instance in lane_merge]
    b = [decision.kill() for decision in decisions]
    b = [os.killpg(os.getpgid(decision.pid), signal.SIGKILL)
         for decision in decisions]

    # kill overtaking and recorder
    recorder.kill()
    os.killpg(os.getpgid(recorder.pid), signal.SIGKILL)

    time.sleep(5)


# RESET WHEN DONE WITH SIMULATION (no cars driving forever)
for namespace in namespaces:
    if "x_" + namespace[1:] in position_dict or "y_" + namespace[1:] in position_dict:
        python_reset_cmd_string = "rosrun decision_py reset_all.py"
        python_reset_cmd_string += " " + namespace
        if position_dict.get("x_" + namespace[1:]) != None:
            python_reset_cmd_string += " {}".format(
                position_dict.get("x_" + namespace[1:]))
        else:
            python_reset_cmd_string += " 0"
        if position_dict.get("y_" + namespace[1:]) != None:
            python_reset_cmd_string += " {}".format(
                position_dict.get("y_" + namespace[1:]))
        else:
            python_reset_cmd_string += " 0"
        print(python_reset_cmd_string)

        subprocess.Popen([python_reset_cmd_string],
                         shell=True, preexec_fn=os.setsid)
print("finished full simulation, please restart gazebo and close all python processes")
