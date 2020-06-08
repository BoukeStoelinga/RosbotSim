#!/usr/bin/env python
import os
import signal
import subprocess
import time
import rospy
from std_msgs.msg import Bool
import signal


rospy.init_node('scenario_master', anonymous=True)
pub = rospy.Publisher('/killswitch', Bool, queue_size=10)

#CHANGE START PARAMETERS HERE
#NOT USING A ROSBOT? PUT IT ASIDE WITH VELOCITY 0
position_dict = dict()
position_dict["x_first"] = -9.8
position_dict["x_second"] = -9.2
position_dict["x_third"] = 0
position_dict["x_fourth"] = 0
position_dict["x_fifth"] = 9
position_dict["y_fifth"] = 7
position_dict["y_fourth"] = 8
position_dict["y_third"] = 9
velocity_dict = dict()
velocity_dict["max_vel_first"] = 0.25
velocity_dict["max_vel_second"] = 0.15
velocity_dict["max_vel_third"] = 0
velocity_dict["max_vel_fourth"] = 0
velocity_dict["max_vel_fifth"] = 0

# launch lane keeping and lane switching sefety
subprocess.Popen(["roslaunch ang_vel_pub lane_keeping_5.launch"],
                 shell=True, preexec_fn=os.setsid)
subprocess.Popen(["roslaunch lane_switching_safety lane_switching_safety.launch"],
                 shell=True, preexec_fn=os.setsid)


for ID in range(0, 10):
    # change the filename of the current situation, make sure every sim has a different
    # name and doesn't get overwritten (for example car_vel_first_1,2,3 etc)
    # position_dict["filename"] = "overtaking_measurements/3_cars_vel_first_{}".format(str(velocity_dict["max_vel_first"]))

    # update the velocity of first (overtaking)
    velocity_dict["max_vel_first"] = 0.25 + 0.01 * ID
    position_dict["filename"] = "overtaking_measurements/1_cars_run_3_vel_first_{}".format(str(velocity_dict["max_vel_first"]))


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
            # print(python_reset_cmd_string)

            subprocess.Popen([python_reset_cmd_string],
                             shell=True, preexec_fn=os.setsid)
    # wait for reset to finish (real time 10 seconds)
    time.sleep(10)
    starttime = rospy.get_time()
    starttime = rospy.get_time() #2x for a good start

    # start platooning nodes
    # MAKE SURE "exec" is in fromt of the command, otherwise it's not killable
    platoons = []
    for namespace in namespaces[1:]:
        cmd_platooning = "exec rosrun platooning platooning_class.py"
        try:
            cmd_platooning += " " + namespace + " " + \
                str(velocity_dict["max_vel_" + namespace[1:]])
        except:
            cmd_platooning += " " + namespace + " 0"

        platoons.append(subprocess.Popen(
            [cmd_platooning], shell=True, preexec_fn=os.setsid))
    # start overtaking node on first
    overtaking = subprocess.Popen(["exec rosrun overtaking overtaking_node.py" + " /first " + str(
        velocity_dict["max_vel_first"])], shell=True, preexec_fn=os.setsid)

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
    rospy.sleep(16)
    print("simulation {} halfway".format(str(ID)))
    rospy.sleep(16)


    # time.sleep(5*60)  # time to simulate
    print("killing everything")
    # publish to killswitch topic (just leave this here)
    pub.publish(Bool(data=True))
    pub.publish(Bool(data=True))

    # kill platoon and decision, both .kill() and os.killpg are neccesary
    a = [platoon.kill() for platoon in platoons]
    a = [os.killpg(os.getpgid(platoon.pid), signal.SIGKILL)
         for platoon in platoons]
    b = [decision.kill() for decision in decisions]
    b = [os.killpg(os.getpgid(decision.pid), signal.SIGKILL)
         for decision in decisions]

    # kill overtaking and recorder
    overtaking.kill()
    recorder.kill()
    os.killpg(os.getpgid(overtaking.pid), signal.SIGKILL)
    os.killpg(os.getpgid(recorder.pid), signal.SIGKILL)

    time.sleep(5)


    velocity_dict["max_vel_first"] += 0.05

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
