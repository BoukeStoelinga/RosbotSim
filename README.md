# RosbotSim
Bachelor thesis project "Cooperative automated driving via the Multi ROSbot Lab setup"

Find requirements in requirements.txt

# Instructions
Make sure the repository is withing your catkin workspace, and is sourced.

Launching a simulation (with gui):

```roslaunch rosbot_simulation lane_merging_track_5```

More scenarios available in rosbot_simulation

Start the movement of the rosbots (launch Decision_py):

```roslaunch decision_py decision_5_py.launch```

Launching lane keeping (image processing and lane keeping pid):

```roslaunch ang_vel_pub lane_keeping_5.launch```

Launching Lane switching safety (lidar):

```roslaunch lane_switching_safety lane_switching_safety.launch```

Launching Overtaking (on one rosbot):

```rosrun overtaking overtaking_node.py '{rosbot_name}'```

Launcing Lane merging (on al rosbots):

```roslaunch lane_merging lane_merging_nodes.launch```

