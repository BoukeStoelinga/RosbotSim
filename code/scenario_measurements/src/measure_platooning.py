#!/usr/bin/env python

import os
import signal
import subprocess
import time

N = 2 #Number of simulations
T = 10 #Time to simulate

total = N*T + N*41
hrs = int(total/3600)
mins = int((total%3600)/60)
secs = int(total%60)

print("\033[1m \033[94m Estimated runtime: {:2d}:{:2d}:{:2d}\033[0m".format(hrs, mins, secs))
time.sleep(3)

for ID in range(N):
    gazebo = subprocess.Popen(["roslaunch scenario_measurements platooning.launch runID:="+str(ID)], shell = True, preexec_fn=os.setsid)
    time.sleep(21) #wait for sim to start up
    time.sleep(T) #simulate and record data
    os.killpg(os.getpgid(gazebo.pid), signal.SIGTERM)  # Send the signal to all the process groups
    time.sleep(20) #wait for gazebo to stop whining
    gazebo.kill() #kill gazebo subprocess
