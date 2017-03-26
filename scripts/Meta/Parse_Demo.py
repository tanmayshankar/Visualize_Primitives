#!/usr/bin/env python

import os
import subprocess
import sys
import shutil
import time
import signal

# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Desk_Demo_11/"
FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Grid_Demo"

# command = ['rosbag play -r 0.5 {0}/D{1}/GD{1}.bag','rosrun image_view image_view image:=/kinect2/hd/image_color_rect']
command = ['rosbag play -r 0.2 {0}/D{1}/GD{1}.bag','rosrun Visualize_Primitives Parse_Kinect_Video_Sync.py {0}/D{1}/']

p = [[] for i in range(2)]

for i in range(10,11):
# for i in range(1,11):

	print("STARTING TO PROCESS BAG",i)
	for j in range(len(command)):

		p[j] = subprocess.Popen(command[j].format(FILE_DIR,i),shell=True)
		# p = subprocess.Popen(c.format(FILE_DIR,i),shell=True)
	p[0].wait()
	time.sleep(2)
	# os.kill(p[1].pid,signal.SIGKILL)
	# if p[1].poll() is None:
	# 	time.sleep(2)
	# 	os.kill(p[1].pid,signal.SIGKILL)
	time.sleep(5)