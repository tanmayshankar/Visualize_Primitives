#!/usr/bin/env python

import numpy as npy
import os
import shutil
import subprocess
import sys
import signal
import time

number_trajectories = 31

# for i in range(0,number_trajectories):	
# 	print("The Counter is ",i)
# 	command = "roslaunch Visualize_Primitives visualize_video_skeleton.launch video_index:={0}".format(i)
# 	subprocess.call(command.split(),shell=False)	

FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/"
number_frames = npy.load(os.path.join(FILE_DIR,"Number_Images_Sorted.npy")).astype(int)


for i in range(0,number_trajectories):
	num_frames = number_frames[i]+40+90
	print("Running on Trajectory: ",i)
	command = "roslaunch Visualize_Primitives visualize_video_skeleton.launch video_index:={0}".format(i)
	pro = subprocess.Popen(command,stdout=subprocess.PIPE,shell=True,preexec_fn=os.setsid)
	time.sleep(num_frames/30)
	os.killpg(os.getpgid(pro.pid),signal.SIGTERM)
