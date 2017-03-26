#!/usr/bin/env python
import os
import subprocess
import sys
import shutil
import time
import signal
import numpy as npy

LOC_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Grid_Demo"

num_images = npy.load(os.path.join(LOC_DIR,"Number_Images.npy"))

# command = "scp tanmay@128.2.194.56:~/Code/Grid_Demo/Grid_Demo/D{0}/Interpolated_Depth_*.png D{0}/"
command = ['rosrun Visualize_Primitives Publish_K2.py {0}/D{1}/ {2}','rosrun Visualize_Primitives Retrieve_IG3D_Window.py {0}/D{1}/ {2}','rosrun Visualize_Primitives Retrieve_Object_Coords_IG3D.py {0}/D{1}/ {2}']

p = [[] for i in range(3)]

for i in range(1,11):

	print("STARTING TO PROCESS:",i)

	for j in range(len(command)):
		p[j] = subprocess.Popen(command[j].format(LOC_DIR,i,num_images[i-1]),shell=True)

	p[0].wait()
	
	os.kill(p[1].pid,signal.SIGKILL)
	os.kill(p[2].pid,signal.SIGKILL)
	time.sleep(2)
	os.kill(p[1].pid,signal.SIGKILL)
	os.kill(p[2].pid,signal.SIGKILL)
	# os.kill(p[1].pid,signal.SIGKILL)
	# if p[1].poll() is None:
	# 	time.sleep(2)
	# 	os.kill(p[1].pid,signal.SIGKILL)
	time.sleep(5)
