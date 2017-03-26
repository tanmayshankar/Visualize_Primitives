#!/usr/bin/env python
import os
import subprocess
import sys
import shutil
import time
import signal
import numpy as npy

# FRCNN_DIR = "/home/tanmay/Code/py-faster-rcnn/tools"
# CPM_DIR = "/home/tanmay/Code/Realtime_Multi-Person_Pose_Estimation/testing/python"
# INTERP_DIR = "/home/tanmay/Code/Meta_Scripts"
IMG_DIR = "/home/tanmay/Code/Grid_Demo/Grid_Demo/"
LOC_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Grid_Demo"

command = "scp tanmay@128.2.194.56:~/Code/Grid_Demo/Grid_Demo/D{0}/HAND_COORDINATES.npy D{0}/"

for i in range(1,11):

	p = subprocess.Popen(command.format(i),shell=True)
	p.wait()
	time.sleep(2)
