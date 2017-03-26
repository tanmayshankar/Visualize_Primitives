#!/usr/bin/env python

import os
import subprocess
import sys
import shutil

def main(argv):

	DIR = str(sys.argv[1])
	demo_index = int(sys.argv[2])
	num_frames = int(sys.argv[3])

	# command = "scp {0}/*.png tanmay@128.2.194.56:~/Code/K2_Demo/Desk_Demo_{1}/".format(DIR,demo_index)
	# subprocess.call(command,shell=True)

	command = "scp tanmay@128.2.194.56:~/Code/K2_Demo/Desk_Demo_{0}/{Interpolated_Depth_*.png,RGB_*.npy,HAND_COORDINATES.npy} Desk_Demo_{0}/".format(demo_index)
	subprocess.call(command.split(),shell=True)


if __name__ == '__main__':
	main(sys.argv)