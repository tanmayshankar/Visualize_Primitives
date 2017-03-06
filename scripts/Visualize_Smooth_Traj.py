#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import numpy as npy
import copy

def publish_trajectory(traj,traj_shoulder):

	traj_pub = rospy.Publisher("trajectory_markers",Marker,queue_size=10)
	traj_pub_shoulder = rospy.Publisher("shoulder_trajectory_markers",Marker,queue_size=10)
	rate = rospy.Rate(20)
	traj_marker = Marker()
	shoulder_marker = Marker()

	# traj_marker.header.frame_id = "rgb_optical_frame"
	traj_marker.header.frame_id = "traj_frame"
	traj_marker.ns = "Trajectory_Line_Strip"
	traj_marker.id = 1
	traj_marker.action = Marker.ADD
	traj_marker.type = Marker.LINE_STRIP
	traj_marker.scale.x = 0.0001
	traj_marker.scale.y = 0.0001
	traj_marker.scale.z = 0.0001
	traj_marker.color.g = 1.0
	traj_marker.color.b = 0.4
	traj_marker.color.a = 1.0

	shoulder_marker.header.frame_id = "traj_frame"
	shoulder_marker.ns = "Trajectory_Shoulder"
	shoulder_marker.id = 1
	shoulder_marker.action = Marker.ADD
	shoulder_marker.type = Marker.LINE_STRIP
	shoulder_marker.scale.x = 0.0001
	shoulder_marker.scale.y = 0.0001
	shoulder_marker.scale.z = 0.0001
	shoulder_marker.color.r = 1.0
	shoulder_marker.color.b = 0.4
	shoulder_marker.color.a = 1.0

	p = Point()
	for i in range(len(traj)):
		p.x = traj[i,0]
		p.y = traj[i,1]
		p.z = traj[i,2]

		traj_marker.points.append(copy.deepcopy(p))

	for i in range(len(traj_shoulder)):
		p.x = traj_shoulder[i,0]
		p.y = traj_shoulder[i,1]
		p.z = traj_shoulder[i,2]

		shoulder_marker.points.append(copy.deepcopy(p))

	while not rospy.is_shutdown():
		traj_marker.header.stamp = rospy.Time.now()
		traj_pub.publish(traj_marker)
		traj_pub_shoulder.publish(shoulder_marker)
		rate.sleep()

def main(argv):
	rospy.init_node("Trajectory_Visualize")

	# FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/"
	TRAJ_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/Smooth_Trajectories/"

	# sorting_indices = npy.load(os.path.join(FILE_DIR,"Sorting_Indices.npy"))
	
	# # traj_ind = sorting_indices[int(sys.argv[1])]
	traj_ind = int(sys.argv[1])

	path = os.path.join(TRAJ_DIR,"LH_Smooth.npy")
	path_sh = os.path.join(TRAJ_DIR,"LH_Shoulder_Smooth.npy")
	# path_sh = os.path.join(TRAJ_DIR,"Traj_{0}".format(traj_ind),"Original_Left_Shoulder_{0}.npy".format(traj_ind))

	traj = npy.load(path)
	traj_shoulder = npy.load(path_sh)

	try:
		# publish_trajectory(traj/100000,traj_shoulder/100000)
		publish_trajectory(traj[traj_ind],traj_shoulder[traj_ind])
	except rospy.ROSInterruptException:
		pass

if __name__ =='__main__':
	main(sys.argv)