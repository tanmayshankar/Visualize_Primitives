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

def publish_trajectory(traj):

	traj_pub = rospy.Publisher("trajectory_markers",Marker,queue_size=10)
	rate = rospy.Rate(10)
	traj_marker = Marker()
	# traj_marker.header.frame_id = "rgb_optical_frame"
	traj_marker.header.frame_id = "traj_frame"
	traj_marker.ns = "Trajectory_Line_Strip"
	traj_marker.id = 1
	traj_marker.action = Marker.ADD
	traj_marker.type = Marker.LINE_STRIP
	traj_marker.scale.x = 0.0001
	traj_marker.scale.y = 0.0001
	traj_marker.scale.z = 0.0001
	traj_marker.color.b = 1.0
	traj_marker.color.a = 1.0

	p = Point()
	for i in range(len(traj)):
		p.x = traj[i,0]
		p.y = traj[i,1]
		p.z = traj[i,2]
		# print(traj_marker.points)
		# print("Printing a New Point", p)
		# traj_marker.points.append([traj[i,0],traj[i,1],traj[i,2]])
		traj_marker.points.append(copy.deepcopy(p))
		# print(traj_marker.points)
		# traj_marker.points.push_back(p)
	# print("PRINTING THE MARKER: ", traj_marker)
	while not rospy.is_shutdown():
		traj_marker.header.stamp = rospy.Time.now()
		traj_pub.publish(traj_marker)
		rate.sleep()

def main(argv):
	rospy.init_node("Trajectory_Visualize")

	TRAJ_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Primitive_Library/Subject1"

	traj_ind = 8
	path = os.path.join(TRAJ_DIR,"Traj_{0}".format(traj_ind),"Original_Left_Hand_{0}.npy".format(traj_ind))
	# path = os.path.join(TRAJ_DIR,"Traj_{0}".format(traj_ind),"Original_Right_Hand_{0}.npy".format(traj_ind))
	print(path)
	traj = npy.load(path)

	try:
		publish_trajectory(traj/100000)
	except rospy.ROSInterruptException:
		pass

if __name__ =='__main__':
	main(sys.argv)