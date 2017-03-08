#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as npy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import os
import copy
from geometry_msgs.msg import PointStamped
import time

class point_cloud_class:

	def __init__(self,objects,num_frames,video_ind):

		# self.point_cloud_sub = rospy.Subscriber("/depth_registered/points",PointCloud2,self.pc_callback)	
		self.pointcloud_sub = message_filters.Subscriber("/depth_registered/points",PointCloud2)
		self.rgb_sub = message_filters.Subscriber("/rgb_image",Image)
		self.frame_number_sub = message_filters.Subscriber("/frame_number",PointStamped)

		self.sequence_sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub,self.pointcloud_sub,self.frame_number_sub],20,1,allow_headerless=False)
		self.sequence_sync.registerCallback(self.rgb_pc_callback)

		self.object_pub = rospy.Publisher("object_coordinates",MarkerArray,queue_size=10)

		self.objects = objects

		self.object_markers = MarkerArray()
		self.obj_marker = Marker()
		self.obj_marker.header.frame_id = "rgb_optical_frame"
		self.obj_marker.ns = "Object"
		self.obj_marker.id = 1
		self.obj_marker.action = Marker.ADD
		self.obj_marker.type = Marker.CUBE
		self.obj_marker.scale.x = 0.001
		self.obj_marker.scale.y = 0.001
		self.obj_marker.scale.z = 0.001
		self.obj_marker.color.b = 1.0
		self.obj_marker.color.a = 1.0
		self.obj_marker.pose.orientation.x = 0.0;
		self.obj_marker.pose.orientation.y = 0.0;
		self.obj_marker.pose.orientation.z = 0.0;
		self.obj_marker.pose.orientation.w = 1.0;

		# print(self.objects.shape)
		for i in range(self.objects.shape[0]):
			self.object_markers.markers.append(copy.deepcopy(self.obj_marker))
	
		for i in range(self.objects.shape[0]):
			self.object_markers.markers[i].ns = "Object_{0}".format(i+1)
			self.object_markers.markers[i].id = i+1
		
		# for i in range(self.objects.shape[0]):
		# 	print(self.object_markers.markers[i].ns,self.object_markers.markers[i].id)

		# print(self.object_markers)
		self.video_ind = video_ind
		self.object_3D_coordinates = npy.zeros((self.objects.shape[0],num_frames,3))
		self.num_frames = num_frames
		print(self.num_frames,self.video_ind)

	def rgb_pc_callback(self, rgb_data, pc_data, frame_num_pt):

		index = rgb_data.header.seq
		frame_number = int(frame_num_pt.point.x)

		if (frame_number==self.num_frames):
		# if (frame_number==23):	
			print(self.object_3D_coordinates)
			npy.save("Trajectory_Object_Coordinates_{0}.npy".format(self.video_ind),self.object_3D_coordinates)
			# with open("Traj_Obj_Coord.npy",'w') as outfile:
				# npy.save(outfile,self.object_3D_coordinates)
			# time.sleep(0.3)
			# rospy.signal_shutdown("Done, Haha")

		print(index,frame_number)

		uv = [[] for i in range(self.objects.shape[0])]

		for i in range(self.objects.shape[0]):
			# uv[i] = [int((self.objects[i,index,0]+self.objects[i,index,2])/2),int((self.objects[i,index,1]+self.objects[i,index,3])/2)]
			uv[i] = [int((self.objects[i,frame_number-1,0]+self.objects[i,frame_number-1,2])/2),int((self.objects[i,frame_number-1,1]+self.objects[i,frame_number-1,3])/2)]

		counter = 0
		
		for points in sensor_msgs.point_cloud2.read_points(pc_data,skip_nans=False,uvs=uv):

			self.object_markers.markers[counter].pose.position.x = points[0]
			self.object_markers.markers[counter].pose.position.y = points[1]
			self.object_markers.markers[counter].pose.position.z = points[2]			
			self.object_3D_coordinates[counter,frame_number-1] = points[0:3]

			counter += 1

		self.object_pub.publish(self.object_markers)


def main(argv):
	
	FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_annotations/"

	objects = npy.load(os.path.join(FILE_DIR,"Traj_{0}_Objects.npy".format(int(sys.argv[1]))))

	rospy.init_node('PointCloud_Objects',disable_signals=True)


	


	FILE_DIR_2 = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/"
	number_frames = npy.load(os.path.join(FILE_DIR_2,"Number_Images_Sorted.npy")).astype(int)
	
	video_ind = int(sys.argv[1])
	num_frames = number_frames[video_ind]

	pc_class = point_cloud_class(objects,num_frames,video_ind)

	try:
		rospy.spin()
	except KeyboardInterrupt:

		print("Shutting Down.")
		
if __name__ == '__main__':
	main(sys.argv)
