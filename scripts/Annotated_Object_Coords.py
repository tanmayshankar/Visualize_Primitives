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

class point_cloud_class:

	def __init__(self,objects):

		# self.point_cloud_sub = rospy.Subscriber("/depth_registered/points",PointCloud2,self.pc_callback)	
		self.pointcloud_sub = message_filters.Subscriber("/depth_registered/points",PointCloud2)
		self.rgb_sub = message_filters.Subscriber("/rgb_image",Image)

		self.sequence_sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub,self.pointcloud_sub],20,1,allow_headerless=False)
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

		print(self.objects.shape)
		for i in range(self.objects.shape[0]):
			self.object_markers.markers.append(copy.deepcopy(self.obj_marker))
	
		for i in range(self.objects.shape[0]):
			self.object_markers.markers[i].ns = "Object_{0}".format(i+1)
			self.object_markers.markers[i].id = i+1
		
		for i in range(self.objects.shape[0]):
			print(self.object_markers.markers[i].ns,self.object_markers.markers[i].id)

		# print(self.object_markers)
	def rgb_pc_callback(self, rgb_data, pc_data):

		index = rgb_data.header.seq

		# uv = npy.zeros((self.objects.shape[0],2))
		uv = [[] for i in range(self.objects.shape[0])]

		for i in range(self.objects.shape[0]):
			uv[i] = [int((self.objects[i,index,0]+self.objects[i,index,2])/2),int((self.objects[i,index,1]+self.objects[i,index,3])/2)]

		counter = 0
		
		for points in sensor_msgs.point_cloud2.read_points(pc_data,skip_nans=False,uvs=uv):
		# points = sensor_msgs.point_cloud2.read_points(pc_data,skip_nans=False,uvs=uv)
			# print(counter,points)
			# print(self.object_markers)
			# print("Hallelujah")

			self.object_markers.markers[counter].pose.position.x = points[0]
			self.object_markers.markers[counter].pose.position.y = points[1]
			self.object_markers.markers[counter].pose.position.z = points[2]

			counter += 1

		self.object_pub.publish(self.object_markers)

def main(argv):
	
	FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_annotations/"

	objects = npy.load(os.path.join(FILE_DIR,"Traj_{0}_Objects.npy".format(int(sys.argv[1]))))

	rospy.init_node('PointCloud_Objects')
	pc_class = point_cloud_class(objects)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down.")
		
if __name__ == '__main__':
	main(sys.argv)