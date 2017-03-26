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
import os
import copy
from geometry_msgs.msg import PointStamped

import image_geometry as ig

class point_cloud_class:

	def __init__(self,num_frames, hand_coords, FILE_DIR):

		self.FILE_DIR = FILE_DIR
		self.caminfo_sub = message_filters.Subscriber("/camera_info_rgb",CameraInfo)
		self.frame_number_sub = message_filters.Subscriber("/frame_number",PointStamped)
		self.depth_image_sub = message_filters.Subscriber("/depth_image",Image)
		self.sequence_sync = message_filters.ApproximateTimeSynchronizer([self.caminfo_sub,self.frame_number_sub,self.depth_image_sub],20,1,allow_headerless=False)
		self.sequence_sync.registerCallback(self.callback)

		self.num_frames = num_frames
		self.hand2d = hand_coords
		self.hand_coords = npy.zeros((self.num_frames,4,3))
	
		self.pixel_width = 0
		self.size_var = 2*self.pixel_width+1
		self.img_geo = ig.PinholeCameraModel()
		self.bridge = CvBridge()

	def callback(self, caminfo, frame_num_pt, depth_image):

		frame_number = int(frame_num_pt.point.x)
		print("Wasting time on: ", frame_number)

		if (frame_number==self.num_frames-1):
			npy.save(os.path.join(self.FILE_DIR,"Hand_Coordinates_3D.npy"),self.hand_coords)
			
		uv = [[] for i in range(self.hand_coords.shape[1])]

		try:
			d_img = self.bridge.imgmsg_to_cv2(depth_image,"passthrough")		
		except CvBridgeError as e:
			print(e)

		self.img_geo.fromCameraInfo(caminfo)
		pt_arr = npy.zeros((4,3))

		print(d_img.shape)

		for i in range(self.hand_coords.shape[1]):
			uv[i] = [self.hand2d[frame_number,i,0],self.hand2d[frame_number,i,1]]	

		for k in range(self.hand_coords.shape[1]):
			denominator = 0
			for i in range(-self.pixel_width,self.pixel_width+1):
				for j in range(-self.pixel_width,self.pixel_width+1):
					uv_aug = [uv[k][0]+i,uv[k][1]+j]
					# print(uv_aug)					
					points = self.img_geo.projectPixelTo3dRay((uv_aug[0],uv_aug[1]))									


					actual_depth = float(d_img[uv_aug[0],uv_aug[1]])/1000
					points = npy.array(points)
					points *= (actual_depth/points[2])

					denominator += 1
							
					pt_arr[k,0] += points[0]
					pt_arr[k,1] += points[1]
					pt_arr[k,2] += points[2]

			self.hand_coords[frame_number,k] = pt_arr[k]/denominator				

def main(argv):
	
	# FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_annotations/"
	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/scripts/K2_Demo/"
	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Demo_9_PNG/"
	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Desk_Demo_13/"
	FILE_DIR = str(sys.argv[1])

	# hand_coords = npy.load(os.path.join(FILE_DIR,"HC2_DD9.npy")).astype(int)
	hand_coords = npy.load(os.path.join(FILE_DIR,"HAND_COORDINATES.npy")).astype(int)

	# number_frames = 208
	# number_frames = 130	
	# number_frames = 345
	number_frames = int(sys.argv[2])
	print(hand_coords)

	rospy.init_node('PointCloud_Objects',disable_signals=True)
	pc_class = point_cloud_class(number_frames, hand_coords, FILE_DIR)

	try:
		rospy.spin()
	except KeyboardInterrupt:

		print("Shutting Down.")
		
if __name__ == '__main__':
	main(sys.argv)


