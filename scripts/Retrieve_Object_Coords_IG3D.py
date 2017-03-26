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
import image_geometry as ig
from geometry_msgs.msg import PointStamped
import os

class detections():

    def __init__(self,image_path):

        self.image_path = image_path
        self.bbox = []
        self.score = []
        self.class_ind = []
        self.class_name = []

class object_class:

	def __init__(self,num_frames, FILE_DIR):

		self.num_frames = num_frames
		self.caminfo_sub = message_filters.Subscriber("/camera_info_rgb",CameraInfo)
		self.frame_number_sub = message_filters.Subscriber("/frame_number",PointStamped)
		self.depth_image_sub = message_filters.Subscriber("/depth_image",Image)

		self.sequence_sync = message_filters.ApproximateTimeSynchronizer([self.caminfo_sub,self.frame_number_sub,self.depth_image_sub],20,1,allow_headerless=False)
		self.sequence_sync.registerCallback(self.callback)
		self.FILE_DIR = FILE_DIR
		self.img_geo = ig.PinholeCameraModel()

		# self.object_coords = npy.zeros((2,self.num_frames,3))
		self.object_coords = npy.zeros((self.num_frames,2,3))
		self.bridge = CvBridge()

	def callback(self, caminfo, frame_num_pt, depth_image):

		frame_number = int(frame_num_pt.point.x)
		print(frame_number)

		if (frame_number==self.num_frames-1):
			npy.save(os.path.join(self.FILE_DIR,"Obj_Coordinates_3D.npy"),self.object_coords)
		
		try:
			d_img = self.bridge.imgmsg_to_cv2(depth_image,"passthrough")		
		except CvBridgeError as e:
			print(e)

		uv = [[] for i in range(2)]

		self.img_geo.fromCameraInfo(caminfo)
		obj_2d = npy.load(os.path.join(self.FILE_DIR,"RGB_{0}.npy".format(frame_number))).item()

		for j in range(len(obj_2d.class_name)):	
			# if obj_2d.class_name[j]=='book':
			if obj_2d.class_name[j]=='bottle':
				# uv[0] = [int((obj_2d.bbox[j][0]+obj_2d.bbox[j][2])/2),int((obj_2d.bbox[j][1]+obj_2d.bbox[j][3])/2)]
				uv[0] = [int((obj_2d.bbox[j][1]+obj_2d.bbox[j][3])/2),int((obj_2d.bbox[j][0]+obj_2d.bbox[j][2])/2)]
			
			if obj_2d.class_name[j]=='cup':
				# uv[1] = [int((obj_2d.bbox[j][0]+obj_2d.bbox[j][2])/2),int((obj_2d.bbox[j][1]+obj_2d.bbox[j][3])/2)]
				uv[1] = [int((obj_2d.bbox[j][1]+obj_2d.bbox[j][3])/2),int((obj_2d.bbox[j][0]+obj_2d.bbox[j][2])/2)]

		for i in range(2):	
			if (uv[i]):
				print(uv[i])
				# self.object_coords[i,frame_number] = self.img_geo.projectPixelTo3dRay((uv[i][0],uv[i][1]))
				self.object_coords[frame_number,i] = self.img_geo.projectPixelTo3dRay((uv[i][0],uv[i][1]))
				actual_depth = float(d_img[uv[i][0],uv[i][1]])/1000
				# self.object_coords[i,frame_number] *= (actual_depth/self.object_coords[i,frame_number,2])
				self.object_coords[frame_number,i] *= (actual_depth/self.object_coords[frame_number,i,2])

def main(argv):
	
	# num_frames = 345
	num_frames = int(sys.argv[2])
	FILE_DIR = str(sys.argv[1])
	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Demo_9_PNG/"
	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Desk_Demo_13/"

	rospy.init_node('Obj_Coord_3D')
	objclass = object_class(num_frames,FILE_DIR)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down.")
		
if __name__ == '__main__':
	main(sys.argv)
