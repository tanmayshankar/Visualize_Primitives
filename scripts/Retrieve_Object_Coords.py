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

class detections():

    def __init__(self,image_path):

        self.image_path = image_path
        self.bbox = []
        self.score = []
        self.class_ind = []
        self.class_name = []

class point_cloud_class:

	def __init__(self,objects):

		self.point_cloud_sub = rospy.Subscriber("/depth_registered/points",PointCloud2,self.pc_callback)	
		self.objects = objects
		# self.pointcloud = PointCloud2()
		self.points = []

	def pc_callback(self,pcdata):

		uv = [[] for i in range(len(self.objects.bbox))]
		
		for i in range(len(self.objects.bbox)):
			uv[i] = [int((self.objects.bbox[i][0]+self.objects.bbox[i][2])/2),int((self.objects.bbox[i][1]+self.objects.bbox[i][3])/2)]

		print(uv)
		counter = 0
		# for points in sensor_msgs.point_cloud2.read_points(pcdata,skip_nans=True,uvs=uv):
		for points in sensor_msgs.point_cloud2.read_points(pcdata,skip_nans=False,uvs=uv):
			self.points.append([points[0],points[1],points[2]])

			print("POINT INDEX: ", counter)
			print([points[0],points[1],points[2]])
			counter +=1

def main(argv):
	
	FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/"

	directory = "microwaving_food/1204150645/"
	file = "RGB_1.npy"

	objects = npy.load(os.path.join(FILE_DIR,directory,file)).item()

	rospy.init_node('PointCloud_Objects')
	pc_class = point_cloud_class(objects)

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down.")

if __name__ == '__main__':
	main(sys.argv)