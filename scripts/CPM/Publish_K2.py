#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point
import message_filters
import os
import numpy as npy
from geometry_msgs.msg import PointStamped

class publish_video_skeleton:

	def __init__(self, FILE_DIR, num_frames):

		self.num_frames = num_frames
		self.FILE_DIR = FILE_DIR
		
		self.rgb_pub = rospy.Publisher("rgb_image",Image,queue_size=5)
		self.depth_pub = rospy.Publisher("depth_image",Image,queue_size=5)
		self.rgb_info_pub = rospy.Publisher("camera_info_rgb",CameraInfo,queue_size=5)
		self.depth_info_pub = rospy.Publisher("camera_info_depth",CameraInfo,queue_size=5)

		self.frame_number_pub = rospy.Publisher("frame_number",PointStamped,queue_size=5)

		self.bridge = CvBridge()

		self.rgb_info = CameraInfo()
		self.rgb_info.header.frame_id = "rgb_optical_frame"
		self.rgb_info.height = 1080
		self.rgb_info.width = 1920
		self.rgb_info.distortion_model = 'plumb_bob'	

		# POWER KINECT PRECALIB
		# self.rgb_info.D = [0.05626844117093032, -0.0741991413086948, 0.0014250797540545752, -0.0016951722389720336, 0.024107681263086548]
		# self.rgb_info.K = [1059.9465578241038, 0.0, 954.8832667758844, 0.0, 1053.9326808799726, 523.7385829106058, 0.0, 0.0, 1.0]
		# self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.rgb_info.P = [1059.9465578241038, 0.0, 954.8832667758844, 0.0, 0.0, 1053.9326808799726, 523.7385829106058, 0.0, 0.0, 0.0, 1.0, 0.0]		

		# # BATTERY KINECT
		# self.rgb_info.D = [ 0.0180894, 0.0196109, 0., 0., -0.0521475]
		# self.rgb_info.K = [ 1060.292, 0., 970.383, 0., 1060.857, 537.568, 0., 0., 1.]
		# self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.rgb_info.P = [ 1060.292, 0., 970.383, 0., 0., 1060.857, 537.568, 0., 0., 0., 1., 0.]

		# # POWER KINECT CALIBRATED:
		# self.rgb_info.D = [ 4.4654283416467225e-02, -5.1842390085880083e-02, -6.3701733442630385e-04, 7.0560217759232122e-04, 8.0238227646430120e-03 ]		
		# self.rgb_info.K = [ 1.0508802349111138e+03, 0., 9.6803703737471221e+02, 0., 1.0492525425755312e+03, 5.3082037618082381e+02, 0., 0., 1. ]		
		# self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.rgb_info.P = [ 1.0508802349111138e+03, 0., 9.6803703737471221e+02, 0., 0., 1.0492525425755312e+03, 5.3082037618082381e+02, 0., 0., 0., 1., 0.]

		# FINAL CALIBRATION:
		self.rgb_info.D = [ 5.1717504939573479e-02, -6.7148951889605527e-02, 4.6117462466262546e-04, -3.0668264612981216e-04, 1.9086522348878591e-02 ]
		self.rgb_info.K = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 1. ]
		self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.rgb_info.P = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 0., 1., 0.]

		self.depth_info = CameraInfo()
		# self.depth_info.header.frame_id = "depth_optical_frame"	
		self.depth_info.header.frame_id = "rgb_optical_frame"	
		self.depth_info.height = 1080
		self.depth_info.width = 1920		
		self.depth_info.distortion_model = 'plumb_bob'
				
		# POWER KINECT PRECALIB
		# self.depth_info.D = [0.08909735083580017, -0.2678897976875305, 0.0, 0.0, 0.09598079323768616]
		# self.depth_info.K = [365.39239501953125, 0.0, 257.818115234375, 0.0, 365.39239501953125, 206.85589599609375, 0.0, 0.0, 1.0]
		# self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.depth_info.P = [365.39239501953125, 0.0, 257.818115234375, 0.0, 0.0, 365.39239501953125, 206.85589599609375, 0.0, 0.0, 0.0, 1.0, 0.0]

		# # BATTERY KINECT
		# self.depth_info.D = [ 0.0890974, -0.2678898, 0., 0., 0.0959808]
		# self.depth_info.K = [365.392, 0., 257.818, 0., 365.392, 206.856, 0., 0., 1.]
		# self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.depth_info.P = [365.392, 0., 257.818, 0., 0., 365.392, 206.856, 0., 0., 0., 1., 0.]

		# # POWER KINECT CALIBRATED:
		# self.depth_info.D = [ 9.8343517599863220e-02, -2.8612696317660713e-01, -7.4890394641482616e-04, -1.3674483058849617e-03, 1.0688453726466322e-01 ]
		# self.depth_info.K = [ 3.5978234166785137e+02, 0., 2.5225559627867887e+02, 0., 3.5954498465510221e+02, 2.0675988572543855e+02, 0., 0., 1. ]
		# self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.depth_info.P = [ 3.5978234166785137e+02, 0., 2.5225559627867887e+02, 0., 0., 3.5954498465510221e+02, 2.0675988572543855e+02, 0., 0., 0., 1., 0.]

		# self.depth_info.D = [ 4.4654283416467225e-02, -5.1842390085880083e-02, -6.3701733442630385e-04, 7.0560217759232122e-04, 8.0238227646430120e-03 ]		
		# self.depth_info.K = [ 1.0508802349111138e+03, 0., 9.6803703737471221e+02, 0., 1.0492525425755312e+03, 5.3082037618082381e+02, 0., 0., 1. ]		
		# self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		# self.depth_info.P = [ 1.0508802349111138e+03, 0., 9.6803703737471221e+02, 0., 0., 1.0492525425755312e+03, 5.3082037618082381e+02, 0., 0., 0., 1., 0.]

		# FINAL CALIBRATION:
		self.depth_info.D = [ 5.1717504939573479e-02, -6.7148951889605527e-02, 4.6117462466262546e-04, -3.0668264612981216e-04, 1.9086522348878591e-02 ]
		self.depth_info.K = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 1. ]
		self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.depth_info.P = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 0., 1., 0.]

		self.rate = rospy.Rate(20)

	def publish_images(self):

		buffer_value = 20

		for j in range(0,self.num_frames+2*buffer_value):

			# if KeyboardInterrupt:
			# 	break
				
			if (j<=buffer_value):
				i=0
			elif (j>=self.num_frames+buffer_value):
				i=self.num_frames-1
			else:
				i=j-buffer_value
			print("BUFFERED",j,i)

			frame_num_pt = PointStamped()
			frame_num_pt.header.stamp = rospy.Time.now()
			frame_num_pt.point.x = i

			# Publish Images:
			print(os.path.join(self.FILE_DIR,"RGB_{0}.png".format(i)))
			img = cv2.imread(os.path.join(self.FILE_DIR,"RGB_{0}.png".format(i)))
			# imgd = cv2.imread(os.path.join(self.FILE_DIR,"Interpolated_Depth_{0}.png".format(i)),-1)					
			imgd = cv2.imread(os.path.join(self.FILE_DIR,"Depth_{0}.png".format(i)),-1)								

			# imgd = cv2.cvtColor(imgd,cv2.COLOR_BGR2GRAY)
			# imgd = npy.uint16(imgd)
		
			img_rgb = self.bridge.cv2_to_imgmsg(img,"bgr8")
			img_rgb.header.frame_id = "rgb_optical_frame"
			img_rgb.header.stamp = rospy.Time.now()
			img_rgb.height = 1080
			img_rgb.width = 1920

			img_depth = self.bridge.cv2_to_imgmsg(imgd,"passthrough")
			# img_depth = self.bridge.cv2_to_imgmsg(imgd,"32FC1")
			# img_depth = self.bridge.cv2_to_imgmsg(imgd,"mono8")
			# img_depth.header.frame_id = "depth_optical_frame"	
			img_depth.header.frame_id = "rgb_optical_frame"
			img_depth.header.stamp = rospy.Time.now()
			img_depth.height = 1080
			img_depth.width = 1920

			self.rgb_info.header.stamp = rospy.Time.now()			
			self.depth_info.header.stamp = rospy.Time.now()

			try:
				self.rgb_pub.publish(img_rgb)
				self.depth_pub.publish(img_depth)
				self.rgb_info_pub.publish(self.rgb_info)
				self.depth_info_pub.publish(self.depth_info)
				self.frame_number_pub.publish(frame_num_pt)
			except CvBridgeError as e:
				print(e)

			self.rate.sleep()

def main(argv):

	rospy.init_node('Publish_Video_Skeleton')

	# FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Demo_9_PNG/"
	FILE_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/K2_Demos/Desk_Demo_13/"

	FILE_DIR = str(sys.argv[1])
	# num_images = 345
	num_images = int(sys.argv[2])
	
	# num_images = 208

	video_skel = publish_video_skeleton(FILE_DIR,num_images)

	try:
		video_skel.publish_images()
	except KeyboardInterrupt:
		print("Shutting Down.")

if __name__ == '__main__':
	main(sys.argv)	



