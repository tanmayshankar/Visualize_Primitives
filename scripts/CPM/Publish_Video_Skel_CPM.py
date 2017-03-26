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

	def __init__(self, FILE_DIR, num_frames, lh_traj, rh_traj):

		self.num_frames = num_frames
		self.FILE_DIR = FILE_DIR
		
		self.rgb_pub = rospy.Publisher("rgb_image",Image,queue_size=5)
		self.depth_pub = rospy.Publisher("depth_image",Image,queue_size=5)
		self.rgb_info_pub = rospy.Publisher("camera_info_rgb",CameraInfo,queue_size=5)
		self.depth_info_pub = rospy.Publisher("camera_info_depth",CameraInfo,queue_size=5)

		self.frame_number_pub = rospy.Publisher("frame_number",PointStamped,queue_size=5)

		self.lh_pub = rospy.Publisher("lh_pose", Marker, queue_size=5)
		self.rh_pub = rospy.Publisher("rh_pose", Marker, queue_size=5)

		self.lh_marker = Marker()
		self.lh_marker.header.frame_id = "traj_frame"
		self.lh_marker.ns = "LH_Line_Strip"
		self.lh_marker.id = 0
		self.lh_marker.action = Marker.ADD
		# self.lh_marker.type = Marker.CUBE
		self.lh_marker.type = Marker.SPHERE
		self.lh_marker.scale.x = 0.1
		self.lh_marker.scale.y = 0.1
		self.lh_marker.scale.z = 0.1
		self.lh_marker.color.g = 1.0
		self.lh_marker.color.a = 1.0
		self.lh_marker.pose.orientation.x = 0.0;
		self.lh_marker.pose.orientation.y = 0.0;
		self.lh_marker.pose.orientation.z = 0.0;
		self.lh_marker.pose.orientation.w = 1.0;

		self.rh_marker = Marker()
		self.rh_marker.header.frame_id = "traj_frame"
		self.rh_marker.ns = "RH_Line_Strip"
		self.rh_marker.id = 1
		self.rh_marker.action = Marker.ADD
		# self.rh_marker.type = Marker.CUBE
		self.rh_marker.type = Marker.SPHERE
		self.rh_marker.scale.x = 0.1
		self.rh_marker.scale.y = 0.1
		self.rh_marker.scale.z = 0.1
		self.rh_marker.color.r = 1.0
		self.rh_marker.color.a = 1.0
		self.rh_marker.pose.orientation.x = 0.0;
		self.rh_marker.pose.orientation.y = 0.0;
		self.rh_marker.pose.orientation.z = 0.0;
		self.rh_marker.pose.orientation.w = 1.0;

		self.lh_traj = lh_traj
		self.rh_traj = rh_traj

		self.bridge = CvBridge()

		self.rgb_info = CameraInfo()
		self.rgb_info.header.frame_id = "rgb_optical_frame"
		self.rgb_info.height = 1080
		self.rgb_info.width = 1920
		self.rgb_info.distortion_model = 'plumb_bob'	

		# FINAL CALIBRATION:
		self.rgb_info.D = [ 5.1717504939573479e-02, -6.7148951889605527e-02, 4.6117462466262546e-04, -3.0668264612981216e-04, 1.9086522348878591e-02 ]
		self.rgb_info.K = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 1. ]
		self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.rgb_info.P = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 0., 1., 0.]

		self.depth_info = CameraInfo()	
		self.depth_info.header.frame_id = "rgb_optical_frame"	
		self.depth_info.height = 1080
		self.depth_info.width = 1920		
		self.depth_info.distortion_model = 'plumb_bob'
				
		# FINAL CALIBRATION:
		self.depth_info.D = [ 5.1717504939573479e-02, -6.7148951889605527e-02, 4.6117462466262546e-04, -3.0668264612981216e-04, 1.9086522348878591e-02 ]
		self.depth_info.K = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 1. ]
		self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.depth_info.P = [ 1.0489507963238227e+03, 0., 9.6300666950576135e+02, 0., 0., 1.0494992411576166e+03, 5.3613714771462526e+02, 0., 0., 0., 1., 0.]

		self.rate = rospy.Rate(20)

	def publish_images(self):

		buffer_value = 20

		for j in range(0,self.num_frames+2*buffer_value):

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
			imgd = cv2.imread(os.path.join(self.FILE_DIR,"Depth_{0}.png".format(i)),-1)								
		
			img_rgb = self.bridge.cv2_to_imgmsg(img,"bgr8")
			img_rgb.header.frame_id = "rgb_optical_frame"
			img_rgb.header.stamp = rospy.Time.now()
			img_rgb.height = 1080
			img_rgb.width = 1920

			img_depth = self.bridge.cv2_to_imgmsg(imgd,"passthrough")
			img_depth.header.frame_id = "rgb_optical_frame"
			img_depth.header.stamp = rospy.Time.now()
			img_depth.height = 1080
			img_depth.width = 1920

			self.rgb_info.header.stamp = rospy.Time.now()			
			self.depth_info.header.stamp = rospy.Time.now()

			self.lh_marker.pose.position.x = self.lh_traj[i-1,1]
			self.lh_marker.pose.position.y = self.lh_traj[i-1,0]
			self.lh_marker.pose.position.z = self.lh_traj[i-1,2]

			self.rh_marker.pose.position.x = self.rh_traj[i-1,1]
			self.rh_marker.pose.position.y = self.rh_traj[i-1,0]
			self.rh_marker.pose.position.z = self.rh_traj[i-1,2]

			try:
				self.rgb_pub.publish(img_rgb)
				self.depth_pub.publish(img_depth)
				self.rgb_info_pub.publish(self.rgb_info)
				self.depth_info_pub.publish(self.depth_info)
				self.frame_number_pub.publish(frame_num_pt)

				self.lh_pub.publish(self.lh_marker)
				self.rh_pub.publish(self.rh_marker)

			except CvBridgeError as e:
				print(e)

			self.rate.sleep()

def main(argv):

	rospy.init_node('Publish_Video_Skeleton')

	FILE_DIR = str(sys.argv[1])
	num_images = int(sys.argv[2])
	
	hc3 = npy.load(os.path.join(FILE_DIR,"Hand_Coordinates_3D.npy"))
	lh = hc3[:,0,:]
	rh = hc3[:,2,:]

	video_skel = publish_video_skeleton(FILE_DIR,num_images,lh,rh)

	try:
		video_skel.publish_images()
	except KeyboardInterrupt:
		print("Shutting Down.")

if __name__ == '__main__':
	main(sys.argv)	



