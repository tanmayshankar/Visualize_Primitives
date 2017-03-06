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

	def __init__(self, FILE_DIR, num_frames,left_hand,left_shoulder):

		self.num_frames = num_frames
		self.FILE_DIR = FILE_DIR
		self.left_hand_traj = left_hand
		self.left_shoulder_traj = left_shoulder

		self.rgb_pub = rospy.Publisher("rgb_image",Image,queue_size=10)
		self.depth_pub = rospy.Publisher("depth_image",Image,queue_size=10)
		self.rgb_info_pub = rospy.Publisher("camera_info_rgb",CameraInfo,queue_size=10)
		self.depth_info_pub = rospy.Publisher("camera_info_depth",CameraInfo,queue_size=10)

		self.lh_hand_pub = rospy.Publisher("left_hand_pose",Marker,queue_size=10)
		self.lh_shoulder_pub = rospy.Publisher("left_hand_shoulder_pose",Marker,queue_size=10)

		self.frame_number_pub = rospy.Publisher("frame_number",PointStamped,queue_size=10)

		self.bridge = CvBridge()

		self.rgb_info = CameraInfo()
		self.rgb_info.header.frame_id = "rgb_optical_frame"
		self.rgb_info.height = 480
		self.rgb_info.width = 640
		self.rgb_info.distortion_model = 'plumb_bob'
		self.rgb_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
		self.rgb_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
		self.rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.rgb_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]

		self.depth_info = CameraInfo()
		self.depth_info.header.frame_id = "depth_optical_frame"
		self.depth_info.height = 480
		self.depth_info.width = 640
		self.depth_info.distortion_model = 'plumb_bob'
		self.depth_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
		self.depth_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
		self.depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
		self.depth_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]

		self.rate = rospy.Rate(20)

		self.left_hand_marker = Marker()
		self.left_hand_marker.header.frame_id = "traj_frame"
		self.left_hand_marker.ns = "LH_Line_Strip"
		self.left_hand_marker.id = 0
		self.left_hand_marker.action = Marker.ADD
		self.left_hand_marker.type = Marker.CUBE
		self.left_hand_marker.scale.x = 0.001
		self.left_hand_marker.scale.y = 0.001
		self.left_hand_marker.scale.z = 0.001
		self.left_hand_marker.color.g = 1.0
		self.left_hand_marker.color.a = 1.0
		self.left_hand_marker.pose.orientation.x = 0.0;
		self.left_hand_marker.pose.orientation.y = 0.0;
		self.left_hand_marker.pose.orientation.z = 0.0;
		self.left_hand_marker.pose.orientation.w = 1.0;

		self.left_shoulder_marker = Marker()
		self.left_shoulder_marker.header.frame_id = "traj_frame"
		self.left_shoulder_marker.ns = "LS_Line_Strip"
		self.left_shoulder_marker.id = 1
		self.left_shoulder_marker.action = Marker.ADD
		self.left_shoulder_marker.type = Marker.CUBE
		self.left_shoulder_marker.scale.x = 0.001
		self.left_shoulder_marker.scale.y = 0.001
		self.left_shoulder_marker.scale.z = 0.001
		self.left_shoulder_marker.color.r = 1.0
		self.left_shoulder_marker.color.a = 1.0
		self.left_shoulder_marker.pose.orientation.x = 0.0;
		self.left_shoulder_marker.pose.orientation.y = 0.0;
		self.left_shoulder_marker.pose.orientation.z = 0.0;
		self.left_shoulder_marker.pose.orientation.w = 1.0;

	def publish_images(self):

		buffer_value = 20

		for j in range(1,self.num_frames+2*buffer_value+1):

			if (j<=buffer_value):
				i=1
			elif (j>=self.num_frames+buffer_value):
				i=self.num_frames
			else:
				i=j-buffer_value+1
			# print("BUFFERED",j,i)

			frame_num_pt = PointStamped()
			frame_num_pt.header.stamp = rospy.Time.now()
			frame_num_pt.point.x = i

		# for i in range(1,self.num_frames+1):

			# Publish Images:
			img = cv2.imread(os.path.join(self.FILE_DIR,"RGB_{0}.png".format(i)))
			imgd = cv2.imread(os.path.join(self.FILE_DIR,"Depth_{0}.png".format(i)))					

			imgd = cv2.cvtColor(imgd,cv2.COLOR_BGR2GRAY)
			imgd = npy.uint16(imgd)
			
			img_rgb = self.bridge.cv2_to_imgmsg(img,"bgr8")
			img_rgb.header.frame_id = "rgb_optical_frame"
			img_rgb.header.seq = i

			img_rgb.header.stamp = rospy.Time.now()
			img_rgb.height = 480
			img_rgb.width = 640

			img_depth = self.bridge.cv2_to_imgmsg(imgd,"passthrough")
			img_depth.header.frame_id = "depth_optical_frame"
			img_depth.header.seq = i
			img_depth.header.stamp = rospy.Time.now()
			img_depth.height = 480
			img_depth.width = 640

			self.rgb_info.header.seq = i
			self.rgb_info.header.stamp = rospy.Time.now()

			self.depth_info.header.seq = i
			self.depth_info.header.stamp = rospy.Time.now()

			# Skeleton Pose Information:
			self.left_hand_marker.pose.position.x = self.left_hand_traj[i-1,0]
			self.left_hand_marker.pose.position.y = self.left_hand_traj[i-1,1]
			self.left_hand_marker.pose.position.z = self.left_hand_traj[i-1,2]

			# Skeleton Pose Information:
			self.left_shoulder_marker.pose.position.x = self.left_shoulder_traj[i-1,0]
			self.left_shoulder_marker.pose.position.y = self.left_shoulder_traj[i-1,1]
			self.left_shoulder_marker.pose.position.z = self.left_shoulder_traj[i-1,2]

			try:
				self.rgb_pub.publish(img_rgb)
				self.depth_pub.publish(img_depth)
				self.rgb_info_pub.publish(self.rgb_info)
				self.depth_info_pub.publish(self.depth_info)

				self.lh_hand_pub.publish(self.left_hand_marker)
				self.lh_shoulder_pub.publish(self.left_shoulder_marker)
				self.frame_number_pub.publish(frame_num_pt)

			except CvBridgeError as e:
			# except:
				print(e)
				# pass

			self.rate.sleep()


def main(argv):

	rospy.init_node('Publish_Video_Skeleton')

	video_ind = int(sys.argv[1])

	FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/"
	# TRAJ_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Primitive_Library/Subject1"
	TRAJ_DIR = "/home/tanmay/catkin_ws/src/Visualize_Primitives/Data/Smooth_Trajectories/"

	image_filelist = npy.load(os.path.join(FILE_DIR,"Image_File_List.npy"))
	number_frames = npy.load(os.path.join(FILE_DIR,"Number_Images_Sorted.npy")).astype(int)
	sorting_indices = npy.load(os.path.join(FILE_DIR,"Sorting_Indices.npy"))

	image_file = os.path.join(FILE_DIR,image_filelist[video_ind])
	num_frames = number_frames[video_ind]
	# traj_ind = sorting_indices[video_ind]

	traj_ind = int(sys.argv[1])

	path = os.path.join(TRAJ_DIR,"LH_Smooth.npy")
	path_sh = os.path.join(TRAJ_DIR,"LH_Shoulder_Smooth.npy")

	traj = npy.load(path)
	traj_shoulder = npy.load(path_sh)

	# lh_traj_path = os.path.join(TRAJ_DIR,"Traj_{0}".format(traj_ind),"Original_Left_Hand_{0}.npy".format(traj_ind))
	# ls_traj_path = os.path.join(TRAJ_DIR,"Traj_{0}".format(traj_ind),"Original_Left_Shoulder_{0}.npy".format(traj_ind))

	# lh_traj = npy.load(lh_traj_path)/100000
	# lh_shoulder_traj = npy.load(ls_traj_path)/100000

	video_skel = publish_video_skeleton(image_file,num_frames,traj[traj_ind],traj_shoulder[traj_ind])

	try:
		# rospy.spin()
		video_skel.publish_images()
	# except rospy.ROSInterruptException:
		# pass
	except KeyboardInterrupt:
		print("Shutting Down.")

if __name__ == '__main__':
	main(sys.argv)	

