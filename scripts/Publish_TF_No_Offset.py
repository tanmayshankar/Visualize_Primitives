#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import message_filters
import os
import tf
import tf2_ros
import numpy as npy

class dynamic_tf:

	def __init__(self,global_tf):
	# def __init__(self,global_tf,objects):

		self.transform = global_tf
		self.tf_cam_rgb = tf.TransformBroadcaster()
		self.tf_cam_depth = tf.TransformBroadcaster()
		self.tf_rgb_traj = tf.TransformBroadcaster()

		self.rate = rospy.Rate(100.)

		self.rot4 = npy.zeros((4,4))
		self.rot4[3,3] = 1.
		self.rot4[:3,:3] = self.transform[:3,:3]
		self.scale = 100000

		self.quaternion = tf.transformations.quaternion_from_matrix(self.rot4)
		self.translation = self.transform[:,3]/self.scale

	def publish_transform(self):

		while not(rospy.is_shutdown()):

			# self.tf_cam_rgb.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"cam_frame","rgb_optical_frame")
			# self.tf_cam_depth.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"cam_frame","depth_optical_frame")

			# self.tf_rgb_traj.sendTransform((0.,0.,0.),(0.,0.,0.,1.),rospy.Time.now(),"rgb_optical_frame","traj_frame")

			self.tf_cam_rgb.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"rgb_optical_frame","cam_frame")
			self.tf_cam_depth.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"depth_optical_frame","cam_frame")

			# self.tf_rgb_traj.sendTransform((0.,0.,0.004),(0.,0.,0.,1.),rospy.Time.now(),"traj_frame","rgb_optical_frame")			
			self.tf_rgb_traj.sendTransform((0.,0.,0.),(0.,0.,0.,1.),rospy.Time.now(),"traj_frame","rgb_optical_frame")

			self.rate.sleep()

def main(argv):

	rospy.init_node('Publish_Dynamic_TF')

	FILE_DIR = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_annotations"
	transforms = npy.load(os.path.join(FILE_DIR,"Global_Transforms.npy"))

	# traj_ind = 13	
	# traj_ind = 10
	
	traj_ind = int(sys.argv[1])
	tf_inst = dynamic_tf(transforms[traj_ind])

	try:
		# rospy.spin()
		tf_inst.publish_transform()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)	
