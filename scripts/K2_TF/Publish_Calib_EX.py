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

	def __init__(self):

		self.tf_rgb_depth = tf.TransformBroadcaster()
		self.tf_rgb_depth_playback = tf.TransformBroadcaster()
		self.tf_rgb_traj = tf.TransformBroadcaster()

		self.rate = rospy.Rate(100.)

		self.rot4 = npy.zeros((4,4))
		self.rot4[3,3] = 1.
		self.rot4[:3,:3] = npy.array([[ 9.9998909681453751e-01, 4.4429083353708810e-03, -1.4376430603502157e-03], [-4.4479772161444501e-03, 9.9998383455554196e-01, -3.5420511401037971e-03], [1.4218828116764972e-03, 3.5484071240406844e-03, 9.9999269350138353e-01 ]])
									# [[9.9998171646913669e-01, 4.7724707966295436e-03, -3.7135225776191551e-03], [-4.7649866537541946e-03, 9.9998660388787863e-01, 2.0216198892244260e-03], [3.7231209527375615e-03, -2.0038880413539950e-03, 9.9999106136159488e-01]]
		self.quaternion = tf.transformations.quaternion_from_matrix(self.rot4)
		self.translation = npy.array([-5.2147013476156308e-02, -9.2332191929765154e-05, -9.8898030319474121e-04])
									# -5.3404696633386789e-02, 1.3946304803673395e-04, 4.3839200723898306e-03 

		self.tq = tf.transformations.quaternion_from_euler(0.,0.,0.)
	def publish_transform(self):

		while not(rospy.is_shutdown()):

			self.tf_rgb_depth.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"kinect2_ir_optical_frame","kinect2_rgb_optical_frame")				
			# self.tf_rgb_depth.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"depth_optical_frame","rgb_optical_frame")						

			self.tf_rgb_depth_playback.sendTransform((self.translation),(self.quaternion),rospy.Time.now(),"rgb_optical_frame","depth_optical_frame")						
			# self.tf_rgb_depth_playback.sendTransform((0.,0.,0.),(0.,0.,0.,1.),rospy.Time.now(),"depth_optical_frame","rgb_optical_frame")						

			self.tf_rgb_traj.sendTransform((-0.5,0.5,0.),(self.tq),rospy.Time.now(),"traj_frame","rgb_optical_frame")						
			
			self.rate.sleep()

def main(argv):

	rospy.init_node('Publish_Dynamic_TF')

	tf_inst = dynamic_tf()

	try:
		# rospy.spin()
		tf_inst.publish_transform()
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)	

