#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import os
import numpy as npy

def publish_images(FILE_DIR, num_frames):

	rgb_pub = rospy.Publisher("rgb_image",Image,queue_size=10)
	depth_pub = rospy.Publisher("depth_image",Image,queue_size=10)
	rgb_info_pub = rospy.Publisher("camera_info_rgb",CameraInfo,queue_size=10)
	depth_info_pub = rospy.Publisher("camera_info_depth",CameraInfo,queue_size=10)
	bridge = CvBridge()
	# while not rospy.is_shutdown():

	rgb_info = CameraInfo()
	rgb_info.header.frame_id = "rgb_optical_frame"
	rgb_info.height = 480
	rgb_info.width = 640
	rgb_info.distortion_model = 'plumb_bob'
	rgb_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
	rgb_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
	rgb_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	rgb_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]

	depth_info = CameraInfo()
	depth_info.header.frame_id = "depth_optical_frame"
	depth_info.height = 480
	depth_info.width = 640
	depth_info.distortion_model = 'plumb_bob'
	depth_info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
	depth_info.K = [525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0]
	depth_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
	depth_info.P = [525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 239.5, 0.0, 0.0, 0.0, 1.0, 0.0]

	rate = rospy.Rate(20)

	for i in range(1,num_frames):
		img = cv2.imread(os.path.join(FILE_DIR,"RGB_{0}.png".format(i)))
		imgd = cv2.imread(os.path.join(FILE_DIR,"Depth_{0}.png".format(i)))		
		# print(imgd.dtype)

		imgd = cv2.cvtColor(imgd,cv2.COLOR_BGR2GRAY)
		imgd = npy.uint16(imgd)
		# print(imgd.dtype)

		img_rgb = bridge.cv2_to_imgmsg(img,"bgr8")
		img_rgb.header.frame_id = "rgb_optical_frame"
		img_rgb.header.seq = i
		img_rgb.header.stamp = rospy.Time.now()
		img_rgb.height = 480
		img_rgb.width = 640

		# img_depth = bridge.cv2_to_imgmsg(imgd,"bgr8")
		img_depth = bridge.cv2_to_imgmsg(imgd,"passthrough")
		# img_depth = bridge.cv2_to_imgmsg(imgd)

		img_depth.header.frame_id = "depth_optical_frame"
		img_depth.header.seq = i
		img_depth.header.stamp = rospy.Time.now()
		img_depth.height = 480
		img_depth.width = 640

		rgb_info.header.seq = i
		rgb_info.header.stamp = rospy.Time.now()

		depth_info.header.seq = i
		depth_info.header.stamp = rospy.Time.now()

		try:
			rgb_pub.publish(img_rgb)
			depth_pub.publish(img_depth)
			rgb_info_pub.publish(rgb_info)
			depth_info_pub.publish(depth_info)
		except CvBridgeError as e:
			print(e)

		rate.sleep()


def main(argv):

	rospy.init_node('Img_Pub')

	filelist = ['taking_food/0510180532', 'taking_food/0510180218', 'taking_food/0510180342', 'taking_medicine/1204143959', 'taking_medicine/1204144120', 'taking_medicine/1204142858', 'microwaving_food/1204150828', 'microwaving_food/1204151136','microwaving_food/1204150645', 'cleaning_objects/0510181415', 'cleaning_objects/0510181310', 'cleaning_objects/0510181236', 'making_cereal/1204142227', 'making_cereal/1204142616','making_cereal/1204142055', 'making_cereal/1204142500','stacking_objects/1204145234', 'stacking_objects/1204144736', 'stacking_objects/1204144410', 'unstacking_objects/1204145630','unstacking_objects/1204145527', 'unstacking_objects/1204145902', 'arranging_objects/0510175554', 'arranging_objects/0510175431', 'arranging_objects/0510175411', 'having_meal/0510182057', 'having_meal/0510182019', 'having_meal/0510182137', 'picking_objects/0510175829', 'picking_objects/0510175855','picking_objects/0510175921']

	file = os.path.join("/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/",filelist[8])
	# file = "/home/tanmay/Research/Code/ActionPrimitives/Data/Cornell_Data/Subject1_rgbd_images/arranging_objects/0510175411"
	num_frames = 648

	try:
		publish_images(file,num_frames)
	except rospy.ROSInterruptException:
		pass

if __name__ == '__main__':
	main(sys.argv)	

