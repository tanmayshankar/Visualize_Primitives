#!/usr/bin/env python
# from __future__ import print_function
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError


class image_converter:

	def __init__(self):

		self.image_pub_rgb = rospy.Publisher("img_rgb",Image,queue_size=10)
		self.image_pub_d = rospy.Publisher("img_depth",Image,queue_size=10)
		self.info_pub = rospy.Publisher("camera_info",CameraInfo,queue_size=10)

		self.info_sub = rospy.Subscriber("/data_throttled_camera_info",CameraInfo,self.callback)
		self.bridge = CvBridge()

	def callback(self,data):
		img = cv2.imread("/home/tanmay/catkin_ws/src/Viz_Primitives/scripts/RGB_153.png")	
		imgd = cv2.imread("/home/tanmay/catkin_ws/src/Viz_Primitives/scripts/Depth_153.png")				

		img_rgb = self.bridge.cv2_to_imgmsg(img,"bgr8")
		img_rgb.header.frame_id='/openni_rgb_optical_frame'
		img_depth = self.bridge.cv2_to_imgmsg(imgd,"passthrough")
		img_depth.header.frame_id='/openni_depth_optical_frame'

		try:
			self.image_pub_rgb.publish(img_rgb)
			self.image_pub_d.publish(img_depth)

			# self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
			# self.image_pub_d.publish(self.bridge.cv2_to_imgmsg(imgd,"bgr8"))
			self.info_pub.publish(data)			
		except CvBridgeError as e:
			print(e)

	# def publish(self):
	# 	img = cv2.imread("/home/tanmay/catkin_ws/src/Viz_Primitives/scripts/RGB_153.png")	
	# 	imgd = cv2.imread("/home/tanmay/catkin_ws/src/Viz_Primitives/scripts/Depth_153.png")			

	# 	# while not rospy.is_shutdown():
	# 	# 	try:
	# 	# 		self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(img,"bgr8"))
	# 	# 		self.image_pub_d.publish(self.bridge.cv2_to_imgmsg(imgd,"bgr8"))
	# 	# 	except CvBridgeError as e:
	# 	# 		print(e)

def main(argv):

	ic = image_converter()
	rospy.init_node('Img_Conv',anonymous=True)
	# ic.publish()
	try:					
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down.")
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)