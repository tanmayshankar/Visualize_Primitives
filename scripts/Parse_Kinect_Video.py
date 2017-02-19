#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

	def __init__(self):

		# self.image_pub_rgb = rospy.Publisher("img_rgb",Image,queue_size=10)
		# self.image_pub_d = rospy.Publisher("img_depth",Image,queue_size=10)
		# self.info_pub = rospy.Publisher("camera_info",CameraInfo,queue_size=10)
		# self.info_sub = rospy.Subscriber("/data_throttled_camera_info",CameraInfo,self.callback)

		# self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_color_rect",Image,self.rgb_callback)
		self.d_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect",Image,self.d_callback)
		self.rgb_counter = 0
		self.d_counter = 0
		self.count = 0
		self.bridge = CvBridge()

	def rgb_callback(self,data):
		
		try:
			rgb_img = self.bridge.imgmsg_to_cv2(data,"bgr8")
		except CvBridgeError as e:
			print(e)

		# print("RGB:",data.header.stamp)
		if (self.rgb_counter==0):
			cv2.imwrite("RGB_{0}.png".format(self.rgb_counter),rgb_img)
		self.rgb_counter += 1
		print(self.rgb_counter)

	def d_callback(self,data):
		
		try:
			d_img = self.bridge.imgmsg_to_cv2(data,"passthrough")
		except CvBridgeError as e:
			print(e)

		# print("Dep",data.header.stamp)	

		if (self.d_counter==0):
			cv2.imwrite("Depth_{0}.png".format(self.d_counter),d_img)
			# self.count += 1
		self.d_counter += 1

def main(argv):

	ic = image_converter()
	rospy.init_node('Img_Conv',anonymous=True)

	try:					
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting Down.")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)