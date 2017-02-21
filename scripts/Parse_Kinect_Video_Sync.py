#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import message_filters

class image_converter:

	def __init__(self):

		# self.rgb_sub = rospy.Subscriber("/kinect2/hd/image_color_rect",Image)
		# self.d_sub = rospy.Subscriber("/kinect2/hd/image_depth_rect",Image)

		self.rgb_sub = message_filters.Subscriber("/kinect2/hd/image_color_rect",Image)
		self.d_sub = message_filters.Subscriber("/kinect2/hd/image_depth_rect",Image)

		self.time_sync = message_filters.TimeSynchronizer([self.rgb_sub,self.d_sub],10)
		self.time_sync.registerCallback(self.rgbd_callback)
		self.rgb_counter = 0
		self.d_counter = 0
		self.count = 0
		self.bridge = CvBridge()

	def rgbd_callback(self,data_rgb,data_d):

		try:
			rgb_img = self.bridge.imgmsg_to_cv2(data_rgb,"bgr8")
		except CvBridgeError as e:
			print(e)

		try:
			d_img = self.bridge.imgmsg_to_cv2(data_d,"passthrough")
		except CvBridgeError as e:
			print(e)
	
		cv2.imwrite("RGB_{0}.png".format(self.rgb_counter),rgb_img)		
		cv2.imwrite("Depth_{0}.png".format(self.d_counter),d_img)
		self.rgb_counter += 1
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