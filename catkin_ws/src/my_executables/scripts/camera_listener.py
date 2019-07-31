#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
	def __init__(self):
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber('/added_camera/camera_for_mirrorin/image_raw',Image,self.callback)
		self.i = 0;
	
	def mirror_inversion(cv_image):
		new_image = cv_image.copy()
		shape = new_image.shape();
		for i in range(shape(0)):
			for j in range(shape(1)):
				new_image[i,shape(1)-j] = cv_image[i,j]
		return new_image
	
	def callback(self,data):
		print("image received")
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		new_image = cv2.flip(cv_image,1)
		cv2.imwrite("robot_vision_inverted"+str(self.i)+".png",new_image)
		self.i+=1
	
			

def main(args):
	ic = image_converter()
	rospy.init_node('image_converter',anonymous= True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	main(sys.argv)
