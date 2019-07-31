#!/usr/bin/env python
# General imports
import numpy as np
import pickle
import time
from operator import add

# ROS and Gazebo imports
import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from std_msgs.msg import Float64

def talker():
	pub = rospy.Publisher('/nao_dcm/RShoulderRoll_position_controller/command',Float64,queue_size = 5)
	rospy.init_node('talker',anonymous=True)
	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		pub.publish(-1.3);
		rate.sleep()
		
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException: pass



