#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import Image,JointState
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
# from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
import random
import csv

class Central:


    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.i = 0
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        # self.stiffness_shoulder = 0.0
        # self.stiffness_head = 0.9
        # self.stiffness_elbow = 0.9
        self.Button_Number = 0  # indicates status of last button pressed
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        joints = [
    
    # Head
    {
        "joint_name": "HeadYaw",
        "publisher": None,
        "limit_min": -2.08567,
        "limit_max": 2.08567
    },
    {
        "joint_name": "HeadPitch",
        "publisher": None,
        "limit_min": -0.671952,
        "limit_max": 0.514872
    },
        ]

    # Initialize joints
        topics = []
        for joint in joints:
            topic_name = '/nao_dcm/' + joint['joint_name'] + '_position_controller/command';
            topics.append(topic_name);
            joint['publisher'] = rospy.Publisher(topic_name, Float64, queue_size=5)

        self.joints = joints
        

        pass



    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

	# def callback(self,data):
    #     # bridge = CvBridge()
	# 	print("image received")
	# 	try:
	# 		cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
	# 	except CvBridgeError as e:
	# 		print(e)
	# 	new_image = cv2.flip(cv_image,1)
	# 	cv2.imwrite("robot_vision_inverted"+str(self.i)+".png",new_image)
	# 	self.i+=1

    def callback(self, data):
        bridge = CvBridge()
        print('image received')
        try:
            cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
        new_image = cv2.flip(cv_image, 1)
        cv2.imwrite('robot_vision_inverted'+str(self.i)+'.png', new_image)
        self.i+=1
        self.image_sub.unregister()


    def set_head_angles(self,head_angle1,head_angle2): ## controls head movement
        self.joints[0]['publisher'].publish(head_angle1)
        self.joints[1]['publisher'].publish(head_angle2)

    def head_angle_record(self, yaw, pitch):            # Record the head joint angles of yaw and pitch
        with open('head_joint.csv', 'a') as samples:
            samples_writer = csv.writer(samples)
            samples_writer.writerow([self.i, yaw, pitch])
        samples.close()



    def central_execute(self):
        # Initialize ROS
        rospy.init_node('nao_skin_ml', anonymous=True)
        srv_joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)

        ros_topic = '/gazebo_contact_info/r_wrist'

        # create several topic subscribers
        # rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber(ros_topic,String,self.joints_cb)
        # rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        # rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        # topic_name1 = '/nao_dcm/HeadYaw_position_controller/command';
        # topic_name2 = '/nao_dcm/HeadPitch_position_controller/command';
        # self.jointPub = rospy.Publisher(topic_name1,Float64,queue_size=10)
        # self.jointPub = rospy.Publisher(topic_name2,Float64,queue_size=10)


        # test sequence to demonstrate setting joint angles
        # self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        # rospy.sleep(1.0)
        # self.set_head_angles(0.5, 0.5)
        # rospy.sleep(3.0)
        self.set_head_angles(0.0, 0.0)
        rospy.sleep(3.0)
        # self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms
        while not rospy.is_shutdown():
            try:
                head_yaw = random.uniform(-0.0872664626, 0.0872664626)         # set the head Yaw
                head_pitch = random.uniform(-0.0872664626, 0.0872664626)         # set the head Pitch
                self.set_head_angles(head_yaw, head_pitch)
                rospy.sleep(2.0)
                self.image_sub = rospy.Subscriber('/added_camera/camera_for_mirrorin/image_raw',Image, self.callback)
                self.head_angle_record(head_yaw, head_pitch)
                print("here")
            except KeyboardInterrupt:
                print("Shutting down")
                break

            
        # self.set_stiffness(False)
        

    # rospy.spin() just blocks the code from exiting, if you need to do any periodic tasks use the above loop
    # each Subscriber is handled in its own thread
    #rospy.spin()


if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
