#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import argparse
from naoqi import ALProxy
import csv
import random


class Central:


    def __init__(self):
        # initialize class variables
        self.motionProxy = ALProxy("ALMotion", "10.152.246.74", 9559)
        self.postureProxy = ALProxy("ALRobotPosture", "10.152.246.74", 9559)
        self.joint_names = []
        self.i = 0
        self.head_moved = 0
        self.cv_image = 0
        self.x=0
        self.y=0
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        # self.stiffness_shoulder = 0.0
        # self.stiffness_head = 0.9
        # self.stiffness_elbow = 0.9
        self.Button_Number = 0  # indicates status of last button pressed
        self.pNames = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        self.pStiffnessLists = [[0.9], [0.9], [0.0], [0.0], [0.9], [0.9]]
        self.pTimeLists = [[1.0], [1.0], [1.0], [1.0], [1.0], [1.0]]
        self.motionProxy.stiffnessInterpolation(self.pNames, self.pStiffnessLists, self.pTimeLists)
        

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

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    def touch_cb(self,data): # added part to save last button pressed
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
        if (data.button == 1 and data.state==1):
            self.Button_Number = 1
   
            # self.set_stiffness(True) #to always return to normal position
        elif (data.button == 2 and data.state==1):
            self.Button_Number = 2
        elif (data.button == 3 and data.state==1):
            self.Button_Number = 3
        else:
            pass

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")  

        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("image window",cv_image)
        if self.head_moved == 1:
            self.cv_image = cv_image
        else:
            pass
        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

    # Images recorde
    def img_record(self):
        cv = self.cv_image
        cv2.imwrite('robot_vision_inverted'+str(self.i)+'.png', cv)
        self.i+=1
        self.head_moved = 0

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)
    def Elbow_yaw_L_moveto(self,arm_angle): # controls left elbow movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LElbowYaw") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(arm_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
    def Elbow_roll_L_moveto(self,angle): # controls left elbow movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LElbowRoll") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)

    def Shoulder_pitch_L_moveto(self,arm_angle): # controls left shoulder movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LShoulderPitch") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(arm_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
    def Shoulder_roll_L_moveto(self,angle): # controls left shoulder movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LShoulderRoll") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)


    def Elbow_yaw_R_moveto(self,arm_angle): # controls right elbow movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("RElbowYaw") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(arm_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)

    def set_head_angles(self,head_angle1, head_angle2): ## controls head movement

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("HeadYaw") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(head_angle1) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.08 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
        joint_angles_to_set.joint_names.append("HeadPitch")
        joint_angles_to_set.joint_angles.append(head_angle2)
        self.jointPub.publish(joint_angles_to_set)

    def head_angle_record(self, yaw, pitch):            # Record the head joint angles of yaw and pitch
        with open('head_joint_real.csv', 'a') as samples:
            samples_writer = csv.writer(samples)
            samples_writer.writerow([yaw, pitch])
        samples.close()

    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)


        # test sequence to demonstrate setting joint angles
        # self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        self.postureProxy.goToPosture("Crouch", 1.0)
        rospy.sleep(1.0)
        self.set_head_angles(0.0, 0.0)
        rospy.sleep(3.0)
        # self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rate = rospy.Rate(10) # sets the sleep time to 10ms
        # while not rospy.is_shutdown():
        #     pass
        while not rospy.is_shutdown():
            try:
                head_yaw = random.uniform(-0.0872664626, 0.0872664626)         # set the head Yaw
                head_pitch = random.uniform(-0.0872664626, 0.0872664626)         # set the head Pitch
                self.set_head_angles(head_yaw, head_pitch)
                print(head_yaw, head_pitch)
                self.head_moved = 1
                rospy.sleep(2.0)
                self.img_record()
                self.head_angle_record(head_yaw, head_pitch)
                rospy.sleep(1.0)
            except KeyboardInterrupt:
                print("Shutting down")
                break

        # rate.sleep()
        # f.close()
        

    # rospy.spin() just blocks the code from exiting, if you need to do any periodic tasks use the above loop
    # each Subscriber is handled in its own thread
    #rospy.spin()


if __name__ == '__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
    
    