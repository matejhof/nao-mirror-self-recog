#!/usr/bin/env python
from __future__ import division
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
import imutils
import glob
import sys
import os
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
import math
from Centre_Function import NNet

# Get the path to where this file locates
path2current = sys.argv[0]
pathname = os.path.dirname(path2current)

# Get the head filter
head_filter_name = glob.glob(pathname + '/Head_Filter/*.png')
head_filter_im = cv2.imread(head_filter_name[0])
head_filter = cv2.cvtColor(head_filter_im, cv2.COLOR_BGR2GRAY)
# plt.imshow(head_filter, cmap = "gray")
# plt.show()

print(type(head_filter[0,0]))

w, h = head_filter.shape[::-1]

class Decoder(nn.Module):
    def __init__(self):
        super(Decoder, self).__init__()

        self.decoder = nn.Sequential(
            nn.Linear(2, 12),
            nn.Tanh(),
            nn.Linear(12, 64),
            nn.Tanh(),
            nn.Linear(64, 128),
            nn.Tanh(),
            nn.Linear(128, w*h),
            nn.Sigmoid(),       # compress to a range (0, 1)
        )

    def forward(self, x):
        decoded = self.decoder(x)
        return decoded


class Central:


    def __init__(self, head_filter):
        # initialize class variables
        self.motionProxy = ALProxy("ALMotion", "10.152.246.191", 9559)
        self.postureProxy = ALProxy("ALRobotPosture", "10.152.246.191", 9559)
        self.videoRecorderProxy = ALProxy("ALVideoRecorder", "10.152.246.191", 9559)
        # self.motionProxy = ALProxy("ALMotion", "10.152.246.74", 9559)
        # self.postureProxy = ALProxy("ALRobotPosture", "10.152.246.74", 9559)
        self.joint_names = []
        self.i = 0
        self.head_moved = 0
        self.cv_image = 0
        self.x=0
        self.y=0
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.head_yaw = 0 
        self.head_pitch = 0
        self.novelty_thres = 0.048039216               # 0.018 is ok, 0.058039216 is also ok
        self.centroid_record_button = 0
        self.arm_record_button = 0
        # self.stiffness_shoulder = 0.0
        # self.stiffness_head = 0.9
        # self.stiffness_elbow = 0.9
        self.Button_Number = 0  # indicates status of last button pressed
        self.pNames = ["HeadYaw", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll"]
        self.pStiffnessLists = [[0.0], [0.0], [0.0], [0.0], [0.9], [0.0]]
        self.pTimeLists = [[1.0], [1.0], [1.0], [1.0], [1.0], [1.0]]
        self.motionProxy.stiffnessInterpolation(self.pNames, self.pStiffnessLists, self.pTimeLists)
        self.videoRecorderProxy.setResolution(1)
        self.videoRecorderProxy.setFrameRate(10)
        # self.videoRecorderProxy.setVideoFormat("MJPG")
        

        # Load data
        self.decoder = Decoder()
        self.decoder.load_state_dict(torch.load(pathname + '/Trained_Networks/trained_decoder_' + 'grayscale'))
        self.decoder.eval()

        self.mu = np.load(pathname + '/Variance_Parameters/mu_for_decoder.npy')       # Load mu
        self.s = np.load(pathname + '/Variance_Parameters/sigma_for_decoder.npy')       # Load s

        self.head_filter = head_filter
        

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
            self.Button_Number = 0
            self.centroid_record_button = 0
            self.arm_record_button = 0

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")

            joint_angle = np.array([self.head_yaw, self.head_pitch])
            decoded_data = self.decoder(torch.from_numpy(joint_angle).float())

            im_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            method = eval('cv2.TM_CCOEFF')
            res = cv2.matchTemplate(im_gray, self.head_filter, method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

            top_left = max_loc

            cropped_img = im_gray[top_left[1]:top_left[1] + h, top_left[0]: top_left[0] + w].copy()         # For greyscale

            diff = np.true_divide((abs(abs((cropped_img - 255 *np.reshape(decoded_data.data.numpy().astype("float64"), (h, w)))) - self.mu)), (self.s + 0.00001))

            thresh = cv2.threshold(diff, 255*self.novelty_thres, 255, cv2.THRESH_BINARY)[1]
            thresh = thresh.astype("uint8")
            cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)

            area=[]
            for i in range(len(cnts)):
                area.append(cv2.contourArea(cnts[i]))

            if len(area)>0:
                max_idx = np.argmax(area)
                if area[max_idx] < 30 or area == []:
                    self.x=0
                    self.y=0

                else:
                    (x, y, w1, h1) = cv2.boundingRect(cnts[max_idx])
                    cv2.rectangle(cv_image, (top_left[0] + x, top_left[1] + y), (top_left[0] + x + w1, top_left[1] + y + h1), (0, 0, 255), 2)
                    self.x = top_left[0] + x + math.floor(w1 / 2)
                    self.y = top_left[1] + y + math.floor(h1 / 2)
                    self.touching_data_record(self.x, self.y)
                    cv2.circle(cv_image, (int(self.x), int(self.y)), 2, (0, 255, 0), -1)

            else:
                self.x=0
                self.y=0


        except CvBridgeError as e:
            rospy.logerr(e)

        cv2.imshow("image window",cv_image)
        cv2.imshow("image window_gray",im_gray)
        cv2.imshow("threshold",thresh)
        cv2.imshow("face",cropped_img)
        cv2.imshow("output",np.reshape(decoded_data.data.numpy().astype("float64"), (h, w)))

        
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

    # Centroid of novelty and corresponding arm joints record
    def touching_data_record(self, x, y):

        if(self.Button_Number ==1): # button 1 pressed

            if(self.centroid_record_button==0):
                print("Centroid is:", x, y)
                self.centroid_record_button = 1
                with open('centroid_samples.csv', 'a') as samples1:
                #it is  'a' as 'append' : don't forget to delete the file if you want to change the data and not just to add to the previous ones!
                    samples_writer1 = csv.writer(samples1)

                    samples_writer1.writerow([x, y])
                samples1.close()
                print("Centroid point is recorded. Please record the arm joints")
            self.Button_Number = 0
            
            # pass

        elif(self.Button_Number ==2): # button 2 pressed

            if(self.arm_record_button==0):
                print("HeadYaw, HeadPitch, Lshoulderpitchs, Lshoulderroll, LElbowRoll:", self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3], self.joint_angles[5])
                self.arm_record_button = 1
                with open('arm_samples.csv', 'a') as samples2:
                #it is  'a' as 'append' : don't forget to delete the file if you want to change the data and not just to add to the previous ones!
                    samples_writer2 = csv.writer(samples2)

                    samples_writer2.writerow([self.joint_angles[0], self.joint_angles[1], self.joint_angles[2], self.joint_angles[3], self.joint_angles[5]])
                samples2.close()
                self.motionProxy.stiffnessInterpolation(["HeadYaw", "HeadPitch"], [[0.9], [0.9]], [[1.0], [1.0]])
                self.set_head_angles(self.head_yaw, self.head_pitch)
                rospy.sleep(2.0)
                self.motionProxy.stiffnessInterpolation(["HeadYaw", "HeadPitch"], [[0.0], [0.0]], [[1.0], [1.0]])
                print("Arm joints are recorded. Please move the post-it")
            self.Button_Number = 0
            
            # pass
        
        else:
            pass


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
    
    def hand_move(self,angle): # controls left shoulder movement
        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("RHand") # each joint has a specific name, look into the joint_state topic or google
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
        joint_angles_to_set.speed = 0.1 # keep this low if you can
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

        rate = rospy.Rate(10) # sets the sleep time to 10ms
        self.set_stiffness(True)
        while not rospy.is_shutdown():
            try:
                # self.head_yaw = random.uniform(-0.0872664626, 0.0872664626)         # set the head Yaw
                # self.head_pitch = random.uniform(-0.0872664626, 0.0872664626)         # set the head Pitch
                self.head_yaw = 0        # set the head Yaw
                self.head_pitch = 0         # set the head Pitch
                self.set_head_angles(self.head_yaw, self.head_pitch)
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

    head_filter_name = glob.glob(pathname + '/Head_Filter/*.png')
    head_filter_im = cv2.imread(head_filter_name[0])
    head_filter = cv2.cvtColor(head_filter_im, cv2.COLOR_BGR2GRAY)
    # plt.imshow(head_filter, cmap = "gray")
    # plt.show()
    

    w, h = head_filter.shape[::-1]

    central_instance = Central(head_filter)
    central_instance.central_execute()