# General imports
import numpy as np
import re
from nao_ml_inc import *
import time
from copy import copy

# ROS and Gazebo imports
import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from std_msgs.msg import Float64
from nao_ml_inc import *
from gazebo_link_pose import *
from std_msgs.msg import String
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import matplotlib.pyplot as plt


# Explauto imports
from explauto import Environment
from explauto.utils import bounds_min_max    

class NaoEnvironment(Environment):
    use_process = True

    def __init__(self, m_mins, m_maxs, s_mins, s_maxs, joints, ros_topic, ee_link_name, use_reset, skin_map, collision_target_link,topics):
        # Set up default explauto environment parameters
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)

        # Store params for later use
        self.joints = joints
        self.rate = rospy.Rate(10) # 10hz
        self.use_reset = use_reset
        self.skin_map = skin_map
        self.x_values_plot =[]
        self.y_values_plot = []
        for i in skin_map:
        	self.x_values_plot.append(self.skin_map[i][0])
	    	self.y_values_plot.append(self.skin_map[i][1])
	   	plt.ion()

        self.collision_target_link = collision_target_link
        
        self.function_mapping = {'RElbowRoll_callback': self.RElbowRoll_callback, 'RElbowYaw_callback': self.RElbowYaw_callback,'RShoulderPitch_callback':self.RShoulderPitch_callback, 
        							'RShoulderRoll_callback': self.RShoulderRoll_callback, 'RWristYaw_callback': self.RWristYaw_callback, 'HeadYaw_callback':self.HeadYaw_callback,
        							 'HeadPitch_callback':self.HeadPitch_callback }

        # Vector of empty sensory effect
        self.empty_sensory_effect = None
        self.sensory_effect = None
		
        self.flag = False
		
       	self.fil = open('/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/output.txt','w')

        # Some simulation parameters
        self.time_delay = 0.3
        self.time_delay_after = 1.0
        self.max_time_delay = 15.0
        
        #joint properties
        self.joints_prop = [0 for joint in joints]
        
        #camera capture variable
        self.current_image = 1
        self.bridge = CvBridge()
        self.i = 0;

        
        # Subscribe to ROS topics for sensory effects
        self.ros_topic = ros_topic
        # ROS node already initialized in the parent code
        # rospy.init_node("nao_explauto_listener", anonymous=True)
        rospy.Subscriber(ros_topic, String, self.explauto_ros_callback)
        for k in range(len(joints)):
        	print 'Topic is: ',topics[k]
        	rospy.Subscriber(topics[k],Float64,self.function_mapping[joints[k]["joint_name"]+'_callback'])
        
        rospy.Subscriber('/added_camera/camera_for_mirrorin/image_raw',Image,self.image_callback)

        # Reset simulation service
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

        # Gazebo end effector position
        self.ee_link_name = ee_link_name
        self.ee = GazeboLinkPose(ee_link_name)
        
        
    def plot_point_in_skin(self,point,color):
		if point == []:
			plt.scatter(self.x_values_plot,self.y_values_plot,color = 'r')
			plt.draw()
			plt.pause(0.0001)
			plt.savefig('all_points.png')
			return 0
		else:
			plt.scatter([point[0]],[point[1]],color = color)
			plt.draw()
			plt.pause(0.0001)
			plt.savefig('special_points.png')

		return 0
	
    def image_callback(self,msg):
    	try:
    		cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    	except CvBridgeError as e:
    		print(e)
    	self.current_image = cv2.flip(cv_image,1)
    
    def RElbowRoll_callback(self,data):
    	self.joints_prop[0] = data.data
    	
    def RElbowYaw_callback(self,data):
    	self.joints_prop[1] = data.data
    	
    def RShoulderPitch_callback(self,data):
    	self.joints_prop[2] = data.data
    	
    def RShoulderRoll_callback(self,data):
    	self.joints_prop[3] = data.data
    	
    def RWristYaw_callback(self,data):
    	self.joints_prop[4] = data.data
    	
    def HeadYaw_callback(self,data):
    	self.joints_prop[5] = data.data
    	
    def HeadPitch_callback(self,data):
    	self.joints_prop[6] = data.data
    	
    def capture_state(self,expected,observed):
    	#print 'Current state of joint is: ',self.joints_prop
    	picture_name = 'Picture_' + str(self.i) + '.png'
    	cv2.imwrite('/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/pictures/'+picture_name,self.current_image)
    	self.fil.write(str(self.joints_prop) + '	'+expected + '	'+observed + '	'+'pictures/'+picture_name )
    	self.fil.write('\n')
    	self.i +=1

    def compute_motor_command(self, joint_pos_ag):
        return bounds_min_max(joint_pos_ag, self.conf.m_mins, self.conf.m_maxs)

    def compute_sensori_effect(self, joint_pos_env):
        nao_ml_set_pose(joint_pos_env, self.joints)
        self.ee.reset_stable()
        if self.use_reset:
            self.reset_simulation()
        time_spent = 0
        while not self.ee.is_stable():
            time.sleep(self.time_delay)
            # self.rate.sleep()
            time_spent = time_spent + self.time_delay
            if time_spent >= self.max_time_delay:
                print("Delay expired!")
                return self.empty_sensory_effect
        time.sleep(self.time_delay_after)
        if self.flag:
        	self.flag = False
        	return self.sensory_effect
        else:	
        	return [None,None]
        	print 'Returning None from compute_sensori_effects' 

    def explauto_ros_callback(self, msg):
        link_name = re.findall('^nao_skin::([^:]+)', msg.data)
        #print 'link_name is: ',link_name
        #print [self.collision_target_link] == link_name
        #Debug data
        self.sensory_effect_str = msg.data
        if self.sensory_effect_str == 'none':
            #print 'This should have been printed'
            #self.sensory_effect = self.empty_sensory_effect
            # Debug data
            self.sensory_effect_dict_key = ''
        else:
            result = copy(self.empty_sensory_effect)
            # print(msg.data)
            collisions = re.findall('collision_(\d+)', msg.data)
            collisions = [int(x) for x in collisions]
            dict_key = '_'.join(str(x) for x in collisions)
            if dict_key in self.skin_map:
                result = copy(self.skin_map[dict_key])
            else:
            	print 'Not found: ',dict_key

            	
            # print(collisions)
            # for i in collisions:
            #     result[i] = 1

            # Debug data
            self.sensory_effect_dict_key = dict_key

            # Return value
            #print 'Flag true, sensory_effect is: ',self.sensory_effect_str
            self.flag = True
            self.sensory_effect = [dict_key,result]
        # print(self.sensory_effect)
        # print(msg.data)
        # print(link_name)

    def plot(self, ax, m, s, **kwargs_plot):
        pass

