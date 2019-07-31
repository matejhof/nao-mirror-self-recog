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

# Explauto imports
from explauto import Environment
from explauto.utils import bounds_min_max    

class NaoEnvironment(Environment):
    use_process = True

    def __init__(self, m_mins, m_maxs, s_mins, s_maxs, joints, ros_topic, ee_link_name, use_reset, total_sensors, skin_map, collision_target_link):
        # Set up default explauto environment parameters
        s_mins = [0 for i in range(total_sensors)]
        s_maxs = [1 for i in range(total_sensors)]
        Environment.__init__(self, m_mins, m_maxs, s_mins, s_maxs)

        # Store params for later use
        self.joints = joints
        self.rate = rospy.Rate(10) # 10hz
        self.use_reset = use_reset
        self.total_sensors = total_sensors
        self.skin_map = skin_map
        self.collision_target_link = collision_target_link

        # Vector of empty sensory effect
        self.empty_sensory_effect = [0 for i in range(self.total_sensors)]
        self.sensory_effect = self.empty_sensory_effect

        # Some simulation parameters
        self.time_delay = 0.1
        self.time_delay_after = 0.3
        self.max_time_delay = 5.0
        
        # Subscribe to ROS topics for sensory effects
        self.ros_topic = ros_topic
        # ROS node already initialized in the parent code
        # rospy.init_node("nao_explauto_listener", anonymous=True)
        rospy.Subscriber(ros_topic, String, self.explauto_ros_callback)
        
        # Reset simulation service
        self.reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        
        # Gazebo end effector position
        self.ee_link_name = ee_link_name
        self.ee = GazeboLinkPose(ee_link_name)

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
                # print("Delay expired!")
                return copy(self.empty_sensory_effect)
        time.sleep(self.time_delay_after)
        return self.sensory_effect
        
    def explauto_ros_callback(self, msg):
        link_name = re.findall('^nao_skin::([^:]+)', msg.data)
        if link_name != [self.collision_target_link]:
            self.sensory_effect = copy(self.empty_sensory_effect)
        else:
            result = copy(self.empty_sensory_effect)
            # print(msg.data)
            collisions = re.findall('collision_(\d+)', msg.data)
            collisions = [int(x) for x in collisions]
            dict_key = '_'.join(str(x) for x in collisions)
            if dict_key in self.skin_map:
                result[self.skin_map[dict_key]] = 1
            # print(collisions)
            # for i in collisions:
            #     result[i] = 1
            self.sensory_effect = result
        # print(self.sensory_effect)
        # print(msg.data)
        # print(link_name)

    def plot(self, ax, m, s, **kwargs_plot):
        pass

