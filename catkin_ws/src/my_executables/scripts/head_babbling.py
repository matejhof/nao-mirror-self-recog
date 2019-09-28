#!/usr/bin/env python
# General imports
import numpy as np
import pickle
from operator import add

# ROS and Gazebo imports
import rospy
from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from std_msgs.msg import Float64
from nao_ml_inc import *
from gazebo_link_pose import *

# Explauto imports
from explauto import Environment
from nao_explauto_env_2_max import NaoEnvironment
from explauto import SensorimotorModel, InterestModel
from explauto.sensorimotor_model import sensorimotor_models

# If True, just test saved model
load_model = False

# If true, save learnt model to disk
save_model = True

# Defines if we want to reset simulation when applying motor commands (Gazebo)
use_reset = True

# Max iterations of the learning process
max_learning_iterations = 1000

# Number of sensors
total_sensors = 240

# End effector link name
ee_link_name = 'nao_skin::r_wrist'

# ROS topic name(s) for Gazebo contacts
ros_topic = '/gazebo_contact_info/r_wrist'

# Target link on which collisions are detected
collision_target_link = 'Head'

# Home pose for the exploration
home_pose = [1.117, 0.083, -0.87, -1.3, 1.638, -0.21, -0.12]
zero_pose = [0, 0, 0, 0, 0, 0, 0]

# Joint data structure
joints = [
    # Torso
    {
        "joint_name": "RElbowRoll",
        "publisher": None,
        "limit_min": 0.0349066,
        "limit_max": 1.54462
    },
    {
        "joint_name": "RElbowYaw",
        "publisherg": None,
        "limit_min": -2.08567,
        "limit_max": 2.08567
    },
    {
        "joint_name": "RShoulderPitch",
        "publisher": None,
        "limit_min": -2.08567,
        "limit_max": 2.08567
    },
    {
        "joint_name": "RShoulderRoll",
        "publisher": None,
        "limit_min": -1.32645,
        "limit_max": 0.314159
    },
    {
        "joint_name": "RWristYaw",
        "publisher": None,
        "limit_min": -1.82387,
        "limit_max": 1.82387
    },
    
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


# Skin to coordinates mapping
skin_map = {}
map_fh = open('/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/my_executables/scripts/coordinate-maps/highres-head.txt', 'r')
for i in range(total_sensors):
    k = str(i) + '_' + str(i + 1)
    skin_map[k] = [float(x) for x in map_fh.readline().strip().split(" ")]
map_fh.close()

# Initialize ROS
rospy.init_node('nao_skin_ml', anonymous=True)
srv_joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)

# Initialize joints
topics = []
for joint in joints:
    topic_name = '/nao_dcm/' + joint['joint_name'] + '_position_controller/command';
    topics.append(topic_name);
    joint['publisher'] = rospy.Publisher(topic_name, Float64, queue_size=5)

# Initialize explauto environment
joint_min = [x['limit_min'] for x in joints]
joint_max = [x['limit_max'] for x in joints]
explauto_env = NaoEnvironment(joint_min, joint_max, [], [], joints, ros_topic, ee_link_name, use_reset, skin_map, collision_target_link,topics)

# Explauto model
#model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'default')
#model.mode = 'exploit'

# Reset output data file
fh = open('/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/my_executables/scripts/output/data.txt', 'w')
fh.close()

# Initialization complete


print("NAO Skin ML: Initialization complete")


# Load saved model from disk
filename = '/home/deepself/Shengzhi_Project/nao-mirror-self-recog/catkin_ws/src/my_executables/scripts/highres-models/model-1000.sav'
fh = open(filename, 'rb')
model = pickle.load(fh)
fh.close()
print 'Model loaded, model size: ',model.size()

# Test learnt model
explauto_env.use_reset = True
#explauto_env.plot_point_in_skin([],'k')


for i in skin_map:
	#explauto_env.plot_point_in_skin([],'k')
	observed = [0 for x in range(total_sensors)]
	expected = [0 for x in range(total_sensors)]	
	#explauto_env.plot_point_in_skin(skin_map[i],'k')
	exp = i.split('_')
	k = int(exp[0])
	if k == 239:
		expected[k] = 1
	else:
		expected[k],expected[k+1]= 1,1
   	s_g = skin_map[i]
   	m = model.inverse_prediction(s_g)
   	s = explauto_env.compute_sensori_effect(m)
   	if s != [None,None] and s != ['1',None]:
   		if s[1] != None:
   			s,point = s[0],s[1]
   			#explauto_env.plot_point_in_skin(point,'g')
   		else:
   			continue
   			
   		obs = s.split('_')
   		k = int(obs[0])
   		print 'This is k: ',k
   		if k == 239:
   			observed[k] = 1;
   		else:
   			observed[k],observed[k+1]= 1,1
   		print 'This is the output: ',s,' and this is expected: ',i
   		observed = ''.join(str(x) for x in observed)
   		expected = ''.join(str(x) for x in expected)
   		explauto_env.capture_state(expected,observed)
	   	print('Expected observation: ',expected)
		print('Actual observation:   ',observed)

		#raw_input("Press Enter to continue...")
		print('')
	else:
		print 'Failed attempt'

