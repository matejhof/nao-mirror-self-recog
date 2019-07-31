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
from nao_explauto_env_2 import NaoEnvironment
from explauto import SensorimotorModel, InterestModel
from explauto.sensorimotor_model import sensorimotor_models, available_configurations
from explauto.sensorimotor_model.non_parametric import NonParametric

# If True, just test saved model
load_model = False

# If true, save learnt model to disk
save_model = True

# Defines if we want to reset simulation when applying motor commands (Gazebo)
use_reset = True

# Max iterations of the learning process
max_learning_iterations = 1000

# Size of testing grid
test_grid_size = 10

# Number of sensors
total_sensors = 25

# End effector link name
ee_link_name = 'nao_skin::r_wrist'

# ROS topic name(s) for Gazebo contacts
ros_topic = '/gazebo_contact_info/r_wrist'

# Target link on which collisions are detected
collision_target_link = 'base_link'

# Home pose for the exploration
home_pose = [1.48, -0.45, 0, -0.1, np.pi/2]
zero_pose = [0, 0, 0, 0, 0]

# Joint data structure
joints = [
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
]

# Skin to coordinates mapping
skin_map = {}
map_fh = open('coordinate-maps/lowres-torso.txt', 'r')
for i in range(total_sensors):
    k = str(i) + '_' + str(i + 2)
    skin_map[k] = [float(x) for x in map_fh.readline().strip().split(" ")]
map_fh.close()

# Initialize ROS
rospy.init_node('nao_skin_ml', anonymous=True)
srv_joint_state = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)

# Initialize joints
for joint in joints:
    topic_name = '/nao_dcm/' + joint['joint_name'] + '_position_controller/command';
    joint['publisher'] = rospy.Publisher(topic_name, Float64, queue_size=5)

# Initialize explauto environment
joint_min = [x['limit_min'] for x in joints]
joint_max = [x['limit_max'] for x in joints]

# Set bounds of sensory space
min_x = min([skin_map[k][0] for k in skin_map])
max_x = max([skin_map[k][0] for k in skin_map])
min_y = min([skin_map[k][1] for k in skin_map])
max_y = max([skin_map[k][1] for k in skin_map])
dx = (max_x - min_x)
dy = (max_y - min_y)
s_mins = [min_x - dx, min_y - dy]
s_maxs = [max_x + dx, max_y + dy]

explauto_env = NaoEnvironment(joint_min, joint_max, s_mins, s_maxs, joints, ros_topic, ee_link_name, use_reset, skin_map, collision_target_link)

# Instantiate a random goal interest model:
im_model = InterestModel.from_configuration(explauto_env.conf, explauto_env.conf.s_dims, 'random')

# Explauto model
model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'default')

# Initialization complete
explauto_env.compute_sensori_effect(zero_pose)
print("NAO Skin ML: Initialization complete")

i = 0
fh = open('output/goals.txt', 'w')
while (not rospy.is_shutdown()) and (i < max_learning_iterations):
    # Generate goal
    s_g = im_model.sample()
    i += 1
    _str = ' '.join(map(str, s_g))
    print(_str)
    fh.write(_str + '\n')
fh.close()

