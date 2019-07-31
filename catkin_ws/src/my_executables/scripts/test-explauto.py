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
from explauto import SensorimotorModel
from explauto import InterestModel
from explauto.interest_model.random import RandomInterest
from explauto.sensorimotor_model import sensorimotor_models, available_configurations
from explauto.sensorimotor_model.non_parametric import NonParametric
from explauto.sensorimotor_model.inverse.cma import fmin as cma_fmin

# If True, just test saved model
load_model = False

# If true, save learnt model to disk
save_model = True

# Defines if we want to reset simulation when applying motor commands (Gazebo)
use_reset = True

# Max iterations of the learning process
max_learning_iterations_1 = 10
max_learning_iterations_2 = 50

cma_maxfevals = 50 # Maximum error function evaluations by CMAES (actually CMAES will slightly overshoot it)
cma_sigma0 = 0.2 # Standard deviation in initial covariance matrix

# Size of testing grid
test_grid_size = 10

# Degrees of Freedom
DOF = 5

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
skin_map = {
    # '1': None,
    '0_2':   [-0.0482040146921, -0.00116878361421],
    '1_3':   [-0.0166540146921, -0.00116878361421],
    '2_4':   [0.0166579955079, -0.00116878361421],
    '3_5':   [0.0482079955079, -0.00116878361421],
    '4_6':   [-0.0605171067051, 0.00845121638579],
    '5_7':   [-0.0331171067051, 0.00845121638579],
    '6_8':   [1.7763568394e-17, 0.00845121638579],
    '7_9':   [0.033121087521, 0.00845121638579],
    '8_10':  [0.060521087521, 0.00845121638579],
    '9_11':  [-0.0615880962161, 0.0278212163858],
    '10_12': [-0.0332080962161, 0.0278212163858],
    '11_13': [1.42108547152e-17, 0.0278212163858],
    '12_14': [0.033212077032, 0.0278212163858],
    '13_15': [0.061592077032, 0.0278212163858],
    '14_16': [-0.0489980095921, 0.0375112163858],
    '15_17': [-0.0166780095921, 0.0375112163858],
    '16_18': [0.0166819904079, 0.0375112163858],
    '17_19': [0.0490019904079, 0.0375112163858],
    '18_20': [-0.0489980095921, 0.0568712163858],
    '19_21': [-0.0166780095921, 0.0568712163858],
    '20_22': [0.0166819904079, 0.0568712163858],
    '21_23': [0.0490019904079, 0.0568712163858],
    '22_24': [-0.033011138495, 0.0665612163858],
    '23_25': [0.0, 0.0665612163858],
    '24_26': [0.0330151193109, 0.0665612163858]
}

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

min_x = min([skin_map[k][0] for k in skin_map])
max_x = max([skin_map[k][0] for k in skin_map])
min_y = min([skin_map[k][1] for k in skin_map])
max_y = max([skin_map[k][1] for k in skin_map])
dx = (max_x - min_x) / 2
dy = (max_y - min_y) / 2
s_mins = [min_x - dx, min_y - dy]
s_maxs = [max_x + dx, max_y + dy]

explauto_env = NaoEnvironment(joint_min, joint_max, s_mins, s_maxs, joints, ros_topic, ee_link_name, use_reset, skin_map, collision_target_link)

# Surrogate sensorimotor model
model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'default')
model.mode = "exploit" # We don't want the sensorimotor model to add exploration noise
im_model = RandomInterest(explauto_env.conf, explauto_env.conf.s_dims)
#model = InterestModel.from_configuration(explauto_env.conf, explauto_env.conf.s_dims, 'random')

# Initialization complete
# explauto_env.compute_sensori_effect(zero_pose)
print("NAO Skin ML: Initialization complete")

rospy.spin()

