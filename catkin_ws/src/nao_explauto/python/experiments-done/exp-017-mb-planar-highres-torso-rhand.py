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
from explauto.sensorimotor_model import sensorimotor_models, available_configurations
from explauto.sensorimotor_model.non_parametric import NonParametric

# If True, just test saved model
load_model = False

# If true, save learnt model to disk
save_model = True

# Defines if we want to reset simulation when applying motor commands (Gazebo)
use_reset = True

# Max iterations of the learning process
max_learning_iterations = 500

# Testing grid (torso; taxel indices)
test_grid = []
map_tg = open('coordinate-maps/test-grid-highres-torso.txt', 'r')
_input = map_tg.readline().strip().split(" ")
test_grid = [int(x) for x in _input]
map_tg.close()

# Number of sensors
total_sensors = 250

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
skin_map_coords = []
map_fh = open('coordinate-maps/highres-torso.txt', 'r')
for i in range(total_sensors):
    k = str(i + 1) + '_' + str(i + 2)
    _input = map_fh.readline().strip().split(" ")
    skin_map[k] = [float(x) for x in _input]
    skin_map_coords.append([float(x) for x in _input])
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

# For highres skin
explauto_env.max_time_delay = 10.0
explauto_env.time_delay_after = 5.0

# Explauto model
model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'exact')
#params = available_configurations('WNN')['default']
#params['k'] = 5
#model = NonParametric(explauto_env.conf, **params)

# Initialization complete
explauto_env.compute_sensori_effect(zero_pose)
explauto_env.compute_sensori_effect(zero_pose)
print("NAO Skin ML: Initialization complete")

# Reached taxels - for manual progress evaluation
reached_taxels = []

if load_model:
    # Load saved model from disk
    filename = 'models/model-' + str(i) + '.sav'
    fh = open(filename, 'rb')
    model = pickle.load(fh)
    fh.close()
if True:
    # Motor babbling
    i = 0
    while (not rospy.is_shutdown()) and (i < max_learning_iterations):
        m = nao_ml_get_random_goal(home_pose, 1)
        s = explauto_env.compute_sensori_effect(m)
        # print(m, s)
        i = i + 1

        if s is None:
            print('Motor command: ', i, [round(x, 2) for x in m])
            print('Observation: None')
            print('')
            pass
        else:
            model.update(m, s)
            reached_taxels.append(s)
            print('Motor command: ', i, [round(x, 2) for x in m])
            print('Observation: ' + ','.join(str(x) for x in s))
            print('')

        # Every $save_every_n$ iterations:
        if (i < 100 and i % 10 == 0) or (i % 100 == 0):
            if save_model:
                filename = 'models/model-' + str(i) + '.sav'
                fh = open(filename, 'wb')
                pickle.dump(model, fh)
                fh.close()

        # Test learnt model - grid
        if (i % 10 == 0):
            filename = 'output/data-' + str(i) + '.txt'
            fh = open(filename, 'w')
            print('Test learnt model')
            for ii in test_grid:
                s_g = skin_map_coords[ii-1]
                s = None
                best_dist = float('Inf')
                for jj in range(len(reached_taxels)):
                    s_t = reached_taxels[jj]
                    dist = ((s_g[0] - s_t[0])**2 + (s_g[1] - s_t[1])**2)**0.5
                    if dist < best_dist:
                        s = s_t[:]
                        best_dist = dist
                print('Expected observation: ' + ','.join(str(x) for x in s_g))
                if s is None:
                    print('Actual observation:   None')
                    fh.write(str(s_g[0]) + ' ' + str(s_g[1]) + ' ' + str(9999) + ' ' + str(9999) + ' ' + str(9999) + '\n')
                else:
                    print('Actual observation:   ' + ','.join(str(x) for x in s))
                    print('Distance:   ' + str(best_dist))
                    fh.write(str(s_g[0]) + ' ' + str(s_g[1]) + ' ' + str(s[0]) + ' ' + str(s[1]) + ' ' + str(best_dist) + '\n')
                print('')
            fh.close()

