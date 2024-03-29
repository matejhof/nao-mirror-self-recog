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
total_sensors = 24

# End effector link name
ee_link_name = 'nao_skin::r_wrist'

# ROS topic name(s) for Gazebo contacts
ros_topic = '/gazebo_contact_info/r_wrist'

# Target link on which collisions are detected
collision_target_link = 'Head'

# Home pose for the exploration
home_pose = [1.117, 0.083, -0.87, -0.02, 1.638, -0.21, -0.12]
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
map_fh = open('/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/coordinate-maps/lowres-head.txt', 'r')
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
print('Ros public topics: ')

topics = rospy.get_published_topics()
for i in topics:
	print(i)
# Bootstrap model
print('Bootstrapping model...')
s = explauto_env.compute_sensori_effect(home_pose)
for i in range(90):
    m = nao_ml_get_random_goal(home_pose, 1)
    s = explauto_env.compute_sensori_effect(m)
    if s is not None:
        model.update(m, s)
     	print('Model updating in process');
print('Bootstrapping done. Model size = ' + str(model.size()))


if load_model:
    # Load saved model from disk
    filename = '/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/models/explauto-saved-models/exp-011/model-500.sav'
    fh = open(filename, 'rb')
    model = pickle.load(fh)
    fh.close()
else:
    # Motor babbling
    i = 90
    while (not rospy.is_shutdown()) and (i < max_learning_iterations):
        # Generate goal
        s_g = im_model.sample()
        
        # Infer a motor command to reach that goal using the sensorimotor model:
        m = model.inverse_prediction(s_g)

        # Execute this command and observe the corresponding sensory effect:
        s = explauto_env.compute_sensori_effect(m)
        print(m, s)
        i = i + 1

        print('Generated goal: ' + ','.join(str(x) for x in s_g))
        if s is None:
            print('Motor command: ', i, [round(x, 2) for x in m])
            print('Observation: None')
            print('')
            pass
        else:
            model.update(m, s)
            print('Motor command: ', i, [round(x, 2) for x in m])
            print('Observation: ' + ','.join(str(x) for x in s))
            print('')

        # print('Raw sensory effect: ' + explauto_env.sensory_effect_str)
        # print('Dictionary key: ' + explauto_env.sensory_effect_dict_key)
        # raw_input("Press Enter to continue...")

        # Every $save_every_n$ iterations:
        if (i < 100 and i % 10 == 0) or (i % 100 == 0):
            if save_model:
                filename = '/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/models/model-' + str(i) + '.sav'
                fh = open(filename, 'wb')
                pickle.dump(model, fh)
                fh.close()

        # Test learnt model - 10x10 grid
        if (i % 100 == 0):
            explauto_env.use_reset = True
            model.mode = "exploit"
            filename = '/home/vojta/code-nao-simulation/gazebo9/catkin_ws/src/my_executables/scripts/models/output/data-' + str(i) + '.txt'
            fh = open(filename, 'w')
            print('Test learnt model')
            for ii in range(test_grid_size):
                for jj in range(test_grid_size):
                    x = min_x + (max_x - min_x) / (test_grid_size - 1) * ii
                    y = min_y + (max_y - min_y) / (test_grid_size - 1) * jj
                    s_g = [x,y]
                    s = None
                    best_dist = float("inf")
                    # Best of K=2, just to be sure
                    for kk in range(2):
                        m = model.inverse_prediction(s_g)
                        s_t = explauto_env.compute_sensori_effect(m)
                        if s_t is not None:
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
                    # raw_input("Press Enter to continue...")
                    print('')
            fh.close()
            explauto_env.use_reset = use_reset
            model.mode = "explore"

