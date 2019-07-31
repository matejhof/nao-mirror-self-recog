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
from nao_explauto_env import NaoEnvironment
from explauto import SensorimotorModel
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
for i in range(total_sensors):
    skin_map[str(i) + '_' + str(i+2)] = i

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
explauto_env = NaoEnvironment(joint_min, joint_max, [], [], joints, ros_topic, ee_link_name, use_reset, total_sensors, skin_map, collision_target_link)

# Explauto model
model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'exact')

# Reset output data file
fh = open('output/data.txt', 'w')
fh.close()

# Initialization complete
explauto_env.compute_sensori_effect(zero_pose)
print("NAO Skin ML: Initialization complete")

if load_model:
    # Load saved model from disk
    filename = 'models/model.sav'
    fh = open(filename, 'rb')
    model = pickle.load(fh)
    fh.close()
else:
    # Random poses around home pose
    i = 0
    s_total = [0 for j in range(total_sensors)]
    while (not rospy.is_shutdown()) and (i < max_learning_iterations):
        m = nao_ml_get_random_goal(home_pose, .6)
        s = explauto_env.compute_sensori_effect(m)
        # print(m, s)
        model.update(m, s)
        i = i + 1
        print('Motor command: ', i, [round(x, 2) for x in m])
        print('Observation: ' + ''.join(str(x) for x in s))
        #raw_input("Press Enter to continue...")
        print('')
        s_total = list(map(add, s_total, s))

        # Every $save_every_n$ iterations:
        if (i < 100 and i % 10 == 0) or (i % 100 == 0):
            # Save model to disk
            if save_model:
                filename = 'models/model-' + str(i) + '.sav'
                fh = open(filename, 'wb')
                pickle.dump(model, fh)
                fh.close()
                
            # Dump some data to output file
            fh = open('output/data.txt', 'a')
            fh.write(str(i) + ' ' + str(s_total) + '\n')
            fh.close()

    print('')
    print('Total number each taxel was hit:')
    print(' '.join(str(x) for x in s_total))
    print('')
    
    '''
    # Explauto random motors
    random_motors = explauto_env.random_motors(n = 10)
    for m in random_motors:
        # Compute the sensori effect s of the motor command m through the environment
        s = explauto_env.compute_sensori_effect(m)
        print(s)
        #model.update(m, s)
    '''

# Test learnt model
explauto_env.use_reset = True
for i in range(total_sensors):
    s_g = [0 for j in range(total_sensors)]
    s_g[i] = 1000000
    m = model.inverse_prediction(s_g)
    s = explauto_env.compute_sensori_effect(m)
    s_g[i] = 1
    print('Expected observation: ' + ''.join(str(x) for x in s_g))
    print('Actual observation:   ' + ''.join(str(x) for x in s))
    if s[i] == 1:
        print('OK!')
    else:
        print('Bad :(')
    raw_input("Press Enter to continue...")
    print('')

