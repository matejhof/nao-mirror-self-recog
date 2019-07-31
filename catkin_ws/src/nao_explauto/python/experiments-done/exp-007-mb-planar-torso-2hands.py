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
max_learning_iterations = 1000

# Size of testing grid
test_grid_size = 10

# Number of sensors
total_sensors = 25

# End effector link name
ee_link_name1 = 'nao_skin::r_wrist'
ee_link_name2 = 'nao_skin::l_wrist'

# ROS topic name(s) for Gazebo contacts
ros_topic1 = '/gazebo_contact_info/r_wrist'
ros_topic2 = '/gazebo_contact_info/l_wrist'

# Target link on which collisions are detected
collision_target_link = 'base_link'

# Home pose for the exploration
home_pose1 = [1.48, -0.45, 0, -0.1, np.pi/2, 0, 0, 0, 0, 0]
home_pose2 = [0, 0, 0, 0, 0, -1.48, 0.45, 0, 0.1, -np.pi/2]
zero_pose = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

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

    {
        "joint_name": "LElbowRoll",
        "publisher": None,
        "limit_min": -1.54462,
        "limit_max": -0.0349066
    },
    {
        "joint_name": "LElbowYaw",
        "publisherg": None,
        "limit_min": -2.08567,
        "limit_max": 2.08567
    },
    {
        "joint_name": "LShoulderPitch",
        "publisher": None,
        "limit_min": -2.08567,
        "limit_max": 2.08567
    },
    {
        "joint_name": "LShoulderRoll",
        "publisher": None,
        "limit_min": -0.314159,
        "limit_max": 1.32645
    },
    {
        "joint_name": "LWristYaw",
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

explauto_env1 = NaoEnvironment(joint_min, joint_max, s_mins, s_maxs, joints, ros_topic1, ee_link_name1, use_reset, skin_map, collision_target_link)
explauto_env2 = NaoEnvironment(joint_min, joint_max, s_mins, s_maxs, joints, ros_topic2, ee_link_name2, False, skin_map, collision_target_link)

# Explauto model
model = SensorimotorModel.from_configuration(explauto_env1.conf, 'nearest_neighbor', 'exact')
#params = available_configurations('WNN')['default']
#params['k'] = 5
#model = NonParametric(explauto_env.conf, **params)

# Initialization complete
explauto_env1.compute_sensori_effect(zero_pose)
print("NAO Skin ML: Initialization complete")

if load_model:
    # Load saved model from disk
    filename = 'models/model.sav'
    fh = open(filename, 'rb')
    model = pickle.load(fh)
    fh.close()
else:
    # Motor babbling
    i = 0
    while (not rospy.is_shutdown()) and (i < max_learning_iterations):
        tmp = explauto_env1.compute_sensori_effect(zero_pose)
        # time.sleep(0.5)
        # Switch hands
        if i % 2 == 0:
            m = nao_ml_get_random_goal(home_pose1, 1)
            m[5:10] = [0] * 5
            s = explauto_env1.compute_sensori_effect(m)
        else:
            m = nao_ml_get_random_goal(home_pose2, 1)
            m[:5] = [0] * 5
            s = explauto_env1.compute_sensori_effect(m)
            s = explauto_env2.compute_sensori_effect(m)

        # print(m, s)
        i = i + 1

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
                filename = 'models/model-' + str(i) + '.sav'
                fh = open(filename, 'wb')
                pickle.dump(model, fh)
                fh.close()

        # Test learnt model - 10x10 grid
        if (i % 100 == 0):
            explauto_env1.use_reset = True
            filename = 'output/data-' + str(i) + '.txt'
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
                        s_t = explauto_env1.compute_sensori_effect(m)
                        if s_t is None:
                            s_t = explauto_env2.compute_sensori_effect(m)
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
            explauto_env1.use_reset = use_reset

