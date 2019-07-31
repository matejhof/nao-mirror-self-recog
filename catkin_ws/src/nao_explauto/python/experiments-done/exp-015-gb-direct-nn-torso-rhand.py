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
max_learning_iterations = 1000

cma_maxfevals = 10 # Maximum error function evaluations by CMAES (actually CMAES will slightly overshoot it)
cma_sigma0 = 0.2 # Standard deviation in initial covariance matrix

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

# Surrogate sensorimotor model
model = SensorimotorModel.from_configuration(explauto_env.conf, 'nearest_neighbor', 'default')
model.mode = "exploit" # We don't want the sensorimotor model to add exploration noise
im_model = RandomInterest(explauto_env.conf, explauto_env.conf.s_dims)
#model = InterestModel.from_configuration(explauto_env.conf, explauto_env.conf.s_dims, 'random')

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
    # Bootstrap surrogate model with motor babbling
    i = 0
    print("[Bootstrap surrogate sensorimotor model]")
    while (not rospy.is_shutdown()) and (i < 90):
        m = nao_ml_get_random_goal(home_pose, 1)
        #m = explauto_env.random_motors(1)[0]
        if (m < joint_min) or (m > joint_max):
            continue
        s = explauto_env.compute_sensori_effect(m)

        i = i + 1

        if not (s is None):
            model.update(m, s)
            print('Motor command: ', i, [round(x, 2) for x in m])
            print('Observation: ' + ','.join(str(x) for x in s))
            print('')

    # Goal Babbling with direct optimization
    print("[Start random goal babbling]")
    i = 90
    while (not rospy.is_shutdown()) and (i < max_learning_iterations):
        # Sample a random goal
        s_g = im_model.sample()
        
        # Find the nearest neighbor of s_g and output the corresponding m
        m0 = model.inverse_prediction(s_g)
        if (m0 < joint_min).any() or (m0 > joint_max).any():
            # print m0
            # print joint_min
            # print joint_max
            print("Model prediction out of joint range")
            continue

        # Error function corresponding to the new goal s_g
        def error_f(m_):
            s_ = explauto_env.compute_sensori_effect(m_)
            if not (s_ is None):
                # Update the surrogate model
                model.update(m_, s_)
                return np.linalg.norm(s_ - s_g) # Output the distance between the reached point s_ and the goal s_g
            else:
                return float('Inf')

        # Call CMAES with the error function for the new goal and use m0 to bootstrap exploration
        m = cma_fmin(error_f, m0, cma_sigma0, options={
            'bounds':[explauto_env.conf.m_mins, explauto_env.conf.m_maxs],
            'verb_log':0, # don't flood my output...
            'verb_disp':False, # ...seriously
            'maxfevals':cma_maxfevals
        })[0]

        i = i + 1

        if m is None:
            print("No motor command was found")
        else:
            s = explauto_env.compute_sensori_effect(m) # Execute best motor command found by CMAES (optional)
            if not (s is None):
                # Update the surrogate model
                model.update(m, s)
                print "i:", i, "Goal:", s_g, "Reaching Error:", np.linalg.norm(s_g - s)

        # Every $save_every_n$ iterations:
        if (i < 100 and i % 10 == 0) or (i % 100 == 0):
            if save_model:
                filename = 'models/model-' + str(i) + '.sav'
                fh = open(filename, 'wb')
                pickle.dump(model, fh)
                fh.close()

        # Test learnt model - 10x10 grid
        if (i % 100 == 0):
            explauto_env.use_reset = True
            model.mode = "exploit"
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

