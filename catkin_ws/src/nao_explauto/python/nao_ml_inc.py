from gazebo_msgs.srv import GetJointProperties, GetJointPropertiesResponse
from random import random
from copy import copy
import rospy


def nao_ml_set_pose(pose, joints):
    for i in range(len(pose)):
        joints[i]['publisher'].publish(pose[i])

def nao_ml_get_joint_state(srv, joints):
    joint_states = []
    for joint in joints:
        joint_full_name = 'nao_skin::' + joint['joint_name']
        state = srv(joint_full_name)
        joint_states.append({"joint_name": joint['joint_name'], "state": state})
    return joint_states

def nao_ml_get_random_goal(home_pose, exploration_step):
    pose = copy(home_pose)
    for i in range(len(pose)):
        pose[i] += (random() - 0.5) * exploration_step
    return pose

