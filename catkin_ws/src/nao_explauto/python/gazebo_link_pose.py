import rospy
from gazebo_msgs.msg import LinkStates
import numpy as np

class GazeboLinkPose:
    link_name = ''
    subscriber = None
    pose = None
    prev_pose = None
    steady_epsilon = 5e-3

    def __init__(self, link_name):
        self.link_name = link_name
        self.subscriber = rospy.Subscriber("/gazebo/link_states", LinkStates, self.callback)

    def callback(self, data):
        try:
            ind = data.name.index(self.link_name)
            self.pose = data.pose[ind]
        except ValueError:
            pass

    def reset_stable(self):
        self.prev_pose = None

    def is_stable(self):
        d = [1, 1, 1]
        if self.pose and self.prev_pose:
            d = [self.pose.position.x - self.prev_pose.position.x, self.pose.position.y - self.prev_pose.position.y, self.pose.position.z - self.prev_pose.position.z]
            # print(np.linalg.norm(d))
        self.prev_pose = self.pose
        return np.linalg.norm(d) <= self.steady_epsilon

