# This version places all collision elements as direct children of casing element
# Coordinates are computed here in the script using RPY angles and rotation matrices
# Status: working

import numpy as np
from placeSensors import *

def rotz(x):
    c = np.cos(x)
    s = np.sin(x)
    R = np.array(((c, -s, 0), (s, c, 0), (0, 0, 1)))
    return R
def roty(x):
    c = np.cos(x)
    s = np.sin(x)
    R = np.array(((c, 0, s), (0, 1, 0), (-s, 0, c)))
    return R
def rotx(x):
    c = np.cos(x)
    s = np.sin(x)
    R = np.array(((1, 0, 0), (0, c, -s), (0, s, c)))
    return R
def rpy2rot(rpy):
    return np.matmul(np.matmul(rotz(rpy[2]), roty(rpy[1])), rotx(rpy[0]))

VERTICES = [
    "coordinates/torso.txt",
    "coordinates/head.txt",
    "coordinates/left-arm.txt",
    "coordinates/right-arm.txt",
]
SENSOR_LENGTH = "0.001"
SENSOR_RADIUS = "0.0022"
SENSOR_NAMES = [
    "skin_torso",
    "skin_head",
    "skin_l_wrist",
    "skin_r_wrist",
]
LINK_NAMES = [
    "torso_casing",
    "head_casing",
    "l_wrist_casing",
    "r_wrist_casing",
]
ARG_NAMES = [
    "contact_torso",
    "contact_head",
    "contact_left_arm",
    "contact_right_arm",
]
sensors_per_link = 10 
sensor_offsets = [
    [0, 0.006533, 0],
    [-0.00566, 0.0098, 0],
    [-0.00566, 0.003267, 0],
    [0, 0, 0],
    [-0.00566, -0.003267, 0],
    [-0.00566, -0.0098, 0],
    [0, -0.006533, 0],
    [0.00566, -0.003267, 0],
    [0.011317, 0, 0],
    [0.00566, 0.003627, 0],
]

with open('link-template-triangle-2.urdf.txt', 'r') as fid:
    linkTemplate = fid.read()

print('<?xml version="1.0" ?>')
print('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')

gazebo_str = '';
fh_map = open('_output-triangle-2-map.txt', 'w');

for i, file in enumerate(VERTICES):
    vertices = np.loadtxt(file)
    collisionStr = ""
    ind = 0
    print('    <xacro:macro name="insert_skin_' + LINK_NAMES[i] + '">')
    for j, vertex in enumerate(vertices):
        center = vertex[0:3]
        normal = vertex[3:6]
        rpy = calcRPY(np.array([0,0,1]), normal)
        
        for k in range(sensors_per_link):
            ind += 1
            name = LINK_NAMES[i] + '_collision_' + str(ind)
            yaw_correction = vertex[6]

            rot_matrix = rpy2rot(rpy)
            #print(rot_matrix)
            #print(np.matmul(rot_matrix, sensor_offsets[k]))
            #quit()
            center_k = np.add(center, np.matmul(rot_matrix, np.matmul(rotz(yaw_correction), sensor_offsets[k])))
            #fh_map.write(name + ' ');
            fh_map.write(str(center_k[0]) + ' ' + str(center_k[1]) + ' ' + str(center_k[2]) + ' ' + '\n')

            link_str = linkTemplate.replace("SENSOR_XYZ", str(center_k)[1:-1])
            link_str = link_str.replace("SENSOR_RPY", str(rpy)[1:-1])
            link_str = link_str.replace("SENSOR_LENGTH", str(SENSOR_LENGTH))
            link_str = link_str.replace("SENSOR_RADIUS", str(SENSOR_RADIUS))
            link_str = link_str.replace("COLLISION_NAME", name)
            link_str = link_str.replace("LINK_NAME", LINK_NAMES[i])
            link_str = link_str.replace("YAW_CORRECTION", str(yaw_correction))
            print(link_str)

            collisionStr = collisionStr + "                    <collision>" + name + "</collision>\n"

    print('    </xacro:macro>')
    
    gazebo_str = gazebo_str + '    <xacro:if value="$(arg ' + ARG_NAMES[i] + ')">\n'
    gazebo_str = gazebo_str + '        <gazebo reference="' + LINK_NAMES[i] + '">\n'
    gazebo_str = gazebo_str + '            <sensor name="' + LINK_NAMES[i] + '_collision_sensor" type="contact">\n'
    gazebo_str = gazebo_str + '                <plugin name="contact_plugin_' + LINK_NAMES[i] + '" filename="/home/user/catkin_ws/src/gazebo_ContactSensor_plugin/build/devel/lib/libcontact.so" />\n'
    gazebo_str = gazebo_str + '                <always_on>1</always_on>\n'
    gazebo_str = gazebo_str + '                <visualize>true</visualize>\n'
    gazebo_str = gazebo_str + '                <update_rate>100</update_rate>\n'
    gazebo_str = gazebo_str + '                <contact>\n'
    gazebo_str = gazebo_str + collisionStr
    gazebo_str = gazebo_str + '                </contact>\n'
    gazebo_str = gazebo_str + '            </sensor>\n'
    gazebo_str = gazebo_str + '        </gazebo>\n'
    gazebo_str = gazebo_str + '    </xacro:if>\n'

    fh_map.write('\n')

print(gazebo_str)
print('</robot>')

fh_map.close()
