import numpy as np
from placeSensors import *

VERTICES = [
    "coordinates/torso.txt",
    "coordinates/head.txt",
    "coordinates/left-arm.txt",
    "coordinates/right-arm.txt",
]
SENSOR_SIZE = "0.015 0.015 0.001"
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

with open('link-template-square.urdf.txt', 'r') as fid:
    linkTemplate = fid.read()

print('<?xml version="1.0" ?>')
print('<robot xmlns:xacro="http://www.ros.org/wiki/xacro">')

gazebo_str = '';

for i, file in enumerate(VERTICES):
    vertices = np.loadtxt(file)
    collisionStr = ""
    print('    <xacro:macro name="insert_skin_' + LINK_NAMES[i] + '">')
    for j, vertex in enumerate(vertices):
        name = LINK_NAMES[i] + '_collision_' + str(j)
        center = vertex[0:3]
        normal = vertex[3:6]
        rpy = calcRPY(np.array([0,0,1]), normal)
        print(linkTemplate.replace("SENSOR_XYZ", str(center)[1:-1]).replace("SENSOR_RPY", str(rpy)[1:-1]).replace("SENSOR_SIZE", str(SENSOR_SIZE)).replace("COLLISION_NAME", name))
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
print(gazebo_str)
print('</robot>')
