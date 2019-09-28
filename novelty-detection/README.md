Tutorial for working with Nao in gazebo 9 and ROS melodic for self-recognision in mirror. For additional info you can emai me to outravoj@fel.cvut.cz
------
------
After installing required software (Gazebo9 ROS melodic, SDK simulator and Naoqi python SDK (softbankrobotics - Matej can give you username and password) along with [Boost1.55.0](https://stackoverflow.com/questions/38705150/naoqi-library-to-python-sdk-in-ubuntu), clone this repository to your computer.

1. Install catkin tools - [this site](https://catkin-tools.readthedocs.io/en/latest/installing.html)
2. Install explauto library - library for controlling nao with learned inverse models (will not build without it due to dependencies) - [this site](http://flowersteam.github.io/explauto/installation.html) 
xxx

**changes to nao_skin.xacro** - For both versions of nao (highres and lowres) go to nao_skin.xacro and all the way down to specification of controllers and plugins and in <plugin name="contact_plugin_torso_casing" filename="/home/vojta/code-nao-simulation/gazebo9/catkin_ws/devel/lib/libcontact.so" /> change filename to path to library libcontact.so in your computer. Without this change the artificial skin will not work.

After these steps Gazebo Gui with spawned Nao should pop up, simulation should be paused and Nao will have only Torso, arms and head with camera for mirroring right infront of him (white box).

# Changes to Nao
xxx

# Versions of Nao's skin
1. High resulotin - launching through aforementioned command "bash launch-naoqi-highres.sh"
	Description in .xacro files in catkin_ws/src/nao_robot/nao_description/urdf/naoSkin2_generated_urdf/
	
2. Low resolution -launching through command "bash launch-naoqi-lowres.sh"
	Description in .xacro files in catkin_ws/src/nao_robot/nao_description/urdf/naoSkin_generated_urdf/

xxx **nao_control_position.launch** and are **not included** in **nao_control_position_skin.launch**. 

# Controlling Nao during the simulation
xxx











