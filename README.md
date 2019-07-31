Tutorial for working with Nao in gazebo 9 and ROS melodic for self-recognision in mirror
------
------
After installing required software (Gazebo9 ROS melodic, SDK simulator and Naoqi python SDK (softbankrobotics - Matej can give you username and password) along with [Boost1.55.0](https://stackoverflow.com/questions/38705150/naoqi-library-to-python-sdk-in-ubuntu) -  clone this repo. in your home directory.

1. Install catkin tools - [this site](https://catkin-tools.readthedocs.io/en/latest/installing.html)
2. Install explauto library - library for controlling nao (will not build without it due to dependencies) - [this site](http://flowersteam.github.io/explauto/installation.html) ,
3. go to catkin_ws
4. Build workspace - catkin build
5. Source - needs to be done whenever you open new terminal for launching gazebo and ROS or you can add it to ~/.bashrc once:
	source /opt/ros/melodic/setup.sh
	source ~/code-nao-simulation/gazebo9/catkin_ws/devel/setup.bash
6. Go to misc
7. Launch gazebo world with Nao in it: bash launch-naoqi-highres.sh

After these steps Gazebo Gui with spawned Nao should pop up, simulation should be paused and Nao will have only Torso, arms and head with camera for mirroring right infront of him (white box).

# Changes to Nao
1. Removed fingers because of simulation issues
2. Legs removed - for our purposes were redundant
3. Added camera for creating a 'mirror'
4. Changed color of casing
5. Fixed torso in space so Nao doesn't fall to the ground and remains stable

# Versions of Nao's skin
1. High resulotin - launching through aforementioned command "bash launch-naoqi-highres.sh"
	Description in .xacro files in catkin_ws/src/nao_robot/nao_description/urdf/naoSkin2_generated_urdf/
	
2. Low resolution -launching through command "bash launch-naoqi-lowres.sh"
	Description in .xacro files in catkin_ws/src/nao_robot/nao_description/urdf/naoSkin_generated_urdf/

Changes to Nao can be done throughout these files, mainly in nao_robot.xacro, for example legs can be added by uncommenting include for nao_legs.xacro along with uncommenting legs transmission in naoTransmission.xacro, also look into launch files if controllers for legs are included.

Controllers for legs are included in **nao_control_position.launch** and are **not included** in **nao_control_position_skin.launch**. 

# Controlling Nao during the simulation
This is done through python scripts, one of the simplest examples catkin_ws/src/my_executables/scripts/camera_listener.py. Once the simulation is runnning, unpause the simulation, open new terminal and run command: "rosrun my_executables camera_listener.py"   

Sensor data and joints commands are communicated through ROS topics that a node can subscribe to (for data) or publish to (for controlling the robot). List of ros topics can be displayed by using command in terminal "rostopic list". Any Topic that is listed can be published and subscribed to.

More complex example of manipulation of robot is head_babbling.py in the same folder.











