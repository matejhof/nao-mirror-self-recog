This is description of data format in output.txt

At the end of movement of hand towards robot's head I recorded following parameters: joint angles, expected sensory output, observed sensory output, mirrored image of robot's torso,head and arm.
Each line is formated in following style:

[RElbowRoll,RElbowYaw,RShoulderPitch,RShoulderRoll,RWristYaw,HeadYaw_callback,HeadPitch]	expected_output	observed_output	pictures/picture_name.png

Every item is seperated by tab.

Joint angles are in radians.

Format of expected and observed output is following. There are 240 collision sensors on Nao's head and these outputs are strings of 240 characters. '0' for non-activated sensor and '1' for activated collision sensor. Activated sensors are always going to be in tuples of TWO to generealize the area of touch. Example of such an output is: 001100...00000000000000000

Pictures are stored in subfolder pictures.

