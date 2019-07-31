# Currently there are two versions of modified NAO:
# Skin  - lowres artificial skin simulation (default)
# Skin2 - highres artificial skin simulation
NAO_VERSION='Skin'

# Contact plugin loader accepts either true (default) or false
# Use this to disable contact plugin on body parts that are unused in particular
# experiment with high resolution skin configuration
# These can be set with -t, -h, -l, -r
CONTACT_TORSO='true'
CONTACT_HEAD='true'
CONTACT_LEFT_ARM='true'
CONTACT_RIGHT_ARM='true'
CONTACT_LEFT_LEG='true'
CONTACT_RIGHT_LEG='true'

while [[ $# -gt 0 ]]
do
key="$1"
case $key in
    -v|--version)
    NAO_VERSION="$2"
    shift # past argument
    shift # past value
    ;;
    -t|--torso)
    CONTACT_TORSO="$2"
    shift # past argument
    shift # past value
    ;;
    -h|--head)
    CONTACT_HEAD="$2"
    shift # past argument
    shift # past value
    ;;
    -l|--left_arm)
    CONTACT_LEFT_ARM="$2"
    shift # past argument
    shift # past value
    ;;
    -r|--right_arm)
    CONTACT_RIGHT_ARM="$2"
    shift # past argument
    shift # past value
    ;;
	-r|--right_leg)
    CONTACT_RIGHT_LEG="$2"
    shift # past argument
    shift # past value
    ;;
    -l|--left_leg)
    CONTACT_LEFT_LEG="$2"
    shift # past argument
    shift # past value
    ;;
    *)    # unknown option
    shift # past argument
    ;;
esac
done

reset
roslaunch nao_gazebo_plugin nao_gazebo_skin.launch disable_controller:=false nao_version:=$NAO_VERSION contact_torso:=$CONTACT_TORSO contact_head:=$CONTACT_HEAD contact_left_arm:=$CONTACT_LEFT_ARM contact_right_arm:=$CONTACT_RIGHT_ARM contact_right_leg:=$CONTACT_RIGHT_ARM contact_left_leg:=$CONTACT_RIGHT_ARM

#rosrun rqt_ez_publisher rqt_ez_publisher --force-discover
#roslaunch nao_gazebo_plugin nao_gazebo.launch
