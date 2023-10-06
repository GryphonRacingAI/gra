# Where things are located on the system (relative paths should mean it works on the container and the Pi)
export CATKIN_WS_PATH=~/catkin_ws
export RBC_REPO=~/rbc
export DOCKERFILE=$RBC_REPO/.devcontainer/Dockerfile

# Default ROS environment variables (could be overriden by setdevmaster / setbotmaster)
export DEV_MASTER_URI="http://localhost:11311"
export DEV_BOT_HOSTNAME="rbc.local"
export DEV_BOT_USER="ubuntu"
export DEV_BOT_MASTER_URI="http://$DEV_BOT_HOSTNAME:11311"
if [ -z "$DEV_ENV" ]; then # Docker environment would define DEV_ENV to 1, so if it's not defined, we assume we are running on the robot
    export DEV_ENV=0
    export ROS_MASTER_URI=$DEV_BOT_MASTER_URI
    export ROS_HOSTNAME=$DEV_BOT_HOSTNAME    
    export RBC_MASTER="bot"
fi
if [ -z "$ROS_DISTRO" ]; then # If ROS_DISTRO is not defined, we assume we are running on the robot
    ROS_DISTRO=noetic
fi

# Arduino environment variables
export ARDUINO_UPDATE_PORT="/dev/ttyACM0"
export ARDUINO_COMMUNICATION_PORT="/dev/ttyS0"

# Persistent variables
# If ~/.penv/persistent_vars.sh does not exist, create it
if [ ! -f ~/.penv/persistent_vars.sh ]; then
    mkdir -p ~/.penv
    touch ~/.penv/persistent_vars.sh
fi
export PERSISTENT_FILE=$(realpath ~/.penv/persistent_vars.sh)