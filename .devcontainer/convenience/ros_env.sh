depends_var ROS_DISTRO
depends_var CATKIN_WS_PATH

# Source ROS underlay
source /opt/ros/$ROS_DISTRO/setup.bash
# Source overlay IF it exists
if [ -f $CATKIN_WS_PATH/devel/setup.bash ]; then
  source $CATKIN_WS_PATH/devel/setup.bash
fi