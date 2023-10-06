#!/usr/bin/env bash
# This is the environment that the remote ROS nodes will run in on the remote host
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_HOSTNAME=$(hostname).local
export ROS_MASTER_URI=http://$ROS_HOSTNAME:11311
exec "$@"