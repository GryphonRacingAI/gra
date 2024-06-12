cmake_minimum_required(VERSION 3.0.2)
project(fsai_mission)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  ultralytics_ros
  fsai_api 
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fsai_mission
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy std_msgs ultralytics_ros fsai_api  
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/mission_supervisor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
