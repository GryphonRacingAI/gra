cmake_minimum_required(VERSION 3.0.2)
project(rbc_navigation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation geometry_msgs tf2_ros tf2_geometry_msgs)

# add_message_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.perspective"
)