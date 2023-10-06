cmake_minimum_required(VERSION 3.0.2)
project(rbc_manager)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation)

# add_message_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )

# Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime smach smach_ros smach_msgs
)