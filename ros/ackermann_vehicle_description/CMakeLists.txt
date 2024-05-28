cmake_minimum_required(VERSION 2.8.3)
project(ackermann_vehicle_description)

# Find required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

# Add message files
add_message_files(
  FILES
  AckermannFeedback.msg
)

# Generate added messages and services
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

# Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Mark executable scripts for installation
catkin_install_python(PROGRAMS
  scripts/odom_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other directories
install(DIRECTORY launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
