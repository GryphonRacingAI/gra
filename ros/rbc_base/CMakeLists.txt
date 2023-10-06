cmake_minimum_required(VERSION 3.0.2)
project(rbc_base)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation controller_manager hardware_interface controller_interface geometry_msgs pluginlib tf2_ros tf2_geometry_msgs sensor_msgs)
find_package(Eigen3 REQUIRED)

add_message_files(
    FILES
    MotorAdaptiveState.msg
    MotorParameters.msg
    MotorSetpoint.msg
    MotorState.msg
    BaseAdaptiveState.msg
    BaseParameters.msg
    BaseSetpoint.msg
    BaseState.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_executable(hardware_parameter_server src/control/hardware_parameter_server/main.cpp)
target_link_libraries(hardware_parameter_server ${catkin_LIBRARIES})

add_executable(robot_hw src/control/hardware/main.cpp)
target_link_libraries(robot_hw ${catkin_LIBRARIES})

add_library(omniwheel_base_controller 
  src/control/omniwheel_base_controller/omniwheel_base_controller.cpp
)
target_link_libraries(omniwheel_base_controller ${catkin_LIBRARIES})
add_dependencies(omniwheel_base_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.perspective"
)

install (DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

install (DIRECTORY description/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/description
)

install(FILES src/control/omniwheel_base_controller/omniwheel_base_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)