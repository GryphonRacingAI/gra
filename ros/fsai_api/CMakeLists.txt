cmake_minimum_required(VERSION 3.0.2)
project(fsai_api)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ackermann_msgs
  rospy
  std_msgs
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES fs_ai_api
  CATKIN_DEPENDS roscpp ackermann_msgs rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  VCU2AI.msg	
)


# Copy the FS-AI_API source files into the current build directory
set(FS_AI_API_DIR "${PROJECT_SOURCE_DIR}/FS-AI_API")

configure_file(${FS_AI_API_DIR}/FS-AI_API/fs-ai_api.c ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.c COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/can.c ${CMAKE_CURRENT_BINARY_DIR}/can.c COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/fs-ai_api.h ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.h COPYONLY)
configure_file(${FS_AI_API_DIR}/FS-AI_API/can.h ${CMAKE_CURRENT_BINARY_DIR}/can.h COPYONLY)

# Add the FS-AI_API sources to the build
add_library(fs_ai_api STATIC ${CMAKE_CURRENT_BINARY_DIR}/fs-ai_api.c ${CMAKE_CURRENT_BINARY_DIR}/can.c)
target_include_directories(fs_ai_api PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Add the ackermann_can node
add_executable(ackermann_can src/ackermann_can.cpp)
add_dependencies(ackermann_can ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ackermann_can
  fs_ai_api
  ${catkin_LIBRARIES}
)

# Add the static inspection node
add_executable(static_inspection_A src/static_inspection_A.cpp)
add_dependencies(static_inspection_A ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(static_inspection_A
  fs_ai_api
  ${catkin_LIBRARIES}
)
