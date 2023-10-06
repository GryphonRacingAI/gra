cmake_minimum_required(VERSION 3.0.2)
project(rbc_web_interface)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ executable
## With catkin_make all executables are created in the same CMakeLists.txt
## file, or in a CMakeLists.txt file in a subdirectory.

## Specify libraries to link a library or executable target against

# ## Web interface build command
# file(GLOB_RECURSE WEB_SOURCE_FILES 
#      ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/*.js
#      ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/*.ts
#      ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/*.tsx
#      ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/*.jsx)

# add_custom_command(
#   OUTPUT ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/dist
#   COMMAND npm install && npm run build
#   WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/path/to/your/web/interface
#   DEPENDS ${WEB_SOURCE_FILES}
# )

# add_custom_target(
#   my_web_interface ALL
#   DEPENDS ${CMAKE_SOURCE_DIR}/path/to/your/web/interface/dist
# )
