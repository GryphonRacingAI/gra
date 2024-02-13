# YOLOv4 ROS

## Overview

This is a ROS package for YOLOv4. It is modified from the repository https://github.com/leggedrobotics/darknet_ros/. It has been tested on ROS Noetic.

## Installation

In order to install darknet_ros, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd yolov4-ws/src
    git clone --recursive https://github.com/t1mkhuan9/yolov4-ros-noetic darknet_ros

Then compile the package by

    cd ../
    catkin_make 

### Common Fixes
If you are compiling with CUDA, you might receive the following build error:

    nvcc fatal : Unsupported gpu architecture 'compute_61'.

This means that you need to check the compute capability (version) of your GPU. You can find a list of supported GPUs in CUDA here: [CUDA - WIKIPEDIA](https://en.wikipedia.org/wiki/CUDA#Supported_GPUs). Simply find the compute capability of your GPU and add it into darknet_ros/CMakeLists.txt. Simply add a similar line like

    -O3 -gencode arch=compute_62,code=sm_62

This build error may also occur sometimes:

    image.c:16:10: fatal error: stb_image.h: No such file or directory
        #include "stb_image.h"

This is because it is not correctly referenced. Change the include statement in `darknet/src/image.c` line 16 and line 20 to

    #include "../3rdparty/stb/include/stb_image.h"
    #include "../3rdparty/stb/include/stb_image_write.h"

Notice: DO NOT change if the problem does not occur.

### Download weights

    cd darknet_ros/yolo_network_config/weights/
    wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights
    wget https://github.com/AlexeyAB/darknet/releases/download/yolov4/yolov4-tiny.weights


## Basic Usage

    source devel/setup.zsh # or setup.bash
    roslaunch darknet_ros yolov4.launch
    roslaunch darknet_ros yolov4-tiny.launch