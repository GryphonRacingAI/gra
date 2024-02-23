# The first bit is an auto generated Dockerfile for ros:ros-core
# generated from docker_images/create_ros_core_image.Dockerfile.em
FROM ubuntu:focal AS ros-base

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO noetic

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-core=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

CMD ["bash"]

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstools \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-base=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-robot=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

ENTRYPOINT [ "/usr/bin/env" ]


FROM ros-base AS gra-base
COPY ./apt-get-wrapper.sh /usr/local/bin/apt-get-wrapper.sh
RUN chmod +x /usr/local/bin/apt-get-wrapper.sh

RUN /usr/local/bin/apt-get-wrapper.sh python3-catkin-tools nano curl git
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rosserial ros-noetic-rqt-ez-publisher
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

# SMACH
# RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-smach
# RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-smach-msgs
# RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-smach-ros

# Install slam_toolbox
# RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-slam-toolbox

# Install move_base_flex
# RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-move-base

# Install ipykernel and pip for python3
RUN /usr/local/bin/apt-get-wrapper.sh python3-pip
RUN pip3 install ipykernel

# Install rplidar and gmapping
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rplidar-ros
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-gmapping
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-navigation
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-move-base-flex
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-explore-lite
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-hector-sensors-description

# Ackermann messages (needed by ackermann_vehicle_navigation)
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-ackermann-msgs

# Install Intel RealSense camera libraries
RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dev librealsense2-dbg  librealsense2-dkms librealsense2-utils ros-noetic-realsense2-camera

# Install Foxglove message schemas
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-foxglove-msgs ros-noetic-foxglove-bridge

# Install dependencies for ultralytics yolov8
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-vision-msgs

# Install velodyne 3D Lidar  package
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-velodyne

# Upstart
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-robot-upstart


# End of ROS dependencies

# Install dependencies for Darknet and OpenCV
RUN /usr/local/bin/apt-get-wrapper.sh build-essential cmake pkg-config unzip yasm git checkinstall
RUN /usr/local/bin/apt-get-wrapper.sh libjpeg-dev libpng-dev libtiff-dev
RUN /usr/local/bin/apt-get-wrapper.sh libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
RUN /usr/local/bin/apt-get-wrapper.sh libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN /usr/local/bin/apt-get-wrapper.sh libgtk2.0-dev libtbb-dev qt5-default
RUN /usr/local/bin/apt-get-wrapper.sh libatlas-base-dev
RUN /usr/local/bin/apt-get-wrapper.sh libfaac-dev libmp3lame-dev libtheora-dev
RUN /usr/local/bin/apt-get-wrapper.sh libvorbis-dev libxvidcore-dev
RUN /usr/local/bin/apt-get-wrapper.sh libopencore-amrnb-dev libopencore-amrwb-dev
RUN /usr/local/bin/apt-get-wrapper.sh libavutil-dev libavfilter-dev libavdevice-dev
RUN /usr/local/bin/apt-get-wrapper.sh libx264-dev libx265-dev libnuma-dev
RUN /usr/local/bin/apt-get-wrapper.sh libv4l-dev v4l-utils
RUN /usr/local/bin/apt-get-wrapper.sh libprotobuf-dev protobuf-compiler
RUN /usr/local/bin/apt-get-wrapper.sh libgoogle-glog-dev libgflags-dev
RUN /usr/local/bin/apt-get-wrapper.sh libgphoto2-dev libeigen3-dev libhdf5-dev doxygen

# Install OpenCV
RUN /usr/local/bin/apt-get-wrapper.sh libopencv-dev

# Install OpenSSH for remote development
# RUN /usr/local/bin/apt-get-wrapper.sh openssh-server
# RUN service ssh start

# Copy in sshd_config file
# COPY ./sshd_config /etc/ssh/sshd_config

# # Install OpenSSH for remote development
# RUN /usr/local/bin/apt-get-wrapper.sh install -y openssh-server
# RUN service ssh start

# Install openssh-server
# RUN /usr/local/bin/apt-get-wrapper.sh openssh-server

# Install sshpass
# RUN /usr/local/bin/apt-get-wrapper.sh sshpass

# RUN /usr/local/bin/apt-get-wrapper.sh sshfs

# RUN /usr/local/bin/apt-get-wrapper.sh rsync

RUN /usr/local/bin/apt-get-wrapper.sh lsyncd

# Install Python packages for ultralytics yolov8
RUN python3 -m pip install lap==0.4.0 \
    onnx==1.14.0 \
    urllib3==1.26.18 \
    numpy==1.23.4 \
    opencv-python==4.7.0.72 \
    ultralytics

RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rplidar-ros
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-gmapping
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-navigation
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-move-base-flex
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-explore-lite
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-hector-sensors-description
    
# Install Intel RealSense camera libraries
RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dev librealsense2-dbg librealsense2-utils ros-noetic-realsense2-camera


# Configure it to run on port 2222
# RUN sed -i 's/#Port 22/Port 2222/g' /etc/ssh/sshd_config


FROM gra-base AS gra-dev

# Set password for root to be gra
RUN echo 'root:gra' | chpasswd

# install ros packages
RUN rm -rf /var/lib/apt/lists/* && apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dkms x11-apps
ENV DEV_ENV=1
