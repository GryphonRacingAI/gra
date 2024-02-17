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

RUN rm -rf /var/lib/apt/lists/* # for some reason this is needed when we switch to or from the apt-get-wrapper

RUN /usr/local/bin/apt-get-wrapper.sh python3-catkin-tools nano curl git
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rosserial ros-noetic-rqt-ez-publisher --fix-missing
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

# # SMACH
# RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach
# RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach-msgs
# RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach-ros

# # Install slam_toolbox
# RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-slam-toolbox

# # Install move_base_flex
# RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-move-base

# Install ipykernel and pip for python3
RUN /usr/local/bin/apt-get-wrapper.sh python3-pip
RUN pip3 install ipykernel

# # Install OpenSSH for remote development
# RUN /usr/local/bin/apt-get-wrapper.sh install -y openssh-server
# RUN service ssh start

# # Copy in sshd_config file
# COPY ./sshd_config /etc/ssh/sshd_config

# # Install scipy, numpy, matplotlib, pandas, scikit-learn, jupyterlab
# RUN pip3 install --upgrade numpy matplotlib scipy pandas scikit-learn ipympl jupyterlab

# # Install openssh-server
# RUN /usr/local/bin/apt-get-wrapper.sh openssh-server

# # Install sshpass
# RUN /usr/local/bin/apt-get-wrapper.sh sshpass

# RUN /usr/local/bin/apt-get-wrapper.sh sshfs

# Configure it to run on port 2222
# RUN sed -i 's/#Port 22/Port 2222/g' /etc/ssh/sshd_config

# RUN /usr/local/bin/apt-get-wrapper.sh rsync

RUN /usr/local/bin/apt-get-wrapper.sh lsyncd
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rplidar-ros
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-gmapping
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-navigation
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-move-base-flex
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-explore-lite
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-hector-sensors-description --ignore-missing
    
# Install Intel RealSense camera libraries
RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dev librealsense2-dbg librealsense2-utils ros-noetic-realsense2-camera

# Install Foxglove message schemas
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-foxglove-msgs ros-noetic-foxglove-bridge

# Upstart
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-robot-upstart

# Ackermann messages (needed by ackermann_vehicle_navigation)
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-ackermann-msgs


FROM gra-base AS gra-dev
# Set password for root to be gra
RUN echo 'root:gra' | chpasswd

RUN rm -rf /var/lib/apt/lists/* # for some reason this is needed when we switch to or from the apt-get-wrapper

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full=1.5.0-1* \
    && rm -rf /var/lib/apt/lists/*

RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dkms x11-apps
ENV DEV_ENV=1
