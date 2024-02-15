FROM shiukaheng/ros-base:latest

# Upgrade cmake to version 3.18
RUN apt-get update && \
    apt-get install -y wget && \
    wget https://github.com/Kitware/CMake/releases/download/v3.18.0/cmake-3.18.0-Linux-x86_64.sh -O /tmp/cmake-install.sh && \
    chmod +x /tmp/cmake-install.sh && \
    /tmp/cmake-install.sh --skip-license --prefix=/usr/local && \
    rm /tmp/cmake-install.sh && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT [ "/usr/bin/env" ]

# Copy in ./apt-get-wrapper.sh
COPY ./apt-get-wrapper.sh /usr/local/bin/apt-get-wrapper.sh
RUN chmod +x /usr/local/bin/apt-get-wrapper.sh

RUN /usr/local/bin/apt-get-wrapper.sh install -y python3-catkin-tools nano curl git x11-apps
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rqt-ez-publisher ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

# Astra Camera
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-rgbd-launch ros-noetic-libuvc-camera ros-noetic-libuvc-ros libuvc-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libgoogle-glog-dev

# LDS-01
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-hls-lfcd-lds-driver

# SMACH
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach-msgs
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-smach-ros

# ROSBridge
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-rosbridge-suite

WORKDIR /root
RUN curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
RUN /root/bin/arduino-cli core update-index
RUN /root/bin/arduino-cli core install arduino:avr
RUN /root/bin/arduino-cli board install arduino:avr:mega
RUN /root/bin/arduino-cli lib install "FireTimer"

# Install slam_toolbox
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-slam-toolbox

# Install move_base_flex
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-move-base

# Install tkinter for python3
RUN /usr/local/bin/apt-get-wrapper.sh install -y python3-tk

# Install rplidar and gmapping
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-rplidar-ros
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-gmapping
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-navigation
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-move-base-flex
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-explore-lite
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-hector-sensors-description
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-ackermann-msgs

# Install realsense 
RUN /usr/local/bin/apt-get-wrapper.sh install -y librealsense2-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y librealsense2-dbg  
RUN /usr/local/bin/apt-get-wrapper.sh install -y librealsense2-dkms
RUN /usr/local/bin/apt-get-wrapper.sh install -y librealsense2-utils
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-realsense2-camera

# Install dependencies for ultralytics yolov8
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-vision-msgs


# End of ROS dependencies

# Install dependencies for Darknet and OpenCV
RUN /usr/local/bin/apt-get-wrapper.sh install -y build-essential cmake pkg-config unzip yasm git checkinstall
RUN /usr/local/bin/apt-get-wrapper.sh install -y libjpeg-dev libpng-dev libtiff-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libavcodec-dev libavformat-dev libswscale-dev libavresample-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libgtk2.0-dev libtbb-dev qt5-default
RUN /usr/local/bin/apt-get-wrapper.sh install -y libatlas-base-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libfaac-dev libmp3lame-dev libtheora-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libvorbis-dev libxvidcore-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libopencore-amrnb-dev libopencore-amrwb-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libavutil-dev libavfilter-dev libavdevice-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libx264-dev libx265-dev libnuma-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libv4l-dev v4l-utils
RUN /usr/local/bin/apt-get-wrapper.sh install -y libprotobuf-dev protobuf-compiler
RUN /usr/local/bin/apt-get-wrapper.sh install -y libgoogle-glog-dev libgflags-dev
RUN /usr/local/bin/apt-get-wrapper.sh install -y libgphoto2-dev libeigen3-dev libhdf5-dev doxygen

# Install OpenCV
RUN /usr/local/bin/apt-get-wrapper.sh install -y libopencv-dev

# Install OpenSSH for remote development
RUN /usr/local/bin/apt-get-wrapper.sh install -y openssh-server
RUN service ssh start

# Copy in sshd_config file
COPY ./sshd_config /etc/ssh/sshd_config

# Install avahi-daemon for mDNS
RUN /usr/local/bin/apt-get-wrapper.sh install -y avahi-daemon iputils-ping

# Install ipykernel and pip for python3
RUN /usr/local/bin/apt-get-wrapper.sh install -y python3-pip
RUN pip3 install ipykernel

# Install scipy, numpy, matplotlib, pandas, scikit-learn, jupyterlab
RUN pip3 install --upgrade numpy matplotlib scipy pandas scikit-learn ipympl jupyterlab

# Install openssh-server
RUN /usr/local/bin/apt-get-wrapper.sh install -y openssh-server

# Install sshpass
RUN /usr/local/bin/apt-get-wrapper.sh install -y sshpass

RUN /usr/local/bin/apt-get-wrapper.sh install -y sshfs

RUN /usr/local/bin/apt-get-wrapper.sh install -y rsync

RUN /usr/local/bin/apt-get-wrapper.sh install -y lsyncd

# Install Python packages for ultralytics yolov8
RUN python3 -m pip install lap==0.4.0 \
    onnx==1.14.0 \
    urllib3==1.26.18 \
    numpy==1.23.4 \
    opencv-python==4.7.0.72 \
    ultralytics

# Configure it to run on port 2222
RUN sed -i 's/#Port 22/Port 2222/g' /etc/ssh/sshd_config

# Upstart
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-robot-upstart

# Copying source code over
# COPY .. /root/gra
# COPY ./.bashrc /root/.bashrc
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
# RUN ln -s /root/gra/ros src


# Set password for root to be gra
RUN echo 'root:gra' | chpasswd

ENV DEV_ENV=1
