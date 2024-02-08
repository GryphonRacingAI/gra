FROM shiukaheng/ros-base:latest

ENTRYPOINT [ "/usr/bin/env" ]

# Copy in ./apt-get-wrapper.sh
COPY ./apt-get-wrapper.sh /usr/local/bin/apt-get-wrapper.sh
RUN chmod +x /usr/local/bin/apt-get-wrapper.sh

RUN /usr/local/bin/apt-get-wrapper.sh python3-catkin-tools nano curl git x11-apps
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-rosserial ros-noetic-rosserial-arduino ros-noetic-rqt-ez-publisher ros-noetic-ros-control ros-noetic-ros-controllers libeigen3-dev

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
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-hector-sensors-description

# Install Intel RealSense camera libraries
RUN /usr/local/bin/apt-get-wrapper.sh librealsense2-dev librealsense2-dbg  librealsense2-dkms librealsense2-utils ros-noetic-realsense2-camera

# Install Foxglove message schemas
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-foxglove-msgs

# Upstart
RUN /usr/local/bin/apt-get-wrapper.sh ros-noetic-robot-upstart

# Copying source code over
# COPY .. /root/gra
# COPY ./.bashrc /root/.bashrc
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
# RUN ln -s /root/gra/ros src

# Set password for root to be gra
RUN echo 'root:gra' | chpasswd

ENV DEV_ENV=1
