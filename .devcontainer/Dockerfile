FROM shiukaheng/ros-base:latest

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

# End of ROS dependencies

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
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-rplidar-ros
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-gmapping
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-navigation
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-move-base-flex
RUN /usr/local/bin/apt-get-wrapper.sh install -y ros-noetic-explore-lite



# Configure it to run on port 2222
RUN sed -i 's/#Port 22/Port 2222/g' /etc/ssh/sshd_config

# Upstart
RUN /usr/local/bin/apt-get-wrapper.sh -y install ros-noetic-robot-upstart

# Copying source code over
# COPY .. /root/rbc
# COPY ./.bashrc /root/.bashrc
RUN mkdir -p /root/catkin_ws
WORKDIR /root/catkin_ws
# RUN ln -s /root/rbc/ros src

# Set password for root to be rbc
RUN echo 'root:rbc' | chpasswd

ENV DEV_ENV=1