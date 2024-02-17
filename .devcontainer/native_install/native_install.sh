#!/bin/bash
if [ $EUID != 0 ]; then
    sudo echo "pi ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
    exit $?
fi
python3 .devcontainer/native_install/docker2sh.py --target gra-base .devcontainer/Dockerfile > installation_script_generated.sh
cp .devcontainer/apt-get-wrapper.sh /usr/local/bin
sudo chmod +x /usr/local/bin/apt-get-wrapper.sh
sudo chmod +x installation_script_generated.sh
sudo ./installation_script_generated.sh

mkdir -p $HOME/catkin_ws
./bot_scripts/install.sh