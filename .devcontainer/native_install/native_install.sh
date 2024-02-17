#!/bin/bash

export USERNAME=$USER
if [ $EUID -eq 0 ]; then
    echo "This script needs to be run as a regular user, without sudo."
    exit 1
fi

if ! sudo true; then
    echo "This scrpt needs to run some commands as sudo, so it needs you to authenticate."
    exit 1
fi

# if sudo grep -q $USERNAME /etc/sudoers
# then
#     echo "$USERNAME is already registered as NOPASSWD"
# else
#     sudo echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
# fi

python3 .devcontainer/native_install/docker2sh.py --target gra-base .devcontainer/Dockerfile > installation_script_generated.sh
sudo cp .devcontainer/apt-get-wrapper.sh /usr/local/bin
sudo chmod +x /usr/local/bin/apt-get-wrapper.sh
sudo chmod +x installation_script_generated.sh
sudo ./installation_script_generated.sh

mkdir -p $HOME/catkin_ws
./bot_scripts/install.sh