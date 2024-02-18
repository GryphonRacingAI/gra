#!/bin/bash
set -e # exit on error

if [ $EUID -eq 0 ]; then
    echo "This script needs to be run as a regular user, without sudo."
    exit 1
fi

if ! sudo true; then
    echo "This scrpt needs to run some commands as sudo, so it needs you to authenticate."
    exit 1
fi

# Do not ask user for password when sudoing ever agin (still needs pw to log in)
export USERNAME=$USER
if sudo grep -q $USERNAME /etc/sudoers
then
    echo "$USERNAME is already registered as NOPASSWD"
else
    sudo bash -c 'echo "${0} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers' "$USERNAME"
fi

echo "Stopping unattended-upgrades because it can block installation..."
sudo systemctl stop unattended-upgrades.service

sudo cp .devcontainer/apt-get-wrapper.sh /usr/local/bin
sudo chmod +x /usr/local/bin/apt-get-wrapper.sh

python3 .devcontainer/native_install/docker2sh.py --target gra-base .devcontainer/Dockerfile > installation_script_generated.sh
sudo chmod +x installation_script_generated.sh
sudo ./installation_script_generated.sh

mkdir -p $HOME/catkin_ws
./bot_scripts/install.sh

# Log out
sudo pkill -KILL -u pi