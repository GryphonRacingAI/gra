#!/bin/bash

# ANSI Color Codes
RED="\033[1;31m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
RESET="\033[0m"

echo "Note: This script assumes you have installed ROS noetic and all the apt-get and python requirements already."

# Function to safely create symlinks
safe_symlink() {
    local source=$1
    local target=$2

    # Check if target is already a symlink
    if [ -L "$target" ]; then
        # Remove the existing symlink if it points somewhere else
        if [ "$(readlink "$target")" != "$source" ]; then
            echo -e "${YELLOW}Warning: Removing old symlink at $target${RESET}"
            rm "$target"
        else
            echo "Info: $target is already correctly symlinked to $source"
            return
        fi
    elif [ -e "$target" ]; then
        echo -e "${YELLOW}Warning: $target exists and is not a symlink. Skipping...${RESET}"
        return
    fi

    # Create the symlink
    echo "Creating symlink: $target -> $source"
    ln -s "$source" "$target"
}

echo "Setting up symlinks..."

# Symlink operations
safe_symlink "$(realpath ../)" ~/gra
safe_symlink "$(realpath ~/gra/ros/)" ~/catkin_ws/src
rm ~/.bashrc
safe_symlink "$(realpath ~/gra/.devcontainer/.bashrc)" ~/.bashrc
safe_symlink "$(realpath ~/gra/.devcontainer/convenience.sh)" ~/convenience.sh
safe_symlink "$(realpath ~/gra/.devcontainer/bash_profile.sh)" ~/bash_profile.sh

echo -e "${GREEN}Installation script finished.${RESET}"
