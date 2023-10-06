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
safe_symlink "$(realpath ../)" ~/rbc
safe_symlink "$(realpath ~/rbc/ros/)" ~/catkin_ws/src
safe_symlink "$(realpath ~/rbc/.devcontainer/.bashrc)" ~/.bashrc
safe_symlink "$(realpath ~/rbc/.devcontainer/convenience.sh)" ~/convenience.sh

# Dynamically create the roscore.service file
echo "Generating roscore.service for user $USER..."
cat > roscore.service <<EOL
[Unit]
Description=roscore service
After=network.target

[Service]
Type=simple
ExecStart=/bin/bash -ic 'source ~/.bashrc && roscore'
Restart=always
User=$USER

[Install]
WantedBy=multi-user.target
EOL

# Check if roscore.service exists before copying
if [ ! -f roscore.service ]; then
    echo -e "${YELLOW}Warning: roscore.service does not exist in current directory. Skipping...${RESET}"
else
    echo "Copying roscore.service to /etc/systemd/system/"
    sudo cp roscore.service /etc/systemd/system/

    echo "Reloading systemd to recognize the service..."
    sudo systemctl daemon-reload

    echo "Enabling the roscore.service to start on boot..."
    sudo systemctl enable roscore.service   

    echo "Starting the roscore.service..."
    sudo systemctl start roscore.service
fi

echo -e "${GREEN}Installation script finished.${RESET}"
