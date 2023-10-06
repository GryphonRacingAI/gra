#!/bin/bash

# ANSI Color Codes
RED="\033[1;31m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
RESET="\033[0m"

# Function to safely remove symlinks
safe_unlink() {
    local target=$1

    if [ -L "$target" ]; then
        echo "Removing symlink at $target"
        rm "$target"
    elif [ -e "$target" ]; then
        echo -e "${YELLOW}Warning: $target exists and is not a symlink. Skipping...${RESET}"
    else
        echo "Info: $target does not exist. Nothing to remove."
    fi
}

echo "Starting uninstallation..."

# Remove the symlinks
safe_unlink ~/rbc
safe_unlink ~/catkin_ws/src
safe_unlink ~/.bashrc
safe_unlink ~/convenience.sh

# Disable the systemd service
if systemctl is-enabled --quiet roscore.service; then
    echo "Disabling roscore.service..."
    sudo systemctl disable roscore.service
else
    echo "Info: roscore.service is not enabled. Nothing to do."
fi

# Remove the service file from the systemd directory
if [ -f /etc/systemd/system/roscore.service ]; then
    echo "Removing roscore.service from /etc/systemd/system/..."
    sudo rm /etc/systemd/system/roscore.service

    # Reload systemd to recognize the changes
    echo "Reloading systemd..."
    sudo systemctl daemon-reload
else
    echo "Info: roscore.service does not exist in /etc/systemd/system/. Nothing to remove."
fi

echo -e "${GREEN}Uninstallation script finished.${RESET}"
