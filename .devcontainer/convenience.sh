#!/bin/bash

source ~/gra/.devcontainer/env_vars.sh

function import() {
    source $GRA_REPO/.devcontainer/convenience/$1.sh
}

# Basic utilities
import refreshenv
import logging
import depends
import run_in_dir
import persistent_exports
import echo_color

# Setting environment
import ros_env

# Shortcuts
import catkin_shortcuts
import arduino_shortcuts
import quick_navigation

# Dockerfile emulation
import dockerfile_emu

# ROS remote utils
import master_switching
import grainfo
import remote_bot_utils

# DX
import rosbash_enhancements
import gra_shortcuts