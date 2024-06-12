#!/bin/sh

sudo ip link set up can0 type can bitrate 500000
rosrun ackermann_vehicle_navigation numba_precompile.py
echo "driverless startup script executed" >> /home/gra/Desktop/driverless_startup_log.txt
