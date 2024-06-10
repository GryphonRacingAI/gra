#!/bin/sh

sudo ip link set up can0 type can bitrate 500000
echo "driverless startup script executed" >> /home/gra/Desktop/driverless_startup_log.txt
