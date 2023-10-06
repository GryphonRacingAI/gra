# Controller
This directory contains the code for the arduino motor controller, which implements the PID control loop.
In order to compile and upload the code to the arduino, you will need to install the [Arduino IDE](https://www.arduino.cc/en/Main/Software).
## Dependencies for building the code
- ArduPID: Install via the Arduino IDE Library Manager
- FireTimer: Install via the Arduino IDE Library Manager
- ROS noetic (for C++ dependencies): `sudo apt install ros-noetic-ros-base`
- rosserial: `sudo apt install ros-noetic-rosserial`
- rosserial_arduino: `sudo apt install ros-noetic-rosserial-arduino`
## Rosserial server
`rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0`