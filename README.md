# Development

## MacOS
On Mac, you may need to run `sudo chown -R $(whoami) ~/.docker` to allow docker to run without sudo.

## X11 auth
To enable X11 forwarding, run `xhost +local:docker` on host computer

## Pre-requisites
- $(hostname).local is broadcasted on the network via mDNS, for both bot and dev computer
- Docker needs to be run on linux

## Notes
- Please make sure the permissions are correct before committing

## Commonly used commands
### Basic ROS
- `roscore` to launch ros
- `rosrun {package name} {node name}`
- `roslaunch {package name} {launch file}`
- `rostopic list`
- `rostopic echo {topic name}` print out messages to topic
### Sensors
- `roslaunch rplidar_ros view_rplidar.launch` to test the RPidar
- `roslaunch realsense2_camera rs_aligned_depth.launch` to test the Intel depth camera
### Other
- `roslaunch gra_recording record.launch output_name:=recording1 duration:=10s` to record 10s of data
- `rosbag play -l recordings/recording1__timestamp.bag` to play a recording in a loop
- `roslaunch urdf_tutorial display.launch model:=ros/ackermann_vehicle_description/urdf/em_3905_hokuyo.urdf.xacro` to launch RViz with a basic setup (remember the X11 command above)
- `roslaunch gra_recording rviz.launch` to display a recorded stream in RViz