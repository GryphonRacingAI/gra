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
- `roscore` to launch ros
- `rosrun {package name} {node name}`
- `roslaunch {package name} {launch file}`
- `roslaunch rplidar_ros view_rplidar.launch` to test the RPlidar
- `rostopic list`
- `rostopic echo {topic name}` print out messages to topic
- `rosbag record -o recording.bag --duration 30 /scan /camera/color/image_raw/compressed /camera/aligned_depth_to_color/image_raw/compressedDepth /tf_static /tf` to record 30s of data
- `rosbag play -l recording.bag` to replay the recording in a loop
- `roslaunch urdf_tutorial display.launch model:=ros/ackermann_vehicle_description/urdf/em_3905_hokuyo.urdf.xacro` to launch RViz (remeber the X11 command above)
- `roslaunch realsense2_camera rs_aligned_depth.launch` to test the RealSense camera

## Compressed images
- RViz is buggy so you will have to set up an rviz configuration with the camera plugged in and set it up to show compressed images. (this is a TODO)