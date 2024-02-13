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
- `rosbag record -o recordings/output_file.bag -a --duration 30` to record 30s of data
- `rosbag replay --loop recordings/output_file.bag` to replay the recording in a loop
- `rosbag record -o {name} --duration=30 /initialpose /joint_states /move_base_simple/goal /rosout /rosout_agg /tf /tf_static /camera/aligned_depth_to_color/camera_info /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/camera_info /camera/color/image_raw/compressed` includes the essential topics for recording depth camera bags.