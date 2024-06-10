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
- To limit Gazebo CPU usage, decrease the update rate in the .world file
    ``` <real_time_update_rate>1000</real_time_update_rate> ```
    e.g. 1000 -> 250

## Known issues
- Ubuntu: memory leak (the memory mysteriously gets full). Deleting the forwarded ports solves the problem some of the time.
- Windows: git on Windows might change the line endings to CRLF which causes a problem when you build the repo. Make sure your line endings are LF if you get the `/bin/sh: 1: /usr/local/bin/apt-get-wrapper.sh: not found` error. `git config --global core.autocrlf false` and recloning the repo should solve the problem. You will need to modify the git config manually if you are using Github desktop and don't have git installed as a command line tool.

## Autonomous System Launch Sequence (full-scale ADS-DV model, with absolute odometry, autocross mission)
1) `roscore`
2) `roslaunch ackermann_vehicle_description ads_dv_sim.launch`
3) `roslaunch ultralytics_ros tracker_with_cloud.launch`
4) `roslaunch ultralytics_ros cone_mapper.launch`
5) `roslaunch fsai_mission autocross.launch`

## Do the following to disable GPU Acceleration: (refer to the wiki page)
1) Go to gra/.devcontainer/docker-compose.yml and comment the deploy section, then rebuild the Docker image in VScode
2) Go to gra/ros/cone_detection/launch/tracker_with_cloud.launch Comment line 17 and uncomment line 15
3) This step reduces Gazebo CPU usage by scaling the simulation time. The simulator will appear slower but CPU load will be reduced significantly.\
Go to gra/ros/ackermann_vehicle_gazebo/worlds/gazebo_world_building/track_small_features.world\
Line 76: <real_time_update_rate>700</real_time_update_rate>\
Reduce the value to reduce CPU load. 1000 is the default value where the simulation time unit = real time unit. When the value is 500, 1 simulation second = 2 real time second.\
Start with 700 and reduce gradually if the cone recognition process is unable to stablise. Unstable process can be observed by the cones swerving in Rviz when the car is steering.

## Commonly used commands
### Basic ROS
- `roscore` to launch ros
- `rosrun {package name} {node name}`
- `roslaunch {package name} {launch file}`
- `rostopic list`
- `rostopic echo {topic name}` print out messages to topic
### Sensors
- `roslaunch rplidar_ros rplidar_a1.launch` to launch the RPlidar
- `roslaunch realsense2_camera rs_aligned_depth.launch` to turn on the Intel depth camera
### Other
- `roslaunch gra_recording record.launch output_name:=recording1 duration:=10s` to record 10s of data
- `rosbag play -l recordings/recording1__timestamp.bag` to play a recording in a loop
- `rosbag record -o {name} --duration=30 /joint_states /rosout /rosout_agg /tf /tf_static /camera/aligned_depth_to_color/camera_info /camera/aligned_depth_to_color/image_raw/compressedDepth /camera/color/camera_info /camera/color/image_raw/compressed` includes the essential topics for recording depth camera rosbags.
- `roslaunch urdf_tutorial display.launch model:=ros/ackermann_vehicle_description/urdf/em_3905_hokuyo.urdf.xacro` to launch RViz with a basic setup (remember the X11 command above)
- `roslaunch gra_recording rviz.launch` to display a recorded stream in RViz
- `rosrun robot_upstart install ackermann_vehicle_navigation/launch/path_follower.launch --job path_follower_ros --symlink` to make a launch file start automatically on boot
- `sudo cp ./driverless_startup.service /etc/systemd/system/driverless_startup.service && sudo systemctl daemon-reload && sudo systemctl enable driverless_startup.service` to register the startup service
