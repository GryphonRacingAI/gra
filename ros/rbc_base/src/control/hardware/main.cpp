#include "robot_hw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_hw");

    // Start a spinner to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create an instance of ControlParameterServer
    RobotHW hardware;

    // Initialize the controller manager
    controller_manager::ControllerManager cm(&hardware, hardware.nh);

    while (ros::ok()) {
        // Update the robot state
        hardware.read();

        // Update the controllers
        cm.update(ros::Time::now(), hardware.control_rate->expectedCycleTime());

        // Send the new commands to the robot
        hardware.write();

        // Sleep for the remaining time until we hit our control rate
        hardware.control_rate->sleep();
    }

    spinner.stop();
    return 0;
}