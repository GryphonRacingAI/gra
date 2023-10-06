#include "./hardware_parameter_server.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "hardware_parameter_server");
    // Create an instance of ControlParameterServer
    HardwareParameterServer paramServer;
    ros::spin();
    return 0;
}