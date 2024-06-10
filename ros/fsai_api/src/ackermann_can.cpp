#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pthread.h>
#include <cmath>

extern "C" {
#include "fs-ai_api.h"
}

// Constants
const float MOTOR_RATIO = 3.5;
const float DEGREE_CONVERSION = 180.0 / M_PI;

// Global variables
fs_ai_api_ai2vcu ai2vcu_data;
pthread_t loop_tid;
int timing_us = 10000;
bool mission_started = false;

void ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
    if (mission_started) {
        // Convert speed from m/s to RPM (Motor ratio is 3.5:1)
        float speed_rpm = msg->speed * MOTOR_RATIO;

        // Convert steering angle from radians to degrees
        float steering_angle_deg = msg->steering_angle * DEGREE_CONVERSION;

        // Update the ai2vcu_data struct
        ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = speed_rpm;
        ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = steering_angle_deg;
        ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 100; // Torque to 100
    }
}

void* loop_thread(void*) {
    fs_ai_api_vcu2ai vcu2ai_data;

    while (ros::ok()) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

        // Handshake logic
        if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_OFF) {
            ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
        } else if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON) {
            ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
        } else {
            ROS_ERROR("HANDSHAKE_BIT error");
        }

        // Check for mission mode selection
        if (vcu2ai_data.VCU2AI_AMI_STATE != AMI_NOT_SELECTED && !mission_started) {
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
        }

        // Check if AS_STATE is ready (2)
        if (vcu2ai_data.VCU2AI_AS_STATE == AS_READY && !mission_started) {
            ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_RUNNING;
            mission_started = true;
        }

        // Send data to VCU
        fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

        // Loop timing
        usleep(timing_us);
    }

    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_can");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun fsai_api ackermann_can <can_interface>");
        return 1;
    }

    if (fs_ai_api_init(argv[1], 1, 0)) {
        ROS_ERROR("fs_ai_api_init() failed");
        return 1;
    }

    // Create a thread for the loop
    if (pthread_create(&loop_tid, NULL, loop_thread, NULL) != 0) {
        ROS_ERROR("Can't create loop thread...");
        return 1;
    }

    // Subscribe to the ackermann_cmd topic
    ros::Subscriber sub = nh.subscribe("/ackermann_cmd", 10, ackermannCmdCallback);

    ros::spin();

    return 0;
}
