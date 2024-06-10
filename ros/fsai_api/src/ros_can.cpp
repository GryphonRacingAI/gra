#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pthread.h>
#include <cmath>
#include <ros/console.h>

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


// Callback to handle incoming Ackermann commands
void ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
    // Convert speed from m/s to RPM (Motor ratio is 3.5:1)
    float speed_rpm = msg->speed * MOTOR_RATIO;

    // Convert steering angle from radians to degrees
    float steering_angle_deg = msg->steering_angle * DEGREE_CONVERSION;

    // Update the ai2vcu_data struct
    ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = speed_rpm;
    ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = steering_angle_deg;
}

void missionStatusCallback(const rosCan_msgs::MissionStatus::ConstPtr& msg) {  // TODO: type
    ai2vcu_data.AI2VCU_MISSION_STATUS = msg->missionStatus;
}

void emergencyBrakeCallback(const rosCan_msgs::EmergencyBrake::ConstPtr& msg) {  // TODO: type
    ai2vcu_data.AI2VCU_ESTOP_REQUEST = msg->emergencyBrake;
}

// Loop thread to handle communication with the VCU
void* loop_thread(void*) {
    fs_ai_api_vcu2ai vcu2ai_data;

    while (ros::ok()) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

        // Publish VCU2AI data
        pubAmiState.publish(vcu2ai_data.VCU2AI_AMI_STATE)

        // Handshake logic
        if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_OFF) {
            ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
        } else if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON) {
            ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
            break; // Handshake established
        } else {
            ROS_ERROR("HANDSHAKE_BIT error");
        }

        // Send data to VCU
        fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

        // Loop timing
        usleep(timing_us);
    }

    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_can");
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
    ros::Subscriber subAckermannCmd = nh.subscribe("/ackermann_cmd", 1, ackermannCmdCallback);

    // Subscribe to mission status topic
    ros::Subscriber subMissionStatus = nh.subscribe("/mission_status", 1, missionStatusCallback);

    // Subscribe to emergency brake request topic
    ros::Subscriber subEmergencyBrake = nh.subscribe("/emergency_brake", 1, emergencyBrakeCallback);

    // Publish to ami_state topic
    ros::Publisher pubAmiState = nh.advertise<fs_ai_api_vcu2ai_struct>("/ami_state", 10);

    ros::spin();

    return 0;
}
