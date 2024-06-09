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

// Loop thread to handle communication with the VCU
void* loop_thread(void*) {
    ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
    fs_ai_api_vcu2ai vcu2ai_data;

    // Step 1: Establish handshake
    // while (ros::ok()) {
    //     // Get data from VCU
    //     fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

    //     // Handshake logic
    //     if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_OFF) {
    //         ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_OFF;
    //     } else if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON) {
    //         ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
    //         break; // Handshake established
    //     } else {
    //         ROS_ERROR("HANDSHAKE_BIT error");
    //     }

    //     // Send data to VCU
    //     fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

    //     // Loop timing
    //     usleep(timing_us);
    // }

    ROS_INFO("Starting Step 2");

    // Step 2: Wait for AMI_STATE to change from 0 to another value (1-7)
    while (ros::ok() && vcu2ai_data.VCU2AI_AMI_STATE == AMI_NOT_SELECTED) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);



        // Loop timing
        usleep(timing_us);
    }

    ROS_INFO("Starting Step 3");
    
    // Step 3: Set MISSION_STATUS to 1
    while (ros::ok() && vcu2ai_data.VCU2AI_RES_GO_SIGNAL != 1) {

        ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
        fs_ai_api_ai2vcu_set_data(&ai2vcu_data);
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);
        ROS_INFO(static_cast<fs_ai_api_ami_state_e>(vcu2ai_data.VCU2AI_AMI_STATE));
        ROS_INFO("Status: " << VCU2AI_AMI_STATE);

    }


    ROS_INFO("Starting Step 4");

    // Step 4: Wait for AS_STATE to become 2
    while (ros::ok() && vcu2ai_data.VCU2AI_AS_STATE != AS_READY) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

        // Loop timing
        usleep(timing_us);
    }

    ROS_INFO("Starting Step 5");
    // Step 5: Set direction to 1 and torque to 100
    ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
    ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 100;

    ROS_INFO("Starting Step 6");

    // Step 6: Main loop to handle incoming Ackermann commands and communicate with VCU
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