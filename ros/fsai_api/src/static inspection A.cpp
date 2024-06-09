#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pthread.h>
#include <cmath>
#include <ros/console.h>

extern "C" {
#include "fs-ai_api.h"
}

// Global variables
fs_ai_api_ai2vcu ai2vcu_data;
pthread_t loop_tid;
int timing_us = 10000;

// Loop thread to handle communication with the VCU
void* loop_thread(void*) {
    ai2vcu_data.AI2VCU_ESTOP_REQUEST = ESTOP_NO;
    fs_ai_api_vcu2ai vcu2ai_data;
    ros::Time step_4_begin;
    ros::Time step_5_begin;

    int step = 2;
     // Step 1: Establish handshake
    while (ros::ok()) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

        if (step == 2){
            // wait for mission to be selected
            if (vcu2ai_data.VCU2AI_AMI_STATE != AMI_NOT_SELECTED) {
                ROS_INFO("Starting Step 3");
                step = 3;
            }
        }
        if (step == 3){
            // confirm mission is selected
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
            // wait for go signal
            if (vcu2ai_data.VCU2AI_RES_GO_SIGNAL == 1) {
                ROS_INFO("Starting Step 4 (steering sweep)");
                step = 4;
                step_4_begin = ros::Time::now();
            }
        }
        if (step == 4){
            time_elapsed = ros::Time::now() - step_4_begin;
            float angle;
            if (time_elapsed < 3){
                angle = linear_interpolate(time_elapsed, 0,3, 0, -25);
            }
            if (time_elapsed >= 3 && time_elapsed < 9){
                angle = linear_interpolate(time_elapsed, 3,9, -25, 25);
            }
            if (time_elapsed >= 9 && time_elapsed < 12){
                angle = linear_interpolate(time_elapsed, 9,12, 25, 0);
            }
            if (time_elapsed >= 12){
                ROS_INFO("Starting Step 5 (acceleration)");
                step=5;
                step_5_begin = ros::Time::now();
            }
            ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = angle;
        }

        if (step == 5){
            time_elapsed = ros::Time::now() - step_5_begin;
            float rpm;
            if (time_elapsed < 10){
                rpm = linear_interpolate(time_elapsed, 0,10, 0, 200);
            }
            if (time_elapsed >= 10){
                ROS_INFO("Starting Step 6 (braking)");
                step=6;
            }
            ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = rpm;
        }

        if (step == 6){
            time_elapsed = ros::Time::now() - step_5_begin;
            float rpm;
            if (time_elapsed < 1){
                rpm = linear_interpolate(time_elapsed, 0,1, 200, 0);
            }
            
            if (time_elapsed >= 1 && time_elapsed < 5){
                rpm = 0;
            }
            if (time_elapsed >= 5){
                ROS_INFO("Starting Step 7 (finished)");
                step=7;
            }
            ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = rpm;
        }
        
        if (step == 7){
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_FINISHED;
            if (vcu2ai_data.VCU2AI_AMI_STATE == AS_FINISHED){
                return 0;
            }
        }

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

    // Step 4: Wait for AS_STATE to become 2
    while (ros::ok() && vcu2ai_data.VCU2AI_AS_STATE != AS_READY) {
        // Get data from VCU
        fs_ai_api_vcu2ai_get_data(&vcu2ai_data);

        // Loop timing
        usleep(timing_us);
    }

    return nullptr;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_inspection_A");
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
