#include <ros/ros.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <pthread.h>
#include <cmath>
#include <sstream>
#include <fsai_api/VCU2AI.h>

extern "C" {
#include "fs-ai_api.h"
}

// Constants
const float MOTOR_RATIO = 3.5;
const float DEGREE_CONVERSION = 180.0 / M_PI;
const float WHEEL_RADIUS = 0.2575; // Example wheel radius in meters

// Global variables
fs_ai_api_ai2vcu ai2vcu_data;
pthread_t loop_tid;
int timing_us = 10000;
bool mission_started = false;
bool mission_finished = false;

ros::Publisher vcu2ai_pub;

void ackermannCmdCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
    ROS_INFO("Message received");
 
    if (mission_started) {
        ROS_INFO("Propagating commands");

        // Convert speed from m/s to RPM (taking into account wheel radius)
        float car_speed_mps = msg->speed;
        float wheel_rpm = (car_speed_mps / WHEEL_RADIUS) * 60.0 / (2 * M_PI);
        float motor_rpm = wheel_rpm * MOTOR_RATIO;

        ROS_INFO("motor rpm %f", motor_rpm);

        // Temporary: set finish on negative speed
        if (motor_rpm < 0) {
            motor_rpm = 0;
            mission_finished = true;
        }

        // Convert steering angle from radians to degrees
        float steering_angle_deg = msg->steering_angle * DEGREE_CONVERSION;

        // Update the ai2vcu_data struct
        ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = motor_rpm;
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
            // ROS_INFO("Handshake 0");
        } else if (vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT == HANDSHAKE_RECEIVE_BIT_ON) {
            ai2vcu_data.AI2VCU_HANDSHAKE_SEND_BIT = HANDSHAKE_SEND_BIT_ON;
            // ROS_INFO("Handshake 1");
        } else {
            ROS_ERROR("HANDSHAKE_BIT error");
        }

        // Check for mission mode selection
        if (vcu2ai_data.VCU2AI_AMI_STATE != AMI_NOT_SELECTED && !mission_started) {
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
            ROS_INFO("Mission Selected");
        }

        // Check if AS_STATE is ready (2)
        if (vcu2ai_data.VCU2AI_AS_STATE == AS_READY && !mission_started) {
            ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_NEUTRAL;
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_SELECTED;
            ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = 0;
            ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = 0;

            ROS_INFO("AS Ready");
        }

        // Goes AS_Driving
        if (vcu2ai_data.VCU2AI_AS_STATE == AS_DRIVING) {
            ai2vcu_data.AI2VCU_DIRECTION_REQUEST = DIRECTION_FORWARD;
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_RUNNING;

            mission_started = true;
            ROS_INFO("AS Driving");
        }

        if (mission_finished) {
            ROS_INFO("Finished");
            ai2vcu_data.AI2VCU_MISSION_STATUS = MISSION_FINISHED;
        }

        // Send data to VCU
        fs_ai_api_ai2vcu_set_data(&ai2vcu_data);

        // Publish VCU data
        fsai_api::VCU2AI msg;
        msg.handshake_receive_bit = vcu2ai_data.VCU2AI_HANDSHAKE_RECEIVE_BIT;
        msg.res_go_signal = vcu2ai_data.VCU2AI_RES_GO_SIGNAL;
        msg.as_state = vcu2ai_data.VCU2AI_AS_STATE;
        msg.ami_state = vcu2ai_data.VCU2AI_AMI_STATE;
        msg.steer_angle_deg = vcu2ai_data.VCU2AI_STEER_ANGLE_deg;
        msg.brake_press_f_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_F_pct;
        msg.brake_press_r_pct = vcu2ai_data.VCU2AI_BRAKE_PRESS_R_pct;
        msg.fl_wheel_speed_rpm = vcu2ai_data.VCU2AI_FL_WHEEL_SPEED_rpm;
        msg.fr_wheel_speed_rpm = vcu2ai_data.VCU2AI_FR_WHEEL_SPEED_rpm;
        msg.rl_wheel_speed_rpm = vcu2ai_data.VCU2AI_RL_WHEEL_SPEED_rpm;
        msg.rr_wheel_speed_rpm = vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm;
        msg.fl_pulse_count = vcu2ai_data.VCU2AI_FL_PULSE_COUNT;
        msg.fr_pulse_count = vcu2ai_data.VCU2AI_FR_PULSE_COUNT;
        msg.rl_pulse_count = vcu2ai_data.VCU2AI_RL_PULSE_COUNT;
        msg.rr_pulse_count = vcu2ai_data.VCU2AI_RR_PULSE_COUNT;

        vcu2ai_pub.publish(msg);

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

    // Publisher for VCU2AI data
    vcu2ai_pub = nh.advertise<fsai_api::VCU2AI>("vcu2ai", 10);

    // Subscribe to the ackermann_cmd topic
    ros::Subscriber sub = nh.subscribe("/ackermann_cmd", 10, ackermannCmdCallback);

    ros::spin();

    return 0;
}
