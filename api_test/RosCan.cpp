#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <cmath>
#include <pthread.h>
#include "fs_ai_api/fs_ai_api.h"

static int driving_flag = 0;
static int mission_complete = 0;

void* loop_thread(void* arg) {
    char inputs[10] = "";
    int data = 0;
    while (1) {
        // background loop
        scanf("%s %d", inputs, &data);

        if (0 == strcmp(inputs, "g")) {
            driving_flag = 1;
        } else if (0 == strcmp(inputs, "f")) {
            mission_complete = 1;
        }
    }
}

class RosCan {
public:
    RosCan(char** argv) {
        ros::NodeHandle nh;
        pthread_t loop_tid;
        // Subscriber
        cmd_vel_sub = nh.subscribe("/ackermann_cmd", 1, &RosCan::cmdVelCallback, this);
        // Publisher
        ros_can_pub = nh.advertise<geometry_msgs::Twist>("/vel_feedback", 1);
        fs_ai_api::fs_ai_api_init(argv, 0, 1);
        // Create the background thread
        pthread_create(&loop_tid, NULL, loop_thread, NULL);
    }

private:
    void cmdVelCallback(const ackermann_msgs::AckermannDrive::ConstPtr& msg) {
        fs_ai_api::fs_ai_api_vcu2ai vcu2ai_data;
        fs_ai_api::fs_ai_api_ai2vcu ai2vcu_data;

        float steering_angle = msg->steering_angle;
        float speed = msg->speed;
        float acceleration = 1;  // acceleration is undecided in ackermann
        float torque = (300 * 0.253 * abs(acceleration + 0.5)) / 2.0;

        fs_ai_api::vcu2ai_get_data(vcu2ai_data);
        fs_ai_api_mission_status_e mission_status = getMissionStatus(vcu2ai_data);
        if (mission_status == fs_ai_api_mission_status_e::MISSION_RUNNING) {
            ai2vcu_data.AI2VCU_DIRECTION_REQUEST = 1;
            ai2vcu_data.AI2VCU_AXLE_TORQUE_REQUEST_Nm = torque;
            ai2vcu_data.AI2VCU_STEER_ANGLE_REQUEST_deg = steering_angle * 180.0 / M_PI;
            ai2vcu_data.AI2VCU_AXLE_SPEED_REQUEST_rpm = speed / (0.253 * 2 * M_PI) * 60;  // wheel radius is 0.253m
        }

        fs_ai_api::ai2vcu_set_data(ai2vcu_data);
        geometry_msgs::Twist vel_feedback;
        vel_feedback.angular.z = vcu2ai_data.VCU2AI_STEER_ANGLE_deg * M_PI / 180.0;
        vel_feedback.linear.x = vcu2ai_data.VCU2AI_RR_WHEEL_SPEED_rpm * 0.253 * 2 * M_PI / 60;

        ros_can_pub.publish(vel_feedback);
    }

    fs_ai_api_mission_status_e getMissionStatus(const fs_ai_api::fs_ai_api_vcu2ai& data) {
        switch (data.VCU2AI_AS_STATE) {
            case fs_ai_api_as_state_e::AS_OFF:
                // Check whether a mission has been chosen, and acknowledge it if so
                if (data.VCU2AI_AMI_STATE != fs_ai_api_ami_state_e::AMI_NOT_SELECTED)
                    return fs_ai_api_mission_status_e::MISSION_SELECTED;
                else
                    return fs_ai_api_mission_status_e::MISSION_NOT_SELECTED;
            case fs_ai_api_as_state_e::AS_READY:
                if (driving_flag) {
                    // our stack is ready to start driving
                    return fs_ai_api_mission_status_e::MISSION_RUNNING;
                } else {
                    // still not ready to start driving
                    return fs_ai_api_mission_status_e::MISSION_SELECTED;
                }
            case fs_ai_api_as_state_e::AS_DRIVING:
                if (mission_complete) {
                    // mission has been finished
                    return fs_ai_api_mission_status_e::MISSION_FINISHED;
                } else {
                    // still doing a mission
                    return fs_ai_api_mission_status_e::MISSION_RUNNING;
                }
            case fs_ai_api_as_state_e::AS_FINISHED:
                return fs_ai_api_mission_status_e::MISSION_FINISHED;
            default:
                return fs_ai_api_mission_status_e::MISSION_NOT_SELECTED;
        }
    }

    ros::Subscriber cmd_vel_sub;
    ros::Publisher ros_can_pub;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_can");

    RosCan roscan(argv);

    ros::spin();

    return 0;
}

