#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include "rbc_base/BaseState.h"
#include "rbc_base/BaseSetpoint.h"
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>

#define NUM_MOTORS 3

class RobotHW : public hardware_interface::RobotHW {
    private:
        hardware_interface::JointStateInterface jnt_state_interface; // This is the interface for reading the state of the joints
        hardware_interface::VelocityJointInterface jnt_vel_interface; // This is the interface for commanding the joints

        // We then need to store the state and command of the joints
        double cmd[3] = {0.0, 0.0, 0.0};
        double pos[3] = {0.0, 0.0, 0.0};
        double vel[3] = {0.0, 0.0, 0.0};
        double eff[3] = {0.0, 0.0, 0.0};

        // ROS node
        ros::Publisher wheel_vel_pub;
        ros::Subscriber wheel_state_sub;

        void baseStateCallback(const rbc_base::BaseState& msg) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                pos[i] = msg.states[i].position;
                vel[i] = msg.states[i].velocity;
                eff[i] = msg.states[i].output;
            }
        }

        float control_frequency;
    
    public:
        ros::NodeHandle nh;
        ros::Rate* control_rate;
        RobotHW() {
            ROS_INFO("Initializing hardware interface");

            // Register the joints with the state and velocity interfaces
            for (int i = 0; i < NUM_MOTORS; i++) {
                std::string joint_name = "wheel_" + std::to_string(i + 1) + "_joint";
                hardware_interface::JointStateHandle state_handle(joint_name, &pos[i], &vel[i], &eff[i]);
                jnt_state_interface.registerHandle(state_handle);
            }

            registerInterface(&jnt_state_interface);

            for (int i = 0; i < NUM_MOTORS; i++) {
                std::string joint_name = "wheel_" + std::to_string(i + 1) + "_joint";
                hardware_interface::JointHandle vel_handle(jnt_state_interface.getHandle(joint_name), &cmd[i]);
                jnt_vel_interface.registerHandle(vel_handle);
            }
            registerInterface(&jnt_vel_interface);

            // Initialize the publisher and subscriber
            wheel_vel_pub = nh.advertise<rbc_base::BaseSetpoint>("base_setpoint", 1000);
            wheel_state_sub = nh.subscribe("base_state", 1000, &RobotHW::baseStateCallback, this);

            // Get the controller frequency
            nh.param<float>("control_frequency", control_frequency, 100.0);

            // Initialize the control rate
            control_rate = new ros::Rate(control_frequency);

            ROS_INFO("Hardware interface initialized");
        }
        void write() {
            // Publish the target wheel velocities
            rbc_base::BaseSetpoint msg;
            for (int i = 0; i < NUM_MOTORS; i++) {
                msg.setpoints[i].velocity = cmd[i];
            }
            wheel_vel_pub.publish(msg);
        }
        void read() {
        }
        ~RobotHW() {
            delete control_rate;
        }
};