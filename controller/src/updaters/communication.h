#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"

#include <ros.h>
#include <std_msgs/String.h>
#include <rbc_base/BaseState.h>
#include <rbc_base/BaseSetpoint.h>
#include <rbc_base/BaseParameters.h>
#include <rbc_base/BaseAdaptiveState.h>
#include <rbc_base/MotorParameters.h>
#include <rbc_base/MotorAdaptiveState.h>

#include "../defs.h"

/**
 * This class communicates the internal controller states and parameters to ROS via rosserial
 */
class Communication : public BaseStateUpdater<RobotState> {
    private:
        // Create node handle
        ros::NodeHandle nh;

        // Create the message objects
        rbc_base::BaseState base_state_msg;
        rbc_base::BaseSetpoint base_setpoint_msg;

        // Wheel state publisher
        ros::Publisher base_state_publisher = ros::Publisher("base_state", &base_state_msg);

        bool debug_loop_bool = false;

        /** Subscribing to topics */

        // "/base_setpoint" topic subscriber, used to set the setpoint of the motors independently
        void baseSetpointCallback(const rbc_base::BaseSetpoint& msg) {
            for (int i = 0; i < NUM_MOTORS; i++) {
                state.motors[i].setpoint = msg.setpoints[i].velocity;
            }
        }
        ros::Subscriber<rbc_base::BaseSetpoint, Communication> base_setpoint_sub = ros::Subscriber<rbc_base::BaseSetpoint, Communication>("base_setpoint", &Communication::baseSetpointCallback, this);

        // // "/base_parameters" topic subscriber, used to set the PID parameters of the motors independently, but may be less reliable because of the big message size and rosserial's limitations
        // void baseParametersCallback(const rbc_base::BaseParameters& msg) {
        //     nh.loginfo("Received base parameters");
        //     state.motors[0].p_in = 50;
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].p_in = msg.parameters[i].p_in;
        //         state.motors[i].i_in = msg.parameters[i].i_in;
        //         state.motors[i].d_in = msg.parameters[i].d_in;
        //         state.motors[i].bias = msg.parameters[i].bias;
        //     }
        // }
        // ros::Subscriber<rbc_base::BaseParameters, Communication> base_parameters_sub = ros::Subscriber<rbc_base::BaseParameters, Communication>("base_parameters", &Communication::baseParametersCallback, this);
        
        // // "/base_global_parameters" topic subscriber, used to set the PID parameters of all motors at once
        // void baseGlobalParametersCallback(const rbc_base::MotorParameters& msg) {
        //     nh.loginfo("Received global parameters");
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].p_in = msg.p_in;
        //         state.motors[i].i_in = msg.i_in;
        //         state.motors[i].d_in = msg.d_in;
        //         state.motors[i].bias = msg.bias;
        //         state.motors[i].control_mode = msg.control_mode;
        //     }
        // }
        // ros::Subscriber<rbc_base::MotorParameters, Communication> base_global_parameters_sub = ros::Subscriber<rbc_base::MotorParameters, Communication>("base_global_parameters", &Communication::baseGlobalParametersCallback, this);

        // // "/base_adaptive_state" topic subscriber, used to set the adaptive state of the motors independently
        // void baseAdaptiveStateCallback(const rbc_base::BaseAdaptiveState& msg) {
        //     nh.loginfo("Received adaptive state");
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].i_accumulator = msg.adaptive_states[i].i_accumulator;
        //     }
        // }
        // ros::Subscriber<rbc_base::BaseAdaptiveState, Communication> base_adaptive_state_sub = ros::Subscriber<rbc_base::BaseAdaptiveState, Communication>("base_adaptive_state", &Communication::baseAdaptiveStateCallback, this);

        // // "/base_global_adaptive_state" topic subscriber, used to set the adaptive state of all motors at once
        // void baseGlobalAdaptiveStateCallback(const rbc_base::MotorAdaptiveState& msg) {
        //     nh.loginfo("Received global adaptive state");
        //     for (int i = 0; i < NUM_MOTORS; i++) {
        //         state.motors[i].i_accumulator = msg.i_accumulator;
        //     }
        // }
        // ros::Subscriber<rbc_base::MotorAdaptiveState, Communication> base_global_adaptive_state_sub = ros::Subscriber<rbc_base::MotorAdaptiveState, Communication>("base_global_adaptive_state", &Communication::baseGlobalAdaptiveStateCallback, this);

    public:
        Communication(RobotState& state) : BaseStateUpdater<RobotState>(state) {
            nh.getHardware()->setBaud(BAUD_RATE);
            nh.initNode();
            nh.subscribe(base_setpoint_sub);
            // nh.subscribe(base_parameters_sub);
            // nh.subscribe(base_adaptive_state_sub);
            nh.advertise(base_state_publisher);
            // nh.subscribe(base_global_parameters_sub);
            // nh.subscribe(base_global_adaptive_state_sub);
        }
        void update(Tick& tick) {
            // Publish the wheel state
            bool has_movement = false;
            for (int i = 0; i < NUM_MOTORS; i++) {
                base_state_msg.states[i].i_accumulator = state.motors[i].i_accumulator;
                base_state_msg.states[i].output = state.motors[i].output;
                base_state_msg.states[i].error = state.motors[i].error;
                base_state_msg.states[i].delta_ticks = state.motors[i].delta_ticks;
                base_state_msg.states[i].position = state.motors[i].position;
                base_state_msg.states[i].velocity = state.motors[i].velocity;
                base_state_msg.states[i].acceleration = state.motors[i].acceleration;
                base_state_msg.states[i].icurrent = state.motors[i].icurrent;
                // Check if there is any movement, if yes, set the flag
                if (state.motors[i].delta_ticks != 0) {
                    has_movement = true;
                }
            }
            // Flip flop bool
            debug_loop_bool = !debug_loop_bool;
            // Set pin 22 according to the flip flop bool
            digitalWrite(22, debug_loop_bool ? HIGH : LOW);
            // Set pin 13 to HIGH if there is any movement, LOW otherwise
            digitalWrite(13, has_movement ? HIGH : LOW);
            base_state_publisher.publish(&base_state_msg);
            nh.spinOnce();
        }
};