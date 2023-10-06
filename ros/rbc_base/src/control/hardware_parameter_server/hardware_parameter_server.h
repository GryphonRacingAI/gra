#include "ros/ros.h"
#include "rbc_base/BaseParameters.h"
#include "rbc_base/BaseAdaptiveState.h"
#include "rbc_base/BaseState.h"

#define NUM_MOTORS 4

class HardwareParameterServer {
    private:
        rbc_base::BaseParameters base_parameters_msg;
        rbc_base::BaseAdaptiveState base_adaptive_state_msg;
        ros::Publisher base_parameters_pub;
        ros::Publisher base_adaptive_state_pub;
        ros::NodeHandle _nh;
        // Generic type method for initializing a parameter
        template <typename T>
        void initParam(T& param, int motor_id, std::string name, T default_value) {
            _nh.param<T>("motor_" + std::to_string(motor_id + 1) + "/" + name, param, default_value);
        }
        // Create integer array of size NUM_MOTORS to store motor mode
        int motor_mode[NUM_MOTORS] = {0};
    public:
        HardwareParameterServer() : _nh("hardware_parameter_server") {
            
            // Initialize the publisher and subscriber
            base_parameters_pub = _nh.advertise<rbc_base::BaseParameters>("/base_parameters", 1000);
            base_adaptive_state_pub = _nh.advertise<rbc_base::BaseAdaptiveState>("/base_adaptive_state", 1000);

            for (int i = 0; i < NUM_MOTORS; i++) {
                initParam<float>(base_parameters_msg.parameters[i].p_in, i, "p_in", 35.0);
                initParam<float>(base_parameters_msg.parameters[i].i_in, i, "i_in", 0.0015);
                initParam<float>(base_parameters_msg.parameters[i].d_in, i, "d_in", 0.0);
                initParam<float>(base_parameters_msg.parameters[i].bias, i, "bias", 50.0);
                initParam<int>(motor_mode[i], i, "control_mode", 3);
                initParam<float>(base_adaptive_state_msg.adaptive_states[i].i_accumulator, i, "i_accumulator_initial", 20.0);
            }
            
            ROS_INFO("Waiting for /base_state to be published...");
            ros::topic::waitForMessage<rbc_base::BaseState>("/base_state");
            ROS_INFO("Received /base_state message, publishing parameters /base_parameters and /adaptive_state");

            // Wait 1 second for the subscriber to connect

            // syncBoolToUint();
            base_parameters_pub.publish(base_parameters_msg);
            base_adaptive_state_pub.publish(base_adaptive_state_msg);
        }
};