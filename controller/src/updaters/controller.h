#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

/**
 * This class is a custom adaptive PID controller that has been heavily
 * modified to work well for our system with the observation that PWM is 
 * roughly proportional to the measured speed of the motor.
 * 
 * We originally used the ArduPID library, but this was rewritten for better
 * readability, and to allow for more runtime configuration.
 * 
 * 
 * Compared to the ArduPID implementation, our code does the following differently:
 * 
 * 
 * *** Bias term is multiplied with the sign of the setpoint ***
 * 
 * I interpret this as adding the minimum PWM value to get the motor to start moving
 * 
 * 
 * *** The accumulated integral value is multiplied with the setpoint before being added to the output ***
 * 
 * Out motor has a fast response time and little inertia, intuitively it makes sense that at setpoint 0,
 * the integral term should be zero as well. 
 * 
 * Also makes use of the assumption that the motor speed is roughly proportional to the PWM value.
 * Effectively, this means that the controller converges on creating a coefficient that forms a linear
 * relationship between the setpoint and the measured value.
 * 
 * Under this new scheme, the I term becomes the main contributor to the output. The controller would be able 
 * to work even without the P and D terms (purely adaptive feedforward control), but with the addition of the two 
 * terms, the controller is able to converge faster and with less overshoot. Moreover, the controller is able to
 * adapt to changes in the system, such as a change in the load on the motor.
 * 
 * However, note that with this modification, the I term contributes a lot more especially when the setpoint is high.
 * A smaller value should be chosen for stability. The P term should be tuned similarly to a normal PID controller.
 *
 * In regular PID systems, the I term accumulator is usually initialized to 0. However in our system this represents
 * the linear relationship between the setpoint and the measured value. Therefore, if it is initialized to 0, the
 * controller takes a long time to slowly converge on the correct value. To speed up the process, the accumulator
 * should be initialized to a value that is close to the correct value. Perhaps having persistent storage of the
 * accumulator value would be useful. It could be implemented by storing the value in EEPROM, and also would be useful
 * for ROS to be able to change the accumulator value.
*/

#define OPEN_LOOP 0 // No feedback control
#define CLASSIC_PID 1 // Original ArduPID implementation
#define CLASSIC_PID_B_I_FLIPPING 2 // Bias and integral term flips polarity when setpoint changes sign
#define ADAPTIVE_PID 3 // I is multiplied with setpoint before being added to output

/**
 * A custom adaptive PID controller that updates motor output based on the measured speed of the motor
 */
class Controller : public BaseStateUpdater<MotorState> {
    private:
        double last_error = 0;
        short last_setpoint_polarity = 0;
    public:
        Controller(MotorState& state) : BaseStateUpdater<MotorState>(state) {}
        void update(Tick& tick) {
            if (state.control_mode == OPEN_LOOP) {
                state.output = state.setpoint;
                state.error = 0;
                last_error = 0;
                last_setpoint_polarity = 0;
                return;
            }
            if (state.encoder_dt >= 0) { // Only update if there is a fresh encoder reading. Otherwise, the readings mean nothing!
                state.error = state.acceleration_bounded_setpoint - state.velocity; // Calculate error
                if (state.error >= state.deadband_min && state.error <= state.deadband_max) {
                    state.error = 0;
                }
                double p_out = state.p_in * state.error; // Calculate proportional term
                double d_out = -state.d_in * state.acceleration; // Calculate derivative term
                // Calculate integral term
                // Polarity: -1 if setpoint is negative, 1 if setpoint is positive, 0 if setpoint is 0
                double setpoint_polarity;
                if (state.control_mode == CLASSIC_PID) {
                    setpoint_polarity = 1;
                    last_setpoint_polarity = 1;
                } else { // CLASSIC_PID_B_I_FLIPPING or ADAPTIVE_PID
                    setpoint_polarity = (state.acceleration_bounded_setpoint == 0) ? 0 : (state.acceleration_bounded_setpoint >= 0) ? 1 : -1;
                }

                double bias_out = state.bias * setpoint_polarity; // Calculate bias term

                double i_temp = state.i_accumulator + (state.i_in * state.encoder_dt * 
                    (state.error * setpoint_polarity + last_error * last_setpoint_polarity) / 2 // Trapezoidal integration
                );

                // Constrain using windup limits
                i_temp = constrain( i_temp, state.i_accumulator_min, state.i_accumulator_max ); // Temporary variable before we constrain with limits

                // Constrain using output limits, so the integral term doesnt grow if we saturate the output
                double pdb_out_temp = p_out + d_out + bias_out; // Calculate output without integral term
                if ( state.control_mode == CLASSIC_PID || state.control_mode == CLASSIC_PID_B_I_FLIPPING ) {
                    double i_out_max = constrain(state.output_max - pdb_out_temp, 0, state.output_max); // We constrain the integral term to the range of the output just to be safe
                    double i_out_min = constrain(state.output_min - pdb_out_temp, state.output_min, 0); 
                    i_temp = constrain( i_temp, i_out_min, i_out_max );
                } else if ( state.control_mode == ADAPTIVE_PID && state.acceleration_bounded_setpoint != 0 ) {
                    // double i_out_max = constrain((state.output_max - pdb_out_temp) / state.setpoint, 0, state.output_max); // We constrain the integral term to the range of the output just to be safe
                    // double i_out_min = constrain((state.output_min - pdb_out_temp) / state.setpoint, state.output_min, 0);
                    // i_temp = constrain( i_temp, i_out_min, i_out_max );
                }
                state.i_accumulator = i_temp;

                // Calculate output
                double temp_out;
                if ( state.control_mode == CLASSIC_PID || state.control_mode == CLASSIC_PID_B_I_FLIPPING ) {
                    temp_out = p_out + d_out + state.i_accumulator * setpoint_polarity + bias_out;
                } else if ( state.control_mode == ADAPTIVE_PID ) {
                    temp_out = p_out + d_out + state.i_accumulator * state.acceleration_bounded_setpoint + bias_out;
                }

                // Constrain using output limits
                state.output = constrain( temp_out, state.output_min, state.output_max );

                last_error = state.error;
                last_setpoint_polarity = setpoint_polarity;
            }
        }
};