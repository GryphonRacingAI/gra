#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"
#include "../utils/exp_smoothener.h"

#include "../states/motor_state.h"

/**
 * Actuates the motor using the raw PWM pins from state
 */
class RawMotor : public BaseStateUpdater<MotorState> {
    private:
        ExpSmoothener smoothener;
        
    public:
        RawMotor(MotorState& state) : BaseStateUpdater<MotorState>(state), smoothener(ExpSmoothener(state.smoothener_alpha)) {
            pinMode(state.lpwm_pin, OUTPUT);
            pinMode(state.rpwm_pin, OUTPUT);
        }
        void update(Tick& tick) {
            // double new_output = smoothener.update(state.output, tick.dt);
            double new_output = state.output;
            if (state.output > 0) {
                analogWrite(state.rpwm_pin, new_output);
                analogWrite(state.lpwm_pin, 0);
            } else {
                analogWrite(state.rpwm_pin, 0);
                analogWrite(state.lpwm_pin, -new_output);
            }
        }
};