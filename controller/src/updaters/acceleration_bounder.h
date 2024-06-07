#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

#include "../defs.h"

/**
 * This class keeps setpoint velocity within acceleration bounds as to not fry the driver chips
 */
class AccelerationBounder : public BaseStateUpdater<MotorState> {
    private:
        
    public:
        AccelerationBounder(MotorState& state) : BaseStateUpdater<MotorState>(state) {
        }
        void update(Tick& tick) {
            // Calculate the maximum and minimium setpoint velocity allowed by the acceleration limit
            double max_setpoint = state.last_setpoint + state.acceleration_limit * tick.dt;
            double min_setpoint = state.last_setpoint - state.acceleration_limit * tick.dt;
            // Constrain the setpoint velocity to the acceleration limit
            state.acceleration_bounded_setpoint = constrain(state.setpoint, min_setpoint, max_setpoint);
            // Update the last setpoint
            state.last_setpoint = state.acceleration_bounded_setpoint;
        }
};