#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

#include "encoder.h"
#include "controller.h"
#include "raw_motor.h"
#include "acceleration_bounder.h"

/**
 * Motor class that contains all the sub-updaters for closed-loop control
 */
class Motor : public BaseStateUpdater<MotorState> {
    public:
        Encoder encoder;
        Controller controller;
        RawMotor raw_motor;
        AccelerationBounder acceleration_bounder;
        
        Motor(MotorState& state) : BaseStateUpdater<MotorState>(state), acceleration_bounder(AccelerationBounder(state)), encoder(Encoder(state)), controller(Controller(state)), raw_motor(RawMotor(state)) {}
        void update(Tick& tick) {
            acceleration_bounder.update(tick);
            encoder.update(tick);
            controller.update(tick);
            raw_motor.update(tick);
        }
};