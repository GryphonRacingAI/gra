#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"
#include "../states/robot_state.h"

#include "motor.h"
#include "communication.h"

#include "../defs.h"

class RobotCore : public BaseStateUpdater<RobotState> {
    public:
        Communication* communication;
        Motor* motors[NUM_MOTORS];
        
        RobotCore(RobotState& state) : BaseStateUpdater<RobotState>(state) {
            // Let's initialize the communication
            communication = new Communication(state);
            // Then initialize all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i] = new Motor(state.motors[i]);
            }
        }
        ~RobotCore() {
            // Delete the communication
            delete communication;
            // Delete all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                delete motors[i];
            }
        }
        void update(Tick& tick) {
            // Let's first update all the motors
            for (int i = 0; i < NUM_MOTORS; i++) {
                motors[i]->update(tick);
            }
            // Then update the communication
            communication->update(tick);
        }
};