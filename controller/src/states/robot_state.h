#pragma once
#include "motor_state.h"
#include "../defs.h"

struct RobotState {
    MotorState motors[NUM_MOTORS];
};