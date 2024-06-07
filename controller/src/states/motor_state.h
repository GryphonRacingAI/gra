#pragma once
#include <Arduino.h>

struct MotorState {
    // *** Hardware configuration ***
    int lpwm_pin;
    int rpwm_pin;
    int hall_a_pin;
    int hall_b_pin;
    double gear_ratio;
    int ppr;
    int isense_pin;

    // *** Controller settings ***
    int control_mode = 3;
    double p_in = 40.0;
    double i_in = 15.0;
    double d_in = 0.0;
    double bias = 20.0;
    double i_accumulator_min = 0;
    double i_accumulator_max = 100;
    double output_min = -255;
    double output_max = 255;
    double deadband_min = 0; // These are automatically updated by the encoder updater
    double deadband_max = 0;

    // *** Controller state ***
    double i_accumulator = 20;
    double setpoint = 0;
    double last_setpoint = 0;
    double acceleration_bounded_setpoint = 0;
    double acceleration_limit = 10;
    double output = 0;
    double error = 0;
    int icurrent = 0;

    // *** Encoder settings ***
    double target_update_rate = 20; // The target update rate in Hz
    double max_abs_acceleration = 200.; // Threshold of acceleration to discard the update
    double max_abs_velocity = INFINITY; // Threshold of velocity to discard the update
    bool second_order_predictor = false; // Whether to use a second order predictor, read encoder source code for more info

    // *** Encoder state ***
    bool discarded = false; // Whether the last update was discarded due to filtering
    double encoder_dt = -1; // The encoder tick accumulation window in seconds (-1 if it was not sampled for this update)
    // Unfiltered raw values
    long update_time = 0; // The last micros() value when the encoder was updated
    long delta_ticks = 0;  // The number of ticks since the last update
    long total_ticks = 0; // The total number of ticks since the controller was initialized
    // Filtered values
    double position = 0; // The current position in radians
    double velocity = 0; // The current velocity in radians/s
    double acceleration = 0; // The current acceleration in radians/s^2

    /** Raw motor settings */
    double smoothener_alpha = 0.001;
};