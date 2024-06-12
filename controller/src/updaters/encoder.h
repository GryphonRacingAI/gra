#pragma once

#include "../utils/time.h"
#include "../utils/base_state_updater.h"

#include "../states/motor_state.h"

/**
 * An encoder updater that updates position, velocity, acceleration from encoder readings
 */
class Encoder : public BaseStateUpdater<MotorState> {

    private:

        volatile long delta_ticks;
        double last_velocity = 0;
        DynamicRate rate;

    public:

        Encoder(MotorState& state) : BaseStateUpdater<MotorState>(state), rate(state.target_update_rate) {
            // Initialize pins
            pinMode(state.hall_a_pin, INPUT);
            pinMode(state.hall_b_pin, INPUT);
        }

        /**
         * Predictively update the state based on the current state
         * 
         * @param time_elapsed The time elapsed since the last update
         */
        void predictive_update(double &time_elapsed) {
            // Method 1: We assume acceleration is constant (this potentially makes it more accurate, but may be impacted by noise)
            if (state.second_order_predictor) {
                state.velocity =
                    state.velocity +
                    state.acceleration * time_elapsed; // Update velocity in state
                state.position =
                    state.position +
                    state.velocity * time_elapsed; // Update position in state
            } else { // Method 2: We assume velocity is constant, which is more
                        // robust to noise. However, acceleration will be left
                        // unmodified so its not consistent.
                state.position =
                    state.position +
                    state.velocity * time_elapsed; // Update position in state
            }
        }

        /**
         * If enough time has elapsed, read and reset delta_ticks, and update the state.
        */
        void update(Tick &tick) {

            double time_elapsed = rate.update(tick); // See if enough time has elapsed

            if (time_elapsed >= 0) { // If it has

                // First, we get the newest delta_ticks
                noInterrupts();
                state.delta_ticks = delta_ticks; // Update delta_ticks in state
                delta_ticks = 0; // Reset ISR delta_ticks
                interrupts();

                // Then, we calculate the preliminary velocity and acceleration
                state.update_time = tick.time; // Update time in state
                state.total_ticks += state.delta_ticks; // Add to total ticks
                double draft_velocity = (double) state.delta_ticks / state.ppr * state.gear_ratio / time_elapsed * 2. * M_PI; // Calculate a draft velocity
                double draft_acceleration = (draft_velocity - last_velocity) / time_elapsed; // Calculate a draft acceleration
                last_velocity = draft_velocity; // Update last velocity because after this point last_velocity is not used

                if (abs(draft_acceleration) < state.max_abs_acceleration && abs(draft_velocity) < state.max_abs_velocity) {

                    // If the acceleration and velocity is small enough, we can update the state
                    state.velocity = draft_velocity; // Update velocity in state
                    state.acceleration = draft_acceleration; // Update acceleration in state
                    state.position = state.position + state.velocity * time_elapsed; // Update position in state
                    state.discarded = false;
                    double resolution = 1. / state.ppr * state.gear_ratio / time_elapsed * 2. * M_PI; // Calculate the resolution of the encoder
                    // Set the deadband
                    state.deadband_min = -resolution + 0.0001;
                    state.deadband_max = resolution - 0.0001;

                } else {

                    // Otherwise, if it is bigger than a certain threshold, we discard the read data, and predict the position, velocity and acceleration instead
                    predictive_update(time_elapsed);
                    state.discarded = true;

                }

                state.encoder_dt = time_elapsed; // Update fresh_update in state

            } else {

                state.encoder_dt = -1; // Update fresh_update in state

            }
        }

        /**
         * Interrupt handler for hall A. Must be manually attached in the main file.
        */
        void hall_a_interrupt() {

            // If hall B is high, we're going forward -- OUTDATED!
            // If hall B is low, we're going backward -- New, because PCB has flipped the hall sensor pins
            if (digitalRead(state.hall_b_pin) == LOW) {
                delta_ticks++;
            } else {
                delta_ticks--;
            }

        }
        
};