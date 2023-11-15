#pragma once
#include <Arduino.h>

/**
 * Tick is a struct that contains the time since the last update in seconds, and absolute time in microseconds.
*/
struct Tick {
    double dt; // 
    long time; // Time in microseconds
};


/**
 * Clock is a class that keeps track of time since the last update. Meant to be used in a best-effort loop.
*/
class Clock {
    private:
        long last_update_time;
    public:
        Clock() {
            last_update_time = 0;
        }
        Tick update() {
            long current_time = micros();
            double dt = (double) (current_time - last_update_time) / 1000000.;
            last_update_time = current_time;
            return { dt, current_time };
        }
};

/**
 * DynamicRate is meant to be used in a best-effort loop to call a function at a certain rate.
 * It uses references to the target frequency and the time elapsed to avoid unnecessary copies.
 * 
 * @param target_freq A reference to the frequency to call the callback function at in Hz.
*/
class DynamicRate {
    private:
        double& target_freq;
        double time_elapsed = 0;
        bool running = false;
    public:
        DynamicRate(double& freq, bool start = true) : target_freq(freq) {
            if (start) {
                this->start();
            }
        }
        ~DynamicRate() {}
        /**
         * Updates the rate
         * 
         * @param tick The tick struct from the clock.
         * @return The time elapsed since the last update, or -1 if the rate has not been reached.
        */
        double update(Tick tick) {
            time_elapsed += tick.dt;
            double target_dt = 1. / target_freq;
            if (time_elapsed >= target_dt) {
                double time_elapsed_copy = time_elapsed;
                time_elapsed = 0;
                return time_elapsed_copy;
            } else {
                return -1;
            }
        }
        void stop() {
            running = false;
        }
        void start() {
            running = true;
        }
};