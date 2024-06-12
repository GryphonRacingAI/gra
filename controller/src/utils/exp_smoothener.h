#pragma once
#include <Arduino.h>

class ExpSmoothener {
public:
    /**
     * Construct an ExpTimeSmoothener object.
     *
     * @param alpha A double representing the alpha rate. A value closer to 1 will retain more past data,
     *              while a value closer to 0 will react more to recent data.
     */
    ExpSmoothener(double& alpha) : _alpha(alpha), _first(true) {}

    /**
     * Update the value to be smoothed.
     *
     * @param value The new raw data value to be included in the smoothed sequence.
     * @param dt The time since the last update, in seconds.
     * @return The smoothed value.
     */
    double update(double value, double dt) {
        // Calculate the alpha factor, such that the value alphas by _alpha in 1 second
        double alpha = pow(_alpha, dt);

        if (_first) {
            _first = false;
            _last_value = value;
        } else {
            // Apply the alpha factor
            _last_value = _alpha * value + (1 - _alpha) * _last_value;
        }
        return _last_value;
    }

private:
    double _last_value;  // The last smoothed value
    double& _alpha;  // The alpha rate
    bool _first;  // A flag indicating whether this is the first value in the sequence
};
