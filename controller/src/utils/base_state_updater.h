#pragma once
#include "time.h"

/**
 * How to use BaseStateUpdater:
 *
 * This class is meant to be used as a framework to aid creating complex time-dependent update chains, and promote composition,
 * ease sharing variables between updaters, and be more readible and maintainable.
 * 
 * States are logical collections of variables organized in structs that are updated over time. For example, our closed-loop controlled
 * motors can be thought of as a collection of PID parameters, setpoint, position, velocity, etc. These variables could be interdependent,
 * but most importantly the contents of the structs are useful information that NEEDS to be shared between different processes. For example,
 * the encoder first computes the velocity of the motor, and then the controller requires the velocity to compute the output, then the raw
 * motor requires the output to set the PWM. The struct should contain information that needs to be shared between processes, but variables
 * only useful for internal computation should be kept encapsulated in updaters.
 * 
 * Updaters are classes that update a state over time. They encapsulate the logic for updating a state within class variables and methods.
 * 
 * The whole system is dependent on using references to the state structs. We have to be careful to not copy the state structs, and also not
 * to delete the state structs while updaters are still using them.
*/

/**
 * Base class for all state updaters. This class is meant to be inherited from, and not used directly. Read source code for more info.
 * 
 * @tparam T The state type to update.
*/
template <typename T>
class BaseStateUpdater {
    public:
        T& state;
        BaseStateUpdater(T& state) : state(state) {}
        void update(Tick& tick) {
        }
};