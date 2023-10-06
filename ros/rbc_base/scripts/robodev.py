#!/usr/bin/env python3

import copy
import threading
import rospy
from rbc_base.msg import BaseSetpoint, BaseState, MotorParameters, MotorAdaptiveState
import numpy as np
import matplotlib.pyplot as plt
import json
import os
from typing import List, Dict, Union, Callable
import scipy.signal
import pandas as pd
from scipy.fft import fft, fftfreq
from pandas.plotting import autocorrelation_plot
from scipy import signal
from scipy.stats import pearsonr, spearmanr

# Decorator to ensure that the communication has been started
def requires_started(func):
    def wrapper(self, *args, **kwargs):
        if not self.started:
            raise Exception("Communication has not been started")
        return func(self, *args, **kwargs)
    return wrapper

# Enum for control modes
class ControlMode:
    """
    Enum class to represent various control modes.
    """
    OPEN_LOOP = 0 # Directly set PWM
    CLASSIC_PID = 1 # Classic PID
    CLASSIC_PID_W_B_I_FLIPPING = 2 # Classic PID with bias and flipping of I accumulator
    ADAPTIVE_PID = 3 # Custom adaptive PID

class Robodev:
    """
    A class to represent the Robodev robotics device.
    """

    def __init__(self, rate: int = 30, num_wheels: int = 4, state_topic: str = 'base_state',
                 setpoint_topic: str = 'base_setpoint', global_parameter_topic: str = 'base_global_parameters',
                 global_adaptive_state_topic = 'base_global_adaptive_state') -> None:
        """
        Initialize the Robodev object.

        Parameters:
            rate (int): The rate of updates.
            num_wheels (int): The number of wheels the robot has.
            state_topic (str): The state topic of the robot.
            setpoint_topic (str): The setpoint topic of the robot.
            global_parameter_topic (str): The global parameter topic of the robot.
            global_adaptive_state_topic (str): The global adaptive state topic of the robot.
        """
        self.rate = rate
        self.num_wheels = num_wheels
        self.state_topic = state_topic
        self.setpoint_topic = setpoint_topic
        self.global_parameter_topic = global_parameter_topic
        self.global_adaptive_state_topic = global_adaptive_state_topic
        self.latest_time = 0
        self.capture_data = False

        self.started = False

        self.start()

    def wheel_states_callback(self, msg: BaseState) -> None:
        """
        Callback function for the wheel states.

        Parameters:
            msg (BaseState): The incoming message with the base state data.
        """
        self.latest_time = rospy.Time.now().to_sec()
        if self.capture_data:
            self.base_states.append({
                "time": self.latest_time,
            })
            for i in range(self.num_wheels):
                self.base_states[-1]["wheel" + str(i+1)] = (
                    msg.states[i].velocity,
                    msg.states[i].error + msg.states[i].velocity,
                    msg.states[i].output,
                    msg.states[i].position,
                    msg.states[i].acceleration,
                    msg.states[i].error,
                    msg.states[i].i_accumulator,
                    msg.states[i].delta_ticks
                )

    @requires_started
    def set_global_velocity(self, vel: float, timed: bool=False) -> None:
        """
        Set the global velocity of the robot.

        Parameters:
            vel (float): The desired velocity.
            timed (bool): If true, it will follow the set rate and be blocking. If false, it will set the velocity as soon as possible.
        """
        self.vel_msg = BaseSetpoint()
        for setpoint in self.vel_msg.setpoints:
            setpoint.velocity = vel
        self.base_setpoint_pub.publish(self.vel_msg)
        self.base_setpoints.append({
            "time": rospy.Time.now().to_sec(),
            "wheel_velocity": vel
        })
        self.r.sleep()

    @requires_started
    def set_global_control_params(self, p_in: float, i_in: float, d_in: float, bias: float, 
                                  control_mode: int = ControlMode.ADAPTIVE_PID) -> None:
        """
        Set the global control parameters of the robot.

        Parameters:
            p_in (float): The proportional gain.
            i_in (float): The integral gain.
            d_in (float): The derivative gain.
            bias (float): The bias for the controller.
            control_mode (int): The control mode.
        """
        # Create message
        self.global_control_params_msg = MotorParameters()
        self.global_control_params_msg.p_in = p_in
        self.global_control_params_msg.i_in = i_in
        self.global_control_params_msg.d_in = d_in
        self.global_control_params_msg.bias = bias
        self.global_control_params_msg.control_mode = control_mode
        # Publish message
        self.base_global_parameters_pub.publish(self.global_control_params_msg)

    @requires_started
    def set_global_adaptive_state(self, i_accumulator: float) -> None:
        """
        Set the global adaptive state of the robot.
        
        Parameters:
            i_accumulator (float): The i_accumulator value.
        """
        self.global_adaptive_state_msg = MotorAdaptiveState()
        self.global_adaptive_state_msg.i_accumulator = i_accumulator
        self.base_global_adaptive_state_pub.publish(self.global_adaptive_state_msg)

    def start(self):
        if self.started:
            print("Communication already started")
            return
        
        self.started = True

        rospy.init_node('controller_stress_test', anonymous=True)
        rospy.loginfo(f"Waiting for {self.state_topic} to start publishing...")
        rospy.wait_for_message('base_state', BaseState)
        rospy.loginfo(f"Got first message from {self.state_topic}, ready!")

        self.base_setpoint_pub = rospy.Publisher(self.setpoint_topic, BaseSetpoint, queue_size=10)
        self.base_states = []
        self.base_setpoints = []
        self.global_control_params_msg = BaseSetpoint()
        self.r = rospy.Rate(self.rate)
        self.base_state_sub = rospy.Subscriber(self.state_topic, BaseState, self.wheel_states_callback)

        self.base_global_parameters_pub = rospy.Publisher(self.global_parameter_topic, MotorParameters, queue_size=10)
        self.base_global_adaptive_state_pub = rospy.Publisher(self.global_adaptive_state_topic, MotorAdaptiveState, queue_size=10)

        # Spin in a separate thread
        self.thread = threading.Thread(target=rospy.spin)
        self.thread.start()

    def stop(self):
        rospy.signal_shutdown("Test finished")
        self.thread.join()

    def clear_data(self):
        self.base_states = []
        self.base_setpoints = []

    def create_track(self) -> "RampTrack":
        """
        Create a RampTrack object.

        Returns:
            A RampTrack object.
        """
        return RampTrack(self)

    def start_capture(self) -> None:
        """
        Start capturing data.
        """
        self.capture_data = True

    def stop_capture(self) -> None:
        """
        Stop capturing data.
        """
        self.capture_data = False

class RampTrack:
    """
    Class to represent a ramp track.
    """

    def __init__(self, communication: 'Robodev') -> None:
        """
        Initialize the RampTrack object.

        Parameters:
            communication (Robodev): The Robodev object for communication.
        """
        self.communication = communication
        self.track = []

    def add_saw_wave(self, max_vel: float, duration: float, periods: int = 1, 
                     transfer_function: Callable[[float], float] = lambda x: x) -> 'RampTrack':
        """
        Add a saw wave to the track.

        Parameters:
            max_vel (float): The maximum velocity.
            duration (float): The duration of the wave.
            periods (int, optional): The number of periods. Defaults to 1.
            transfer_function (Callable[[float], float], optional): The transfer function. Defaults to identity function.

        Returns:
            The instance of the RampTrack (for chaining operations).
        """
        samples = duration * self.communication.rate
        for i in range(periods):
            ramp = np.concatenate((np.linspace(0, 1, int(samples / 4 / periods)), np.linspace(1, 0, int(samples / 4 / periods)),
                                   np.linspace(0, -1, int(samples / 4 / periods)), np.linspace(-1, 0, int(samples / 4 / periods))))
            self.track.extend(transfer_function(vel) * max_vel for vel in ramp)
        return self

    def add_constant(self, value: float, duration: float) -> 'RampTrack':
        """
        Add a constant value to the track.

        Parameters:
            value (float): The constant value to add.
            duration (float): The duration of the constant value.

        Returns:
            The instance of the RampTrack (for chaining operations).
        """
        # Your add_constant code here...
        samples = duration * self.communication.rate
        self.track.extend([value] * samples)
        return self

    def add_linear(self, start: float, end: float, duration: float) -> 'RampTrack':
        """
        Add a linear ramp to the track.

        Parameters:
            start (float): The start value of the ramp.
            end (float): The end value of the ramp.
            duration (float): The duration of the ramp.

        Returns:
            The instance of the RampTrack (for chaining operations).
        """
        samples = duration * self.communication.rate
        self.track.extend(np.linspace(start, end, samples))
        return self

    def run(self) -> "RampResult":
        """
        Run the track.

        Returns:
            A RampResult object containing the results of the run.
        """
        self.communication.start_capture()
        start_time = self.communication.latest_time
        for vel in self.track:
            self.communication.set_global_velocity(vel, True)
        end_time = self.communication.latest_time
        cropped_base_states = copy.deepcopy(self.communication.base_states)
        cropped_base_setpoints = copy.deepcopy(self.communication.base_setpoints)
        # Remove data before start time and after end time using filter
        cropped_base_states = list(filter(lambda x: x["time"] >= start_time and x["time"] <= end_time, cropped_base_states))
        cropped_base_setpoints = list(filter(lambda x: x["time"] >= start_time and x["time"] <= end_time, cropped_base_setpoints))
        # Remove time offset
        for i in range(len(cropped_base_states)):
            cropped_base_states[i]["time"] -= start_time
        for i in range(len(cropped_base_setpoints)):
            cropped_base_setpoints[i]["time"] -= start_time
        self.communication.stop_capture()
        return RampResult(cropped_base_states, cropped_base_setpoints)
    
    def clear(self) -> 'RampTrack':
        """
        Clear the track.

        Returns:
            The instance of the RampTrack (for chaining operations).
        """
        self.track = []
        return self

    def clear_data(self) -> 'RampTrack':
        """
        Clear the communication data.

        Returns:
            The instance of the RampTrack (for chaining operations).
        """
        self.communication.clear_data()
        return self

class RampResult:
    def __init__(self, wheel_states=[], wheel_commands=[]):
        self.wheel_states = wheel_states
        self.wheel_commands = wheel_commands

    def plot(self, exclude_wheel_indices=[]):
        # Initialize figure and axis
        fig, ax = plt.subplots()

        # Extract command times and velocities
        command_velocities = []
        command_times = []
        for command in self.wheel_commands:
            command_velocities.append(command["wheel_velocity"])
            command_times.append(command["time"])

        # Plot commanded velocities
        ax.plot(command_times, command_velocities, label='Commanded velocities', linestyle='--', color='k')

        # Prepare lists to store wheel state times and velocities
        state_times = []
        wheel_velocities = [[] for _ in range(4)]  # create a list of 4 empty lists

        for state in self.wheel_states:
            state_times.append(state["time"])
            for i in range(4):
                if i not in exclude_wheel_indices:
                    wheel_velocities[i].append(state["wheel" + str(i+1)][0])

        # Plot wheel velocities
        for i in range(4):
            if i not in exclude_wheel_indices:
                ax.plot(state_times, wheel_velocities[i], label='Wheel {} velocities'.format(i+1))

        # Set the y-axis limit to be between the min and max of the commanded velocities
        ax.set_ylim([min(command_velocities), max(command_velocities)])

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity')
        ax.legend(loc='upper right')

        # Display the plot
        plt.show()


    def save(self, filename=None):
        # If no filename is given, use the current time
        if filename is None:
            filename = str(rospy.Time.now().to_sec()) + ".json"
        # Save the data to a file
        with open(filename, 'w') as f:
            json.dump({
                "wheel_states": self.wheel_states,
                "wheel_commands": self.wheel_commands
            }, f, indent=4)
        # Log full path of where the data was saved
        rospy.loginfo("Saved data to {}".format(os.path.abspath(filename)))

    def open(self, filename):
        # Open the data file
        with open(filename, 'r') as f:
            data = json.load(f)
        # Set the wheel states and wheel commands
        self.wheel_states = data["wheel_states"]
        self.wheel_commands = data["wheel_commands"]

    def plot_error(self, exclude_wheel_indices: list = []):
        """Plot the error over time for each wheel."""
        fig, ax = plt.subplots()

        for i in range(1, 5):
            if i in exclude_wheel_indices:
                continue
            error = [state["wheel"+str(i)][6] for state in self.wheel_states]
            time = [state["time"] for state in self.wheel_states]

            ax.plot(time, error, label='Wheel ' + str(i))

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Error')
        ax.legend(loc='upper right')
        plt.show()

    def plot_output_velocity_scatter(self, exclude_wheel_indices: list = []):
        """Plot a scatter of the output and velocity for each wheel."""
        fig, ax = plt.subplots()

        for i in range(1, 5):
            if i in exclude_wheel_indices:
                continue
            output = [state["wheel"+str(i)][2] for state in self.wheel_states]
            velocity = [state["wheel"+str(i)][0] for state in self.wheel_states]

            ax.scatter(output, velocity, label='Wheel ' + str(i))

        ax.set_xlabel('Output')
        ax.set_ylabel('Velocity')
        ax.legend(loc='upper right')
        plt.show()

    def calculate_correlations(self):
        """
        Calculate correlation metrics between setpoint and velocity.

        Returns:
            dict: An array of dictionaries containing Pearson and Spearman correlation coefficients for each wheel.
        """
        results = []
        for i in range(1, 5):
            setpoints = [state["wheel"+str(i)][1] for state in self.wheel_states]
            velocities = [state["wheel"+str(i)][0] for state in self.wheel_states]

            # Calculate Pearson correlation
            pearson_corr, _ = pearsonr(setpoints, velocities)

            # Calculate Spearman correlation
            spearman_corr, _ = spearmanr(setpoints, velocities)

            results.append({
                'pearson': pearson_corr,
                'spearman': spearman_corr
            })

    def plot_setpoint_v_velocity(self, exclude_wheel_indices: list = []):
        """
        Create a scatter plot between setpoint and velocity.
        """
        fig, ax = plt.subplots()

        for i in range(1, 5):
            if i in exclude_wheel_indices:
                continue
            setpoints = [state["wheel"+str(i)][1] for state in self.wheel_states]
            velocities = [state["wheel"+str(i)][0] for state in self.wheel_states]

            ax.scatter(setpoints, velocities, label='Wheel ' + str(i))

        ax.set_xlabel('Setpoint')
        ax.set_ylabel('Velocity')
        ax.legend(loc='upper right')
        plt.show()


    def plot_all(self, exlucde_wheel_indices: list = []):
        """
        Plot all the data in wheel_states.

        Args:
            exlucde_wheel_indices (list): A list of wheel indices to exclude from the plot.
        """
        fig, axs = plt.subplots(7, figsize=(10, 20))
        labels = ["velocity", "setpoint", "output", "position", "acceleration", "error", "i_accumulator", "delta_ticks"]
        
        for i in range(7):
            # data = [state["wheel1"][i] for state in self.wheel_states]
            # axs[i].plot(data)
            # axs[i].set(ylabel=labels[i])
            for j in range(1, 5):
                if j in exlucde_wheel_indices:
                    continue
                data = [state["wheel"+str(j)][i] for state in self.wheel_states]
                axs[i].plot(data, label='Wheel ' + str(j))
                axs[i].set(ylabel=labels[i])
                axs[i].legend(loc='upper right')
        
        plt.xlabel('Time step')
        plt.tight_layout()
        plt.show()
