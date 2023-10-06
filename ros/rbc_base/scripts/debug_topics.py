#!/usr/bin/env python3

from __future__ import annotations
import rospy

from rbc_base.msg import BaseState, BaseSetpoint
from std_msgs.msg import Float32

import threading

def main():
    
    motors = 4 # Default to 4 motors
    base_state_topics(motors)
    thread = base_setpoint_topics(motors)

    # Spin
    rospy.spin()
    thread.join()

def base_state_topics(motors):
    properties = ['i_accumulator', 'output', 'error', 'delta_ticks', 'velocity', 'position', 'acceleration', 'setpoint'] # Define the properties to publish
    pubs: list[rospy.Publisher] = [] # List of publishers

    for i in range(motors): # We create a publisher for each motor and each property
        for prop in properties:
            pub = rospy.Publisher('motor_' + str(i+1) + '_' + prop, Float32, queue_size=10)
            pubs.append(pub)

    def callback(msg: BaseState): # Handling incoming messages in /base_state and splitting them to individual topics
        # Split the message into individual topics
        for i in range(motors):
            for prop in properties:
                pub = pubs[i*len(properties) + properties.index(prop)]
                if prop == 'setpoint': # Setpoint is a special case
                    pub.publish(msg.states[i].error + msg.states[i].velocity)
                else:
                    # Print properties of motor 0
                    pub.publish(msg.states[i].__getattribute__(prop))

    rospy.init_node('debug_topics')
    sub = rospy.Subscriber('base_state', BaseState, callback)
    rospy.loginfo(f"Subscribed to /base_state, publishing {motors*len(properties)} topics in format /motor_<motor>_<property>")
    rospy.loginfo(f"Available properties: {properties}")

def base_setpoint_topics(motors):
        # Create a publisher for the BaseSetpoint message
    pub = rospy.Publisher('base_setpoint', BaseSetpoint, queue_size=10)

    # Create a BaseSetpoint message
    base_setpoint_msg : BaseSetpoint = BaseSetpoint()

    # Create a list to store individual motor setpoints
    individual_setpoints = []

    # Create subscribers for individual motor setpoints
    def callback(msg: Float32, i: int):
        individual_setpoints[i] = msg.data

    for i in range(motors):
        individual_setpoints.append(0)
        rospy.Subscriber('motor_' + str(i+1) + '_setpoint', Float32, callback, i)

    # Create a thread to publish the BaseSetpoint message
    def publish_setpoint():
        while not rospy.is_shutdown():
            base_setpoint_msg.setpoints = individual_setpoints
            pub.publish(base_setpoint_msg)

    # Start the thread
    thread = threading.Thread(target=publish_setpoint)
    thread.start()
    return thread

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass