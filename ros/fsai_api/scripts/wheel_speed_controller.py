#!/usr/bin/env python3

import rospy
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI
import math

# Constants
WHEEL_RADIUS = 0.2575
MOTOR_RATIO = 3.5

# PID controller parameters
Kp = 0.5
Ki = 0.02
# Kd = 0.01

class SpeedController:
    def __init__(self):
        rospy.init_node('wheel_speed_controller', anonymous=False)
        
        # PID variables
        self.integral = 1.1/Ki # experimentally determined using static operation
        # self.previous_error = 0.0
        
        # Desired speed
        self.desired_speed = 0.0
        self.desired_steering = 0.0
        
        # Subscribers
        rospy.Subscriber('/vcu2ai', VCU2AI, self.vcu2ai_callback)
        rospy.Subscriber('/ackermann_cmd_controller', AckermannDrive, self.ackermann_cmd_callback)
        
        # Publisher
        self.ackermann_cmd_publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.controlled_ackermann_publish)
        self.actual_speed_mps = 0
    
    def vcu2ai_callback(self, msg):
        # Get the actual rear wheel speeds (in RPM)
        rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm
        
        # Compute the average wheel speed
        actual_wheel_speed_rpm = (rl_wheel_speed_rpm + rr_wheel_speed_rpm) / 2.0
        
        # Convert wheel speed to car speed (m/s)
        self.actual_speed_mps = (actual_wheel_speed_rpm * 2 * math.pi * WHEEL_RADIUS) / 60.0

    
    def controlled_ackermann_publish(self, timer):
        cmd = AckermannDrive()
        cmd.steering_angle = self.desired_steering
        if self.desired_speed > 0.05:
            # PID control
            error = self.desired_speed - self.actual_speed_mps

            self.integral += self.constrain(error, -1, 1)
            # derivative = error - self.previous_error
            output_with_feedback = self.desired_speed + Kp * error + Ki * self.integral # + Kd * derivative
            # self.previous_error = error
            
            # Create and publish the new AckermannDrive message
            cmd.speed = output_with_feedback
            rospy.loginfo(f"e: {error:.2f} \t x: {self.desired_speed:.2f} + p {Kp * error:.2f} + i {Ki * self.integral:.2f} = {output_with_feedback:.2f}")
        else:
            cmd.speed = self.desired_speed
        self.integral = self.constrain(self.integral*Ki, 0.4, 2.5)/Ki
        
        self.ackermann_cmd_publisher.publish(cmd)

    def constrain(self, x, minimum_x, maximum_x):
        if x < minimum_x:
            return minimum_x
        if x > maximum_x:
            return maximum_x
        return x

    def ackermann_cmd_callback(self, msg):
        # Get the desired speed (m/s)
        self.desired_speed = msg.speed
        self.desired_steering = msg.steering_angle

if __name__ == '__main__':
    try:
        controller = SpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
