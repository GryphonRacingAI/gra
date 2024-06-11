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
Ki = 0.1
Kd = 0.01

class SpeedController:
    def __init__(self):
        rospy.init_node('wheel_speed_controller', anonymous=False)
        
        # PID variables
        self.integral = 0.0
        self.previous_error = 0.0
        
        # Desired speed
        self.desired_speed = 0.0
        
        # Subscribers
        rospy.Subscriber('/vcu2ai', VCU2AI, self.vcu2ai_callback)
        rospy.Subscriber('/ackermann_cmd', AckermannDrive, self.ackermann_cmd_callback)
        
        # Publisher
        self.pub = rospy.Publisher('/ackermann_cmd_PID', AckermannDrive, queue_size=10)
    
    def vcu2ai_callback(self, msg):
        # Get the actual rear wheel speeds (in RPM)
        rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm
        
        # Compute the average wheel speed
        actual_wheel_speed_rpm = (rl_wheel_speed_rpm + rr_wheel_speed_rpm) / 2.0
        
        # Convert wheel speed to car speed (m/s)
        actual_speed_mps = (actual_wheel_speed_rpm * 2 * math.pi * WHEEL_RADIUS) / 60.0
        
        # PID control
        error = self.desired_speed - actual_speed_mps
        self.integral += error
        derivative = error - self.previous_error
        output = Kp * error + Ki * self.integral + Kd * derivative
        self.previous_error = error
        
        # Create and publish the new AckermannDrive message
        cmd = AckermannDrive()
        cmd.speed = output
        self.pub.publish(cmd)
    
    def ackermann_cmd_callback(self, msg):
        # Get the desired speed (m/s)
        self.desired_speed = msg.speed

if __name__ == '__main__':
    try:
        controller = SpeedController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
