#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
from fsai_api.msg import VCU2AI

class AutonomousDemonstration:
    WHEEL_CIRCUMFERENCE = 1.64
    def __init__(self):
        rospy.init_node('static_inspection_A', anonymous=False)
        self.ackermann_publisher = rospy.Publisher('/ackermann_cmd', AckermannDrive, queue_size=1)
        self.brake_publisher = rospy.Publisher('/brake', Bool, queue_size=1)
        self.emergency_brake_publisher = rospy.Publisher('/emergency_brake', Bool, queue_size=1)
        self.chequered_flag_publisher = rospy.Publisher('/chequered_flag', Bool, queue_size=1)
        rospy.Subscriber('/vcu2ai', VCU2AI, self.vcu2ai_callback)
        self.rate = rospy.Rate(10)  # 10 Hz

        self.rl_wheel_speed_rpm = None
        self.rr_wheel_speed_rpm = None
        self.rr_pulse_count = None
        self.rl_pulse_count = None

    def vcu2ai_callback(self, msg):
        self.rl_wheel_speed_rpm = msg.rl_wheel_speed_rpm
        self.rr_wheel_speed_rpm = msg.rr_wheel_speed_rpm
        self.rr_pulse_count = msg.rr_pulse_count
        self.rl_pulse_count = msg.rl_pulse_count

    def start(self):
        self.sweep_steering()
        time.sleep(1)
        self.accelerate_10m()
        self.brake_10m()
        self.accelerate_10m()
        self.emergency_brake()

    def sweep_steering(self):
        rospy.loginfo("Starting steering sweep")
        angles = [0.367, -0.367, 0.0]
        start_time = time.time()
        angle = 0
        steering_done = False
        while not rospy.is_shutdown() and not steering_done:
            elapsed_time = time.time() - start_time
            
            if elapsed_time < 2:
                angle = self.linear_interpolate(elapsed_time, 0, 2, 0, angles[0])
            elif 3 < elapsed_time < 5:
                angle = self.linear_interpolate(elapsed_time, 3, 5, angles[0], angles[1])
            elif 6 < elapsed_time < 8.5:
                angle = self.linear_interpolate(elapsed_time, 6, 8, angles[1], angles[2])
            
            if elapsed_time > 9:
                steering_done = True

            ackermann_message = AckermannDrive()
            ackermann_message.steering_angle = angle
            self.ackermann_publisher.publish(ackermann_message)
            rospy.loginfo(f"Set steering angle to {angle}")
            self.rate.sleep()
        rospy.loginfo(f"Steering demo done")
        

    def accelerate_10m(self):
        rospy.loginfo("Starting drivetrain ramp up")
        assert self.rl_pulse_count is not None
        assert self.rr_pulse_count is not None
        start_stopwatch = time.time()
        ACCELERATION = 1.5 # m/s^2
        initial_rl = self.rl_pulse_count
        initial_rr = self.rr_pulse_count
        pulse_count_10m = 10/self.WHEEL_CIRCUMFERENCE*20 # 10m / (WHEEL_CIRCUMFERENCE m / rotation) * 20 (pulse/rotation)

        while not rospy.is_shutdown() and (self.rl_pulse_count < initial_rl + pulse_count_10m) and (self.rr_pulse_count < initial_rr+pulse_count_10m):
            ackermann_message = AckermannDrive()
            elapsed = time.time() - start_stopwatch
            ackermann_message.speed = elapsed * ACCELERATION
            rospy.loginfo(f"Actual speed: {self.rr_wheel_speed_rpm}, {self.rr_wheel_speed_rpm}")
            self.ackermann_publisher.publish(ackermann_message)
            self.rate.sleep()
            if elapsed > 10:
                break
    
    def brake_10m(self):
        ackermann_message = AckermannDrive()
        self.ackermann_publisher.publish(ackermann_message)
        while not rospy.is_shutdown() and self.rl_wheel_speed_rpm > 5:
            self.brake_publisher.publish(True)
            self.rate.sleep()
        self.brake_publisher.publish(False)

    def emergency_brake(self):
        self.emergency_brake_publisher.publish(True)

    @staticmethod
    def linear_interpolate(value, a1, a2, b1, b2):
        if value < a1:
            return b1
        elif value > a2:
            return b2
        else:
            left_span = a2 - a1
            right_span = b2 - b1
            value_scaled = float(value - a1) / float(left_span)
            return b1 + (value_scaled * right_span)

if __name__ == '__main__':
    try:
        inspection = AutonomousDemonstration()
        inspection.start()
    except rospy.ROSInterruptException:
        pass
