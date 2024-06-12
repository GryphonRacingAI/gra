#!/usr/bin/env python3

import time
import rospy
import math
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

class StaticInspectionA:
    def __init__(self):
        rospy.init_node('static_inspection_A', anonymous=False)
        self.ackermann_publisher = rospy.Publisher('/ackermann_cmd_controller', AckermannDrive, queue_size=1)
        self.chequered_flag_publisher = rospy.Publisher('/chequered_flag', Bool, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def start(self):
        self.sweep_steering()
        self.ramp_up_drivetrain()

    def sweep_steering(self):
        rospy.loginfo("Starting steering sweep")
        time.sleep(3)
        angles = [-0.7, 0.7, 0.0]
        for angle in angles:
            ackermann_message = AckermannDrive()
            ackermann_message.steering_angle = angle
            self.ackermann_publisher.publish(ackermann_message)
            rospy.loginfo(f"Set steering angle to {angle}")
            time.sleep(3)  # Wait for a few seconds to settle

    def ramp_up_drivetrain(self):
        rospy.loginfo("Starting drivetrain ramp up")
        start_stopwatch = time.time()
        WHEEL_RADIUS = 0.2575
        AXLE_SPEED = 200  # Desired axle speed in rpm
        car_speed_mps = AXLE_SPEED * (2 * math.pi) / 60 * WHEEL_RADIUS

        while not rospy.is_shutdown():
            ackermann_message = AckermannDrive()
            elapsed = time.time() - start_stopwatch
            ackermann_message.speed = self.linear_interpolate(elapsed, 0, 10, 0, car_speed_mps)
            rospy.loginfo(f"Speed: {ackermann_message.speed}")
            self.ackermann_publisher.publish(ackermann_message)
            self.rate.sleep()
            if elapsed > 10:
                break

        time.sleep(5)  # Wait for 5 seconds before stopping
        self.stop_drivetrain()
        self.signal_completion()

    def stop_drivetrain(self):
        ackermann_message = AckermannDrive()
        ackermann_message.speed = 0
        self.ackermann_publisher.publish(ackermann_message)
        rospy.loginfo("Drivetrain stopped")

    def signal_completion(self):
        self.chequered_flag_publisher.publish(Bool(data=True))
        rospy.loginfo("Mission completed, chequered flag set")

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
        inspection = StaticInspectionA()
        inspection.start()
    except rospy.ROSInterruptException:
        pass
