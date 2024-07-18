#!/usr/bin/env python3

import time
import rospy
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive

class AutonomousDemonstration:
    WHEEL_CIRCUMFERENCE = 1.64

    def __init__(self):
        rospy.init_node('autonomous_demo', anonymous=False)
        self.ackermann_publisher = rospy.Publisher('/ackermann_cmd_controller', AckermannDrive, queue_size=1)
        self.brake_publisher = rospy.Publisher('/brake', Bool, queue_size=1)
        self.emergency_brake_publisher = rospy.Publisher('/emergency_brake', Bool, queue_size=1)
        self.chequered_flag_publisher = rospy.Publisher('/chequered_flag', Bool, queue_size=1)
        self.pause_publisher = rospy.Publisher('/pause', Bool, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def start(self):
        time.sleep(1)
        self.pause_publisher.publish(Bool(data=True))
        self.sweep_steering()
        time.sleep(1)
        self.pause_publisher.publish(Bool(data=False))
        time.sleep(4)
        self.pause_publisher.publish(Bool(data=True))
        self.brake_10m()
        time.sleep(4)
        self.pause_publisher.publish(Bool(data=False))
        time.sleep(4)
        self.emergency_brake()
        rospy.loginfo("Autonomous demo routine completed.")

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
        rospy.loginfo("Steering demo done")

    def brake_10m(self):
        self.brake_publisher.publish(Bool(data=True))
        rospy.loginfo("Braking for 4 seconds")
        time.sleep(4)
        self.brake_publisher.publish(Bool(data=False))
        rospy.loginfo("Braking complete")

    def emergency_brake(self):
        self.emergency_brake_publisher.publish(Bool(data=True))
        rospy.loginfo("Emergency brake applied")

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
        demo = AutonomousDemonstration()
        demo.start()
    except rospy.ROSInterruptException:
        pass
