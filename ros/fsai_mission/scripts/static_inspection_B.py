#!/usr/bin/env python3

import time
import rospy
import math
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Bool

class StaticInspectionB:
    def __init__(self):
        rospy.init_node('static_inspection_B', anonymous=False)
        self.ackermann_publisher = rospy.Publisher('/ackermann_cmd_controller', AckermannDrive, queue_size=1)
        self.emergency_brake_publisher = rospy.Publisher('/emergency_brake', Bool, queue_size=1)
        self.rate = rospy.Rate(10) # 10hz
        
    def start(self):
        self.step1()
        self.step2()

    def step1(self):
        rospy.loginfo("step 1")
        start_stopwatch = time.time()
        WHEEL_RADIUS = 0.2575
        AXLE_SPEED = 100
        car_speed_mps = AXLE_SPEED * (2*math.pi) / 60 * WHEEL_RADIUS

        while not rospy.is_shutdown():
            ackermann_message = AckermannDrive()
            elapsed = time.time() - start_stopwatch
            ackermann_message.speed = self.linear_interpolate(elapsed, 0, 5, 0, car_speed_mps)
            rospy.loginfo(f"speed: {ackermann_message.speed}")
            self.ackermann_publisher.publish(ackermann_message)
            self.rate.sleep()
            if elapsed > 6:
                return
            
    def step2(self):
        self.emergency_brake_publisher.publish(True)


    @staticmethod
    def linear_interpolate(value, a1, a2, b1, b2):
        # Figure out how 'wide' each range is
        leftSpan = a2 - a1
        rightSpan = b2 - b1

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - a1) / float(leftSpan)

        # Convert the 0-1 range into a value in the right range.
        return b1 + (valueScaled * rightSpan)

if __name__ == '__main__':
    try:
        inspection = StaticInspectionB()
        inspection.start()
    except rospy.ROSInterruptException:
        pass
