#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point
from ultralytics_ros.msg import ConeDetection
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16

class LapCounter:
    def __init__(self):
        rospy.init_node('Lap_counter', anonymous=False)

        # Subscribers
        self.cone_sub = rospy.Subscriber('/cone_list', ConeDetection, self.cone_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.Lap_counter_pub = rospy.Publisher('/Laps', UInt16, queue_size=10)

        # State
        self.car_position = Point()
        self.cone_destination = Point()
        self.previous_position = Point()
        self.lap_count = 0  # Lap count

    def cone_callback(self, msg):
        self.cone_destination = msg.large_orange_cones

    def odom_callback(self, msg):
        self.car_position = msg.pose.pose.position
        if self.car_position is not None and len(self.cone_destination) >= 2:
            # Assuming cone_positions are sorted or you can sort them
            first_cone = self.cone_destination[0]
            second_cone = self.cone_destination[1]

            if self.previous_position is not None:
                if second_cone.y < self.car_position.y < first_cone.y and self.car_position.x == 0:
                    if not (second_cone.y < self.previous_position.y < first_cone.y and self.previous_position.x == 0):
                        self.lap_count += 1
                        rospy.loginfo("Lap completed! Total laps: %d", self.lap_count)
                elif first_cone.y < self.car_position.y < second_cone.y and self.car_position.x == 0:
                    if not (second_cone.y < self.previous_position.y < first_cone.y and self.previous_position.x == 0):
                        self.lap_count += 1
                        rospy.loginfo("Lap completed! Total laps: %d", self.lap_count)
            self.previous_position = self.car_position
            self.Lap_counter_pub.publish(self.lap_count)

if __name__ == '__main__':
    try:
        Laps = LapCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass