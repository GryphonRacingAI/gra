#!/usr/bin/env python3

import rospy
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
        self.lap_counter_pub = rospy.Publisher('/Laps', UInt16, queue_size=10)

        # State
        self.car_position = Point()
        self.cone_positions = []
        self.previous_position = None
        self.lap_count = 0  # Lap count

    def cone_callback(self, msg):
        self.cone_positions = msg.large_orange_cones
        # rospy.loginfo("Large orange cone positions received:")
        for cone in self.cone_positions:
            # rospy.loginfo(f"  Cone position: x={cone.x}, y={cone.y}, z={cone.z}")
            pass

    def odom_callback(self, msg):
        self.car_position = msg.pose.pose.position
        # rospy.loginfo(f"Current car position: x={self.car_position.x}, y={self.car_position.y}, z={self.car_position.z}")

        if self.previous_position is not None and len(self.cone_positions) >= 2:
            first_cone = self.cone_positions[0]
            second_cone = self.cone_positions[1]
            # rospy.loginfo(f"First cone position: x={first_cone.x}, y={first_cone.y}, z={first_cone.z}")
            # rospy.loginfo(f"Second cone position: x={second_cone.x}, y={second_cone.y}, z={second_cone.z}")

            # Calculate vectors
            finish_line_vector = (second_cone.x - first_cone.x, second_cone.y - first_cone.y)
            prev_car_vector = (self.previous_position.x - first_cone.x, self.previous_position.y - first_cone.y)
            curr_car_vector = (self.car_position.x - first_cone.x, self.car_position.y - first_cone.y)

            # Calculate cross products to determine if the car has crossed the finish line
            prev_cross = finish_line_vector[0] * prev_car_vector[1] - finish_line_vector[1] * prev_car_vector[0]
            curr_cross = finish_line_vector[0] * curr_car_vector[1] - finish_line_vector[1] * curr_car_vector[0]

            rospy.loginfo(f"Previous cross product: {prev_cross}")
            rospy.loginfo(f"Current cross product: {curr_cross}")

            if prev_cross * curr_cross < 0:  # Signs are different, indicating a crossing
                self.lap_count += 1
                rospy.loginfo("Lap completed! Total laps: %d", self.lap_count)
                self.lap_counter_pub.publish(self.lap_count)

        self.previous_position = self.car_position

if __name__ == '__main__':
    try:
        Laps = LapCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
