#!/usr/bin/env python3

# This node counts the laps by drawing a line connecting large orange cones and detecting cross product sign change

import rospy
from geometry_msgs.msg import Point
from ultralytics_ros.msg import ConeDetection
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
import math

class LapCounter:
    def __init__(self):
        rospy.init_node('lap_counter', anonymous=False)

        # Subscribers
        self.cone_sub = rospy.Subscriber('/cone_list', ConeDetection, self.cone_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.lap_counter_pub = rospy.Publisher('/laps', UInt16, queue_size=10)

        # State
        self.car_position = Point()
        self.cone_positions = []
        self.previous_position = None
        self.lap_count = 0  # Lap count
        self.in_cooldown = False

        # Timer for periodic publishing
        self.publish_timer = rospy.Timer(rospy.Duration(1), self.publish_lap_count)

        rospy.loginfo("Lap Counter node initialised")

    def cone_callback(self, msg):
        self.cone_positions = msg.large_orange_cones

    def odom_callback(self, msg):
        self.car_position = msg.pose.pose.position

        if self.previous_position is not None and len(self.cone_positions) >= 2 and not self.in_cooldown:
            first_cone = self.cone_positions[0]
            second_cone = self.cone_positions[1]

            # Calculate vectors
            finish_line_vector = (second_cone.x - first_cone.x, second_cone.y - first_cone.y)
            prev_car_vector = (self.previous_position.x - first_cone.x, self.previous_position.y - first_cone.y)
            curr_car_vector = (self.car_position.x - first_cone.x, self.car_position.y - first_cone.y)

            # Calculate cross products to determine if the car has crossed the finish line
            prev_cross = finish_line_vector[0] * prev_car_vector[1] - finish_line_vector[1] * prev_car_vector[0]
            curr_cross = finish_line_vector[0] * curr_car_vector[1] - finish_line_vector[1] * curr_car_vector[0]

            if prev_cross * curr_cross < 0:  # Signs are different, indicating a crossing
                if self.is_near_finish_line(first_cone, second_cone, self.car_position):
                    self.lap_count += 1
                    rospy.loginfo("Lap completed! Total laps: %d", self.lap_count)
                    self.initiate_cooldown()

        self.previous_position = self.car_position

    def is_near_finish_line(self, first_cone, second_cone, car_position, threshold=4.0):    # 4.0 meter away from the line segment will be ignored
        """Check if the car is near the line segment formed by the first and second cone."""
        def point_line_distance(px, py, x1, y1, x2, y2):
            # Calculate the distance from point (px, py) to the line segment (x1, y1) - (x2, y2)
            line_mag = math.dist((x2, x1), (y2, y1))
            if line_mag < 1e-6:
                return float('inf')  # Line segment too small, effectively a point
            u1 = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / (line_mag**2)
            u = max(min(u1, 1), 0)
            ix = x1 + u * (x2 - x1)
            iy = y1 + u * (y2 - y1)
            return math.dist((px, py), (ix, iy))

        distance = point_line_distance(car_position.x, car_position.y, first_cone.x, first_cone.y, second_cone.x, second_cone.y)
        rospy.loginfo(f"Distance from car to finish line: {distance}")
        return distance <= threshold

    def initiate_cooldown(self):
        self.in_cooldown = True
        rospy.loginfo("Cooldown initiated.")
        rospy.Timer(rospy.Duration(10), self.end_cooldown, oneshot=True)  # 10 second cooldown period

    def end_cooldown(self, event):
        self.in_cooldown = False
        rospy.loginfo("Cooldown ended. Ready to detect laps again.")

    def publish_lap_count(self, event):
        self.lap_counter_pub.publish(self.lap_count)

if __name__ == '__main__':
    try:
        Laps = LapCounter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
