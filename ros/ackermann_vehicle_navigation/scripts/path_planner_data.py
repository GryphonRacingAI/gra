#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import Point  # This might still be used for other purposes
from ultralytics_ros.msg import ConeDetection, PathPlanningInput, Point2D  # Ensure Point2D is imported
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class PathPlannerData:
    def __init__(self):
        rospy.init_node('path_planner_data', anonymous=True)
        # Subscribers
        self.cone_sub = rospy.Subscriber('/cone_list', ConeDetection, self.cone_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Publishers
        self.path_input_pub = rospy.Publisher('/path_planning_input', PathPlanningInput, queue_size=10)
        # State
        self.car_position = Point2D()
        self.car_direction = Point2D(x=1.0, y=0.0)
        self.cones_by_type = [[] for _ in range(5)]  # List of lists for each cone type

    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        orientation_list = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        # Update car position and direction
        self.car_position.x = pos.x
        self.car_position.y = pos.y
        self.car_direction.x = np.cos(yaw)
        self.car_direction.y = np.sin(yaw)

    def cone_callback(self, msg):
        # Filter and categorize cones
        cone_types = [msg.yellow_cones, msg.blue_cones, msg.orange_cones, msg.large_orange_cones, msg.unknown_cones]
        cone_indices = [1, 2, 3, 4, 0]  # Mapping to fsd_path_planning indices
        for idx, cones in zip(cone_indices, cone_types):
            filtered_cones = []
            for cone in cones:
                distance = np.hypot(cone.x - self.car_position.x, cone.y - self.car_position.y)
                if distance <= 10.0:  # Filter cones more than 10 meters away
                    filtered_cones.append(Point2D(x=cone.x, y=cone.y))
            self.cones_by_type[idx] = filtered_cones

        self.publish_path_planning_input()

    def publish_path_planning_input(self):
        msg = PathPlanningInput()
        msg.header = Header(stamp=rospy.Time.now(), frame_id='path_planner')
        msg.car_position = self.car_position
        msg.car_direction = self.car_direction
        # Assigning cone positions for each type
        msg.yellow_cones = self.cones_by_type[1]
        msg.blue_cones = self.cones_by_type[2]
        msg.orange_small_cones = self.cones_by_type[3]
        msg.orange_big_cones = self.cones_by_type[4]
        msg.unknown_cones = self.cones_by_type[0]
        self.path_input_pub.publish(msg)

if __name__ == '__main__':
    try:
        planner_data = PathPlannerData()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
