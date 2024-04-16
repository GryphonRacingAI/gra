#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Point, Quaternion
from std_msgs.msg import Header
from ultralytics_ros.msg import ConeDetection, ExtendedConeDetection
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class PathPlannerData:
    def __init__(self):
        rospy.init_node('path_planner_data', anonymous=False)

        # Subscribers
        self.cone_sub = rospy.Subscriber('/cone_list', ConeDetection, self.cone_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.filtered_cones_pub = rospy.Publisher('/filtered_cone_list', ExtendedConeDetection, queue_size=10)

        # State
        self.car_position = Point()
        self.car_orientation = Quaternion()

        print("path_planner_data node alive")

    def odom_callback(self, msg):
        # print("Received odom data")
        # Update car position
        self.car_position = msg.pose.pose.position
        self.car_orientation = msg.pose.pose.orientation

    def cone_callback(self, msg):
        # print("Received cone data")
        filtered_cones = ExtendedConeDetection()
        filtered_cones.header = Header(stamp=rospy.Time.now(), frame_id='path_planner')
        filtered_cones.car_position = self.car_position
        filtered_cones.car_orientation = self.car_orientation

        # Filter cones based on distance
        cone_lists = [msg.blue_cones, msg.yellow_cones, msg.orange_cones, msg.large_orange_cones, msg.unknown_cones]
        filtered_lists = [[] for _ in cone_lists]

        car_pos_array = np.array([self.car_position.x, self.car_position.y, self.car_position.z])
        for i, cones in enumerate(cone_lists):
            for cone in cones:
                cone_array = np.array([cone.x, cone.y, cone.z])
                if np.linalg.norm(cone_array - car_pos_array) <= 10.0:
                    filtered_lists[i].append(cone)

        # Assign filtered lists back to the message
        filtered_cones.blue_cones, filtered_cones.yellow_cones, filtered_cones.orange_cones, \
        filtered_cones.large_orange_cones, filtered_cones.unknown_cones = filtered_lists

        self.filtered_cones_pub.publish(filtered_cones)
        # print("Published filtered data")

if __name__ == '__main__':
    try:
        planner_data = PathPlannerData()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
