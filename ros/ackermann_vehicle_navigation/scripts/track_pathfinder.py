#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from ultralytics_ros.msg import ExtendedConeDetection
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes

class TrackPathfinder:
    def __init__(self):
        rospy.init_node('track_pathfinder', anonymous=True)

        self.path_planner = PathPlanner(MissionTypes.trackdrive)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.Subscriber('/filtered_cone_list', ExtendedConeDetection, self.path_callback)
        self.first_path_time = None  # Timestamp for the first calculated path

        print("fsd loaded")
        print("node initialised")

    def path_callback(self, msg):
        orientation = msg.car_orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        car_position = np.array([msg.car_position.x, msg.car_position.y])
        car_direction = np.array([np.cos(yaw), np.sin(yaw)])

        global_cones = [
            np.array([[cone.x, cone.y] for cone in msg.unknown_cones]),
            np.array([[cone.x, cone.y] for cone in msg.yellow_cones]),
            np.array([[cone.x, cone.y] for cone in msg.blue_cones]),
            np.array([[cone.x, cone.y] for cone in msg.orange_cones]),
            np.array([[cone.x, cone.y] for cone in msg.large_orange_cones])
        ]

        path = self.path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction)
        self.publish_path(path, msg.header)

    def publish_path(self, path, header):
        if self.first_path_time is None:
            self.first_path_time = rospy.Time.now()

        current_time = rospy.Time.now()
        if (current_time - self.first_path_time).to_sec() > 10:  # Wait for 10 seconds before publishing paths
            ros_path = Path()
            ros_path.header = Header(stamp=rospy.Time.now(), frame_id='world')

            for point in path[3:]:  # Skip the first three waypoints
                pose = PoseStamped()
                pose.header = ros_path.header
                pose.pose.position.x = point[1]
                pose.pose.position.y = point[2]
                pose.pose.position.z = 0
                ros_path.poses.append(pose)

            self.path_pub.publish(ros_path)

if __name__ == '__main__':
    try:
        pathfinder = TrackPathfinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
