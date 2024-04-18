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

        # Initialize the path planner
        self.path_planner = PathPlanner(MissionTypes.trackdrive)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.Subscriber('/filtered_cone_list', ExtendedConeDetection, self.path_callback)

        # Control flag for initial warm-up
        self.initial_run = True
        print("Node initialised")

    def path_callback(self, msg):
        # Convert orientation quaternion to Euler angles to get the yaw
        orientation = msg.car_orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Prepare data for path planning
        car_position = np.array([msg.car_position.x, msg.car_position.y])
        car_direction = np.array([np.cos(yaw), np.sin(yaw)])

        # Organise cones by type
        global_cones = [
            np.array([[cone.x, cone.y] for cone in msg.unknown_cones]),  # Index 0
            np.array([[cone.x, cone.y] for cone in msg.yellow_cones]),  # Index 1
            np.array([[cone.x, cone.y] for cone in msg.blue_cones]),  # Index 2
            np.array([[cone.x, cone.y] for cone in msg.orange_cones]),  # Index 3
            np.array([[cone.x, cone.y] for cone in msg.large_orange_cones])  # Index 4
        ]

        # Calculate the path
        path = self.path_planner.calculate_path_in_global_frame(global_cones, car_position, car_direction)

        # Check if this is the initial run
        if self.initial_run:
            # Do not publish, just process to "warm up"
            print("Initial path calculation completed, delaying 5 seconds before becoming fully functional.")
            rospy.sleep(30)  # Delay 5 seconds to allow system to stabilse
            self.initial_run = False
        else:
            # Publish the path, skipping the first three waypoints to create some lead space
            self.publish_path(path, msg.header)

    def publish_path(self, path, header):
        ros_path = Path()
        ros_path.header = Header(stamp=rospy.Time.now(), frame_id='world')

        # Start from the 4th waypoint, skipping the first three
        for point in path[3:]:
            pose = PoseStamped()
            pose.header = ros_path.header  # Use updated header for consistency across poses
            pose.pose.position.x = point[1]  # path_x
            pose.pose.position.y = point[2]  # path_y
            pose.pose.position.z = 0  # Assuming the path is essentially 2D
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

if __name__ == '__main__':
    try:
        pathfinder = TrackPathfinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
