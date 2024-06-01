#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header, UInt16
from tf.transformations import euler_from_quaternion
from ultralytics_ros.msg import ExtendedConeDetection
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes

print("fsd loaded")

class TrackPathfinder:
    def __init__(self):
        rospy.init_node('track_pathfinder', anonymous=True)

        self.path_planner = PathPlanner(MissionTypes.trackdrive)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.Subscriber('/filtered_cone_list', ExtendedConeDetection, self.path_callback)
        rospy.Subscriber('/chequered_flag', UInt16, self.chequered_flag_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        self.final_lap = False
        self.final_path_published = False
        print("node initialised")

    def chequered_flag_callback(self, msg):
        if msg.data == 1 and not self.final_path_published:
            self.final_lap = True
            rospy.loginfo("Final lap detected! Preparing to publish the final path.")

    def path_callback(self, msg):
        if self.final_path_published:
            return  # Stop further processing after publishing the final path

        if self.final_lap:
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

            # Publish the final path
            self.publish_final_path(path, msg.header)
            self.final_path_published = True
            rospy.loginfo("Final path published. Stopping further path planning.")
            rospy.signal_shutdown("Final path published.")
            return  # Exit the callback to stop further processing

        # Normal path planning
        self.process_and_publish_path(msg)

    def process_and_publish_path(self, msg):
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

        # Publish the path, skipping the first four waypoints to create some lead space
        self.publish_path(path, msg.header)

    def publish_path(self, path, header):
        ros_path = Path()
        ros_path.header = Header(stamp=rospy.Time.now(), frame_id='world')

        # Start from the 5th waypoint, skipping the first four
        for point in path[4:]:
            pose = PoseStamped()
            pose.header = ros_path.header 
            pose.pose.position.x = point[1]  # path_x
            pose.pose.position.y = point[2]  # path_y
            pose.pose.position.z = 0 
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

    def publish_final_path(self, path, header):
        ros_path = Path()
        ros_path.header = Header(stamp=rospy.Time.now(), frame_id='world')

        # Publish only the path from index 2 to 4
        for point in path[2:4]:
            pose = PoseStamped()
            pose.header = ros_path.header  
            pose.pose.position.x = point[1]  # path_x
            pose.pose.position.y = point[2]  # path_y
            pose.pose.position.z = 0  
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

if __name__ == '__main__':
    try:
        pathfinder = TrackPathfinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
