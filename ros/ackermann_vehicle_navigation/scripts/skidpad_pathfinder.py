#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header
from tf.transformations import euler_from_quaternion
from ultralytics_ros.msg import ExtendedConeDetection
from fsd_path_planning import PathPlanner, MissionTypes, ConeTypes

print("fsd loaded")

class SkidpadPathfinder:
    def __init__(self):
        rospy.init_node('skidpad_pathfinder', anonymous=True)

        # Initialize the PathPlanner with Skidpad mission type
        self.path_planner = PathPlanner(MissionTypes.skidpad)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.Subscriber('/filtered_cone_list', ExtendedConeDetection, self.path_callback)

        self.rate = rospy.Rate(10)  # 10 Hz
        print("Skidpad node initialised")

    def path_callback(self, msg):
        # Convert orientation quaternion to Euler angles to get the yaw
        orientation = msg.car_orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # Prepare data for path planning
        car_position = np.array([msg.car_position.x, msg.car_position.y])
        car_direction = np.array([np.cos(yaw), np.sin(yaw)])

        # Organize all cones into a single array
        all_cones = []
        for cone_list in [msg.unknown_cones, msg.yellow_cones, msg.blue_cones, msg.orange_cones, msg.large_orange_cones]:
            if cone_list:
                cones_array = np.array([[cone.x, cone.y] for cone in cone_list])
                all_cones.append(cones_array)

        if all_cones:  # Check if there are any cones detected
            global_cones = np.vstack(all_cones)  # Combine all cones into a single array
        else:
            global_cones = np.empty((0, 2))  # No cones detected, use an empty array

        # Calculate the path using skidpad specific logic
        try:
            path = self.path_planner.calculate_path_in_global_frame([global_cones], car_position, car_direction)
            # Publish the path, potentially adjusting for skidpad specifics
            self.publish_path(path, msg.header)
        except Exception as e:
            rospy.logerr(f"Error calculating path: {e}")


    def publish_path(self, path, header):
        ros_path = Path()
        ros_path.header = Header(stamp=rospy.Time.now(), frame_id='world')

        # Publish the path as before but consider any skidpad-specific adjustments
        for point in path:
            pose = PoseStamped()
            pose.header = ros_path.header
            pose.pose.position.x = point[1]  # path_x
            pose.pose.position.y = point[2]  # path_y
            pose.pose.position.z = 0  # Assuming the path is essentially 2D
            ros_path.poses.append(pose)

        self.path_pub.publish(ros_path)

if __name__ == '__main__':
    try:
        pathfinder = SkidpadPathfinder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
