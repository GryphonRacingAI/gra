#!/usr/bin/env python3

# This node transforms the cone coordinates, which are projected and therefore attached to the camera frame, to the global frame (i.e. the true coordinates of the cones).

import rospy
import tf
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from vision_msgs.msg import Detection3DArray
import math

class BoundingBoxTransformer:
    def __init__(self):
        rospy.init_node('bounding_box_transformer', anonymous=False)
        
        # The subscriber to the original YOLO 3D results
        self.sub = rospy.Subscriber("/yolo_3d_result", Detection3DArray, self.callback)
        
        # The publisher for the transformed bounding boxes
        self.pub = rospy.Publisher("/transformed_yolo_3d_result", Detection3DArray, queue_size=10)
        
        # The subscriber to the robot's odometry to get its current position
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # TF2 listener setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Current robot position
        self.robot_position = None

    def odom_callback(self, data):
        self.robot_position = data.pose.pose.position

    def transform_pose(self, pose, target_frame):
        """
        Transform the given pose to the target frame.
        """
        try:
            # Ensure we have the transform available
            self.tf_buffer.can_transform(target_frame, pose.header.frame_id, rospy.Time(0), rospy.Duration(4.0))
            
            # Transform the pose
            transformed_pose = self.tf_buffer.transform(pose, target_frame, rospy.Duration(4.0))
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Error transforming pose: %s" % str(e))
            return None

    def callback(self, data):
        if not data.detections:
            rospy.logwarn("Received an empty detections array.")
            return

        if self.robot_position is None:
            rospy.logwarn("Robot position is not available yet.")
            return

        transformed_data = Detection3DArray()
        transformed_data.header = data.header
        transformed_data.header.frame_id = "world"

        for detection in data.detections:
            if detection.header.frame_id == "":
                # rospy.logwarn("Detection header frame_id is empty. Setting it to 'camera_link_optical'.")
                detection.header.frame_id = "camera_link_optical"

            for result in detection.results:
                # Create a PoseStamped from the detection pose
                pose_stamped = PoseStamped()
                pose_stamped.header = detection.header
                pose_stamped.pose = detection.bbox.center
                
                # Check if we can transform the pose to the target frame
                if not self.tf_buffer.can_transform("world", pose_stamped.header.frame_id, rospy.Time(0)):
                    rospy.logwarn("Cannot transform from %s to world" % pose_stamped.header.frame_id)
                    continue

                # Transform pose to the target frame
                transformed_pose_stamped = self.transform_pose(pose_stamped, "world")
                
                if transformed_pose_stamped:
                    # Calculate distance from the robot to the cone
                    dx = transformed_pose_stamped.pose.position.x - self.robot_position.x
                    dy = transformed_pose_stamped.pose.position.y - self.robot_position.y
                    distance = math.sqrt(dx**2 + dy**2)
                    
                    # Only add the detection if it is within 10 meters
                    if distance < 10.0:
                        detection.bbox.center = transformed_pose_stamped.pose
                        transformed_data.detections.append(detection)

        if transformed_data.detections:
            self.pub.publish(transformed_data)
        else:
            rospy.logwarn("No detections were transformed.")


if __name__ == '__main__':
    bb_transformer = BoundingBoxTransformer()
    rospy.spin()
