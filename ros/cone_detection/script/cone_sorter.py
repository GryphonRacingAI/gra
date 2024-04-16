#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection3DArray
from ultralytics_ros.msg import ConeDetection  

# A dictionary to map id values to human-readable names
CONE_TYPES = {
    0: "blue_cones",
    1: "large_orange_cones",
    2: "orange_cones",
    3: "unknown_cones",
    4: "yellow_cones"
}

class ConeSorter:
    def __init__(self):
        rospy.init_node('cone_sorter', anonymous=False)
        
        self.cone_lists = {name: [] for name in CONE_TYPES.values()}  
        
        self.sub = rospy.Subscriber("/transformed_yolo_3d_result", Detection3DArray, self.callback)
        self.pub = rospy.Publisher("/cone_list", ConeDetection, queue_size=10)

    def is_new_cone(self, new_cone, existing_cones):
        for cone in existing_cones:
            if np.linalg.norm(np.array([cone.x, cone.y]) - np.array([new_cone.x, new_cone.y])) < 1.0:   # Avoid redundant cones. Value is cone proximity threshold in meters
                return False
        return True

    def callback(self, data):
        for detection in data.detections:
            # Skip detections with invalid size criteria
            size = detection.bbox.size
            if np.isinf(size.x) or np.isinf(size.y) or np.isinf(size.z):
                rospy.loginfo("Skipping detection with infinite size.")
                continue
            if size.x > 0.5 or size.y > 0.5 or size.z > 0.5:
                rospy.loginfo("Skipping detection with size larger than 0.5.")
                continue

            cone_type = CONE_TYPES[detection.results[0].id]  # Map the id to the human-readable name
            new_cone = Point(detection.bbox.center.position.x, detection.bbox.center.position.y, 0)  # Z value is not used

            if self.is_new_cone(new_cone, self.cone_lists[cone_type]):
                self.cone_lists[cone_type].append(new_cone)

        self.publish_cones()


    def publish_cones(self):
        cone_list_msg = ConeDetection()
        cone_list_msg.header.stamp = rospy.Time.now()

        for cone_type, points in self.cone_lists.items():
            setattr(cone_list_msg, cone_type, points)
        
        self.pub.publish(cone_list_msg)

if __name__ == '__main__':
    cone_sorter = ConeSorter()
    rospy.spin()
