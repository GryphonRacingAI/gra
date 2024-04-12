#!/usr/bin/env python3

# This node listens to /cone_list and creater marker array for visualisation on rviz

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from ultralytics_ros.msg import ConeDetection

def create_marker(x, y, color, diameter, height, marker_id):
    marker = Marker()
    marker.header.frame_id = "world"
    marker.id = marker_id
    marker.type = marker.CYLINDER
    marker.action = marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = height / 2.0  # Center the cylinder vertically
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = height
    marker.color.a = 0.35  # Alpha
    marker.color.r = color['r']
    marker.color.g = color['g']
    marker.color.b = color['b']
    return marker

def callback(data):
    marker_array = MarkerArray()
    marker_id = 0  # Initialize marker ID
    colors = {
        'blue_cones': {'r': 0, 'g': 0, 'b': 1},
        'yellow_cones': {'r': 1, 'g': 1, 'b': 0},
        'orange_cones': {'r': 1, 'g': 0.5, 'b': 0},
        'large_orange_cones': {'r': 1, 'g': 0.5, 'b': 0},
        'unknown_cones': {'r': 1, 'g': 1, 'b': 1}
    }
    sizes = {
        'blue_cones': (0.23, 0.325),
        'yellow_cones': (0.23, 0.325),
        'orange_cones': (0.23, 0.325),
        'large_orange_cones': (0.285, 0.505),
        'unknown_cones': (0.23, 0.325)
    }

    for cone_type in sizes:
        for point in getattr(data, cone_type):
            marker = create_marker(point.x, point.y, colors[cone_type], sizes[cone_type][0], sizes[cone_type][1], marker_id)
            marker_array.markers.append(marker)
            marker_id += 1  # Increment the marker ID for each new marker

    pub.publish(marker_array)  # Publish the MarkerArray

if __name__ == '__main__':
    rospy.init_node('cone_marker')
    pub = rospy.Publisher('/cone_marker_array', MarkerArray, queue_size=100)
    rospy.Subscriber('/cone_list', ConeDetection, callback)
    rospy.spin()
