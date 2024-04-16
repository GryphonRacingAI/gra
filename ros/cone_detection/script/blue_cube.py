#!/usr/bin/env python3

# This script serves as a tester to Rviz axis orientation
# Works by publishing a 1x1x1 blue cube dummy marker message to /visulization_marker

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def publish_marker():
    rospy.init_node('cube_marker_publisher', anonymous=True)
    
    # Publisher to publish Marker messages on the 'visualization_marker' topic
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    # Give the node time to connect to ROS master
    rospy.sleep(1)
    
    while not rospy.is_shutdown():
        # Define the marker
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        
        # Set the namespace and id for this marker
        marker.ns = "cube"
        marker.id = 0
        
        # Set the marker type to CUBE
        marker.type = Marker.CUBE
        
        # Set the pose of the marker to the origin
        marker.pose.position.x = 1
        marker.pose.position.y = 1
        marker.pose.position.z = 1
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        
        # Set the scale of the marker (1x1x1 meter)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        
        # Set the color of the marker (blue with full opacity)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        # Publish the marker
        pub.publish(marker)
        
        # Wait for a while before publishing again
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
