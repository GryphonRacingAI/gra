#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
import tf2_ros

def odom_publisher():
    rospy.init_node('dummy_odom', anonymous=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(10)  # 10Hz

    # Initial pose
    x = 0.0
    y = 0.0
    z = 0.0

    while not rospy.is_shutdown():
        # Update pose
        x += 0.1  # Increment x by 0.1 meter per iteration

        # Publish the transform from /world to /base_link
        world_to_base_link = TransformStamped()
        world_to_base_link.header.stamp = rospy.Time.now()
        world_to_base_link.header.frame_id = "world"
        world_to_base_link.child_frame_id = "base_link"
        world_to_base_link.transform.translation = Vector3(x, y, z)
        world_to_base_link.transform.rotation = Quaternion(0, 0, 0, 1)
        tf_broadcaster.sendTransform(world_to_base_link)

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "world"
        odom_msg.child_frame_id = "base_link"

        # Set pose and twist
        odom_msg.pose.pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
        odom_msg.twist.twist = Twist(Vector3(1.0, 0, 0), Vector3(0, 0, 0))

        odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher()
    except rospy.ROSInterruptException:
        pass
