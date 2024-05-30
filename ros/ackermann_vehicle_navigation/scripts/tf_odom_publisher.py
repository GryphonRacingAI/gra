#!/usr/bin/env python3
# This node takes the ABSOLUTE robot state directly from gazebo and take that as /odom

import rospy
import tf2_ros
import geometry_msgs.msg
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import random  # Import for generating Gaussian noise

def handle_vehicle_pose(msg, vehicle_name):
    current_time = rospy.Time.now()

    # Check if enough time has passed since the last publish
    global last_publish_time
    if last_publish_time is not None and (current_time - last_publish_time).to_sec() < 0.05:  # 20 Hz maximum rate
        return
    last_publish_time = current_time

    try:
        vehicle_index = msg.name.index(vehicle_name)
    except ValueError:
        rospy.logwarn(f"{vehicle_name} not found in Gazebo model states.")
        return

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = current_time
    t.header.frame_id = global_frame_id
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.pose[vehicle_index].position.x + random.gauss(0, position_sigma)
    t.transform.translation.y = msg.pose[vehicle_index].position.y + random.gauss(0, position_sigma)
    t.transform.translation.z = 0.0  # Assuming flat terrain
    t.transform.rotation = add_noise_to_orientation(msg.pose[vehicle_index].orientation)

    br.sendTransform(t)

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = global_frame_id
    odom_msg.child_frame_id = "base_link"
    odom_msg.pose.pose.position.x = t.transform.translation.x
    odom_msg.pose.pose.position.y = t.transform.translation.y
    odom_msg.pose.pose.position.z = 0
    odom_msg.pose.pose.orientation = t.transform.rotation
    odom_msg.twist.twist = msg.twist[vehicle_index]
    odom_publisher.publish(odom_msg)

def add_noise_to_orientation(orientation):
    # Adding Gaussian noise to quaternion orientation
    return geometry_msgs.msg.Quaternion(
        x=orientation.x + random.gauss(0, orientation_sigma),
        y=orientation.y + random.gauss(0, orientation_sigma),
        z=orientation.z + random.gauss(0, orientation_sigma),
        w=orientation.w + random.gauss(0, orientation_sigma)
    )

if __name__ == '__main__':
    rospy.init_node('tf_odom_publisher')
    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    global_frame_id = rospy.get_param('~global_frame_id', 'odom')
    odom_publisher = rospy.Publisher('/odom', Odometry, queue_size=1)
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_vehicle_pose, vehicle_name)
    last_publish_time = None
    position_sigma = 0.00  # Standard deviation for position noise
    orientation_sigma = 0.0  # Standard deviation for orientation noise

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
