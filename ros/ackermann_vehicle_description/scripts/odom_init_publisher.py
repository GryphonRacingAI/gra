#!/usr/bin/env python3
# This node takes the absolute robot state directly from Gazebo and uses it as odometry information for calibrating 2D laser odometry starting pose.

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

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

    odom_msg = Odometry()
    odom_msg.header.stamp = current_time
    odom_msg.header.frame_id = global_frame_id
    odom_msg.child_frame_id = "base_link"

    # Assign the pose from Gazebo directly to odometry
    odom_msg.pose.pose = msg.pose[vehicle_index]
    # Assign the twist from Gazebo directly to odometry
    odom_msg.twist.twist = msg.twist[vehicle_index]

    odom_publisher.publish(odom_msg)

if __name__ == '__main__':
    rospy.init_node('init_odom_publisher')

    vehicle_name = rospy.get_param('~vehicle_name', 'ackermann_vehicle')
    global_frame_id = rospy.get_param('~global_frame_id', 'odom')

    odom_publisher = rospy.Publisher('/init_odom', Odometry, queue_size=1)
    rospy.Subscriber('/gazebo/model_states', ModelStates, handle_vehicle_pose, vehicle_name)

    last_publish_time = None

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
