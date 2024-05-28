#!/usr/bin/env python3
# This node simulates encoder reading by reading rear wheel joints in gazebo and compute wheel speeds, and listen to command steering angle

import rospy
from sensor_msgs.msg import JointState
from ackermann_vehicle_description.msg import AckermannFeedback
from ackermann_msgs.msg import AckermannDrive

class GazeboTFPublisher:
    def __init__(self):
        rospy.init_node('gazebo_tf_publisher')

        # Publishers
        self.publisher = rospy.Publisher('ackermann_feedback', AckermannFeedback, queue_size=10)

        # Subscribers
        self.joint_subscriber = rospy.Subscriber('joint_states', JointState, self.joint_callback)
        self.drive_subscriber = rospy.Subscriber('ackermann_cmd', AckermannDrive, self.drive_callback)

        # Initialize variables
        self.current_drive = AckermannDrive()

    def joint_callback(self, msg):
        # Find the indices for the rear axles
        try:
            left_index = msg.name.index('left_rear_axle')
            right_index = msg.name.index('right_rear_axle')
        except ValueError:
            rospy.logerr("Rear axle joints not found in the joint states.")
            return

        # Get the velocities of the rear wheels
        left_wheel_speed = msg.velocity[left_index]
        right_wheel_speed = msg.velocity[right_index]

        # Publish the feedback message
        feedback_msg = AckermannFeedback()
        feedback_msg.left_wheel_speed = left_wheel_speed
        feedback_msg.right_wheel_speed = right_wheel_speed
        feedback_msg.steering_angle = self.current_drive.steering_angle
        self.publisher.publish(feedback_msg)

    def drive_callback(self, msg):
        # Update the current Ackermann drive command
        self.current_drive = msg

def main():
    try:
        tf_publisher = GazeboTFPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
