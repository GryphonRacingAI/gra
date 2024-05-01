#!/usr/bin/env python3
# This node computes odometry based on wheel speeds of both rear wheels and the steering angle and publish to /odom 

import math
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header
from ackermann_vehicle_description.msg import AckermannFeedback 

class AckermannState:
    def __init__(self, position, orientation, left_wheel_speed, right_wheel_speed, steering_angle, time):
        self.position = position
        self.orientation = orientation
        self.left_wheel_speed = left_wheel_speed
        self.right_wheel_speed = right_wheel_speed
        self.steering_angle = steering_angle
        self.time = time

class OdomPublisher:
    def __init__(self):
        rospy.init_node('odom_publisher')

        # Parameters
        self.axle_length = rospy.get_param('~axle_length')
        self.wheelbase_length = rospy.get_param('~wheelbase_length')
        self.wheel_radius = rospy.get_param('~wheel_radius')
        self.center_of_mass_offset = rospy.get_param('~center_of_mass_offset', 0.0)
        self.damping_factor = rospy.get_param('~damping_factor', 1)

        # Publishers
        self.publisher = rospy.Publisher('odom', Odometry, queue_size=10)

        # Subscribers
        self.subscriber = rospy.Subscriber('ackermann_feedback', AckermannFeedback, self.feedback_callback)

        self.state = AckermannState(
            position=np.array([0, 0, 0]),
            orientation=R.from_euler('xyz', [0, 0, 0]),
            left_wheel_speed=0.0, 
            right_wheel_speed=0.0,
            steering_angle=0.0,
            time=rospy.Time.now()
        )



    def feedback_callback(self, msg):
        self.state = self.state_update(self.state, msg)
        output = self.output(self.state)
        self.publisher.publish(output)

    def state_update(self, state, feedback):
        average_wheel_speed = (feedback.left_wheel_speed + feedback.right_wheel_speed) / 2
        linear_speed = average_wheel_speed * self.wheel_radius
        turn_radius = self.turn_radius(feedback.steering_angle)
        angular_speed = linear_speed / turn_radius

        time_delta = (rospy.Time.now() - state.time).to_sec()
        heading_delta = angular_speed * time_delta

        orientation_delta = R.from_euler('xyz', [0, 0, heading_delta])
        position_delta = state.orientation.apply([linear_speed * time_delta, 0, 0])

        return AckermannState(
            position=state.position + self.damping_factor * position_delta,
            orientation=orientation_delta * state.orientation,
            left_wheel_speed=feedback.left_wheel_speed,
            right_wheel_speed=feedback.right_wheel_speed,
            steering_angle=feedback.steering_angle,
            time=rospy.Time.now()
        )

    def output(self, state):
        quaternion = state.orientation.as_quat()
        linear_velocity = state.orientation.apply([self.wheel_radius * (state.left_wheel_speed + state.right_wheel_speed) / 2, 0, 0])
        angular_speed = linear_velocity[0] / self.turn_radius(state.steering_angle)

        return Odometry(
            header=Header(
                stamp=state.time,
                frame_id='odom'
            ),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(x=state.position[0], y=state.position[1], z=state.position[2]),
                    orientation=Quaternion(x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3])
                )
            ),
            twist=TwistWithCovariance(
                twist=Twist(
                    linear=Vector3(x=linear_velocity[0], y=linear_velocity[1], z=linear_velocity[2]),
                    angular=Vector3(z=angular_speed)
                )
            )
        )

    def turn_radius(self, steering_angle):
        if steering_angle == 0:
            return math.inf
        else:
            radius = math.sqrt(self.center_of_mass_offset**2 + self.wheelbase_length**2 * 1 / math.tan(steering_angle)**2)
            return math.copysign(radius, steering_angle)

def main():
    try:
        odom_publisher = OdomPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
