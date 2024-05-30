#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt16
from ackermann_msgs.msg import AckermannDrive

def convert_to_angle(steering_angle, speed):
    motor_angle=int(99+80*speed)
    servo_angle=int(51/2+(51/6)*steering_angle)
    return motor_angle, servo_angle

def cmd_vel_callback(msg):
    # steering_angle=msg.angular.z
    # speed=msg.linear.x

    steering_angle=msg.steering_angle
    speed=msg.speed

    motor_angle, servo_angle=convert_to_angle(steering_angle, speed)

    motor_publisher.publish(motor_angle)
    servo_publisher.publish(servo_angle)

def listener():
    rospy.init_node('angle_converter',anonymous=False)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_callback)
    global motor_publisher,servo_publisher
    motor_publisher=rospy.Publisher('/motor_angle',UInt16,queue_size=3)
    servo_publisher=rospy.Publisher('/servo_angle',UInt16,queue_size=3)

    rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

