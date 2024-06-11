#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist

def publish_velocity():
    rospy.init_node('static_inspection_B', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    vel_msg = Twist()
    vel_msg.linear.x = 50*2*math.pi/60

    # Publish the message in a loop until ROS is shut down
    while not rospy.is_shutdown():
        pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        publish_velocity()
    except rospy.ROSInterruptException:
        pass
