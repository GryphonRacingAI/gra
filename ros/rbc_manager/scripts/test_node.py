#!/usr/bin/env python3

import time
import rospy

if __name__ == '__main__':
    rospy.init_node('test_node')
    rospy.loginfo('test_node is running')
    time.sleep(2)
    rospy.loginfo('test_node is shutting down')
