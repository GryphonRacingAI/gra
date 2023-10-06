import rospy
import smach
import time
import random

class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'not_initialized'])

    def execute(self, userdata):
        rospy.loginfo('Initializing...')

        """
        Pseudo code:
        - Run base bringup
        - Run lidar bringup
        - Run scanning camera bringup
        - Run SLAM bringup (slam_toolbox)
        - Run navigation bringup (move_base)
        - If any of the above fails, return 'not_initialized'
        - If all of the above succeeds, return 'initialized'
        - Can determine through roslaunch api's .is_alive() and .exit_code [https://github.com/ros/ros_comm/blob/030e132884d613e49a576d4339f0b8ec6f75d2d8/tools/roslaunch/src/roslaunch/nodeprocess.py]
        """

        time.sleep(1)
        if random.random() < 0.1:
            return 'not_initialized'
        return 'initialized'
    
class WaitReinitialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialize'])

    def execute(self, userdata):
        rospy.loginfo('Failed to initialize. Press enter to try again...')
        _ = input()
        return 'initialize'