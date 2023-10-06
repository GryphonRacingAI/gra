import rospy
import smach
import time

class Shutdown(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exit'])

    def execute(self, userdata):
        rospy.loginfo('Running shutdown procedure...')
        time.sleep(1)
        rospy.loginfo('Shutdown complete.')
        return 'exit'