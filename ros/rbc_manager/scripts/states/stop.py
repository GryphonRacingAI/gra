import rospy
import smach

class Stop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'shutdown'], input_keys=['map'], output_keys=['map'])

    def execute(self, userdata):
        rospy.loginfo('Emergency stop. Press enter to go back to ready state.')
        _ = input()
        return 'ready'