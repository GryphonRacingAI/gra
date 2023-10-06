import rospy
import smach

class Ready(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['exploration', 'scan', 'manual', 'shutdown'], input_keys=['map', 'scan'], output_keys=['map', 'scan'])
        

    def execute(self, userdata):
        rospy.loginfo('Ready. Choose a mode: exploration, scan, manual, shutdown')
        mode = input()
        if mode == 'exploration':
            return 'exploration'
        elif mode == 'scan':
            return 'scan'
        elif mode == 'manual':
            return 'manual'
        else: # Must be shutdown
            return 'shutdown'