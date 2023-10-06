import smach

class Manual(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'stop'], input_keys=['map'], output_keys=['map'])

    def execute(self, userdata):
        print('Entered manual control. Press enter to go back to ready state.')
        _ = input()
        return 'ready'