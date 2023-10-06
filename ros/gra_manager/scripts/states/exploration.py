import rospy
import smach
import time
import random

class Exploration(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'stop'], input_keys=['map'], output_keys=['map'])

    def execute(self, userdata):
        rospy.loginfo('Exploring...')
        time.sleep(1) # During this stage, user can trigger a stop any time. But we will ignore it for now.
        if random.random() < 0.1:
            # This is a failure case. However, failure simply means we don't have a map and we still return to ready state.
            # There may also be a case where it requires manual recovery. We do not have to handle that in this example as it should be encapsulated in the exploration state. Only if the exploration state cannot handle it, then it should return a failure.
            print('Failed to build a map. Returning to ready state.')
            userdata.map = None
            return 'ready'
        print('Successfully built a map. Returning to ready state.')
        userdata.map = 'map'
        return 'ready'