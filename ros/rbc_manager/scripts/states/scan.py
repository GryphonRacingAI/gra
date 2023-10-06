import smach
import time
import random

class Scan(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['ready', 'stop'], input_keys=['map'], output_keys=['map', 'scan'])

    def execute(self, userdata):
        if userdata['map'] is None:
            print('Cannot scan without a map. Returning to ready state.')
            return 'ready'
        print('Scanning...')
        time.sleep(1) # During this stage, user can trigger a stop any time. But we will ignore it for now.
        if random.random() < 0.1:
            # This is a failure case. However, failure simply means we don't have a scan and we still return to ready state.
            # There may also be a case where it requires manual recovery. We do not have to handle that in this example as it should be encapsulated in the scan state. Only if the scan state cannot handle it, then it should return a failure.
            print('Failed to scan. Returning to ready state.')
            userdata.scan = None
            return 'ready'
        print('Successfully scanned. Returning to ready state.')
        userdata.scan = 'scan'
        return 'ready'