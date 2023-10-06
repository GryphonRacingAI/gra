#!/usr/bin/env python3

import rospy
import smach

from states.initialization import Initialization, WaitReinitialize
from states.ready import Ready
from states.exploration import Exploration
from states.scan import Scan
from states.manual import Manual
from states.stop import Stop
from states.shutdown import Shutdown

def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['exit'])
    sm.userdata.map = None
    sm.userdata.scan = None
    
    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('INITIALIZATION', Initialization(), 
                               transitions={'initialized':'READY', 
                                            'not_initialized':'WAIT_REINITIALIZE'})
        smach.StateMachine.add('WAIT_REINITIALIZE', WaitReinitialize(),
                                 transitions={'initialize':'INITIALIZATION'})
        smach.StateMachine.add('READY', Ready(), 
                               transitions={'exploration':'EXPLORATION', 
                                            'scan':'SCAN',
                                            'manual':'MANUAL',
                                            'shutdown':'SHUTDOWN'})
        smach.StateMachine.add('EXPLORATION', Exploration(), 
                               transitions={'ready':'READY', 
                                            'stop':'STOP'})
        smach.StateMachine.add('SCAN', Scan(), 
                               transitions={'ready':'READY', 
                                            'stop':'STOP'})
        smach.StateMachine.add('MANUAL', Manual(), 
                               transitions={'ready':'READY', 
                                            'stop':'STOP'})
        smach.StateMachine.add('STOP', Stop(), 
                               transitions={'ready':'READY',
                                            'shutdown':'SHUTDOWN'})
        smach.StateMachine.add('SHUTDOWN', Shutdown(), 
                                 transitions={'exit':'exit'})
        
    # Execute SMACH plan
    outcome = sm.execute()
    print(outcome)

if __name__ == '__main__':
    main() 