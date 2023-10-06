#!/usr/bin/env python3

import smach

# Base state for smach which readily provides access to the roslaunch_smach instance during execution
class BaseState(smach.State):
    def __init__(self, outcomes):
        smach.State.__init__(self, outcomes=outcomes, input_keys=['_roslaunch_smach'])
        self.roslaunch_smach = None
    def execute(self, userdata):
        # TODO: Assert type for _roslaunch_smach, if fail raise error
        # Register roslaunch_smach instance
        self.roslaunch_smach = userdata["_roslaunch_smach"]
        self.roslaunch_smach.subscribe(self) # TODO: Provide these steps as a decorator (?)
    def update(self, result): # How to handle outcome of nodes exiting. Meant to be extended.
        pass
    def on_enter(self):
        pass
    def on_exit(self):
        pass