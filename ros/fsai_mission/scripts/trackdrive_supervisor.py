#!/usr/bin/env python3

# Trackdrive lap count: 10
# This rosnode handles lap count and run completion logics, stopping the car after trackdrive completion.
# TODO: this rosnode should also handle the light indicator for mission completion on the ADS-DV.

import rospy
from std_msgs.msg import UInt16, Bool

class TrackdriveSupervisor:
    def __init__(self):
        rospy.init_node('trackdrive_supervisor', anonymous=False)

        self.chequered_flag_pub = rospy.Publisher('/chequered_flag', Bool, queue_size=10)
        rospy.Subscriber('/laps', UInt16, self.lap_callback)

        self.final_lap_detected = False
        rospy.loginfo("Trackdrive Supervisor node initialised")

    def lap_callback(self, msg):
        if msg.data == 11 and not self.final_lap_detected:
            rospy.loginfo("Final lap completion detected! Sending chequered flag signal.")
            self.chequered_flag_pub.publish(Bool(data=True))
            self.final_lap_detected = True

if __name__ == '__main__':
    try:
        supervisor = TrackdriveSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
