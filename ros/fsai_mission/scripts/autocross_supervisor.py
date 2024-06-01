#!/usr/bin/env python3

# Autocross lap count: 1
# This rosnode handles lap count and run completion logics, stopping the car after autocross completion.
# TODO: this rosnode should also handles the light indicator for mission completion on the ADS-DV.

import rospy
from std_msgs.msg import UInt16

class AutocrossSupervisor:
    def __init__(self):
        rospy.init_node('autocross_supervisor', anonymous=False)

        self.chequered_flag_pub = rospy.Publisher('/chequered_flag', UInt16, queue_size=10)
        rospy.Subscriber('/laps', UInt16, self.lap_callback)

        self.final_lap_detected = False
        rospy.loginfo("Autocross Supervisor node initialised")

    def lap_callback(self, msg):
        if msg.data == 1 and not self.final_lap_detected:
            rospy.loginfo("Final lap completion detected! Sending chequered flag signal.")
            self.chequered_flag_pub.publish(UInt16(data=1))
            self.final_lap_detected = True

if __name__ == '__main__':
    try:
        supervisor = AutocrossSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
