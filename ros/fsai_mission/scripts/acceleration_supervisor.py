#!/usr/bin/env python3

# Acceleration: non-loop track, stops after large orange cone pair
# This rosnode handles lap count and run completion logics, stopping the car after acceleration completion.
# TODO: this rosnode should also handles the light indicator for mission completion on the ADS-DV.

import rospy
from std_msgs.msg import UInt16

class AccelerationSupervisor:
    def __init__(self):
        rospy.init_node('acceleration_supervisor', anonymous=False)

        self.chequered_flag_pub = rospy.Publisher('/chequered_flag', UInt16, queue_size=10)
        rospy.Subscriber('/laps', UInt16, self.lap_callback)

        self.final_lap_detected = False
        rospy.loginfo("Acceleration Supervisor node initialised")

    def lap_callback(self, msg):
        if msg.data == 1 and not self.final_lap_detected:
            rospy.loginfo("Acceleration completion detected! Sending chequered flag signal.")
            self.chequered_flag_pub.publish(UInt16(data=1))
            self.final_lap_detected = True

if __name__ == '__main__':
    try:
        supervisor = AccelerationSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
