#!/usr/bin/env python3

import rospy
import subprocess
from fsai_api.msg import VCU2AI

class MissionSupervisor:
    def __init__(self):
        rospy.init_node('mission_supervisor', anonymous=True)
        
        # Subscriber to the VCU2AI message
        rospy.Subscriber('/vcu2ai', VCU2AI, self.vcu2ai_callback)
        
        # Store the current AMI_STATE
        self.current_ami_state = None

    def vcu2ai_callback(self, msg):
        # Check the AS_STATE
        if msg.as_state == 3:  # AS_DRIVING
            self.start_mission(msg.ami_state)

    def start_mission(self, ami_state):
        if self.current_ami_state == ami_state:
            return  # Avoid restarting the same mission
        
        self.current_ami_state = ami_state

        # Determine which mission to start
        if ami_state == 1:
            self.start_launch_file('acceleration.launch')
        elif ami_state == 2:
            self.start_launch_file('skidpad.launch')
        elif ami_state == 3:
            self.start_launch_file('autocross.launch')
        elif ami_state == 4:
            self.start_launch_file('trackdrive.launch')
        elif ami_state == 5:
            self.start_launch_file('static_inspection_A.launch')
        elif ami_state == 6:
            self.start_launch_file('static_inspection_B.launch')
        elif ami_state == 7:
            self.start_launch_file('autonomous_demo.launch')
        else:
            rospy.logwarn("AMI_STATE not recognised or not selected.")

    def start_launch_file(self, launch_file):
        rospy.loginfo("Starting mission: %s", launch_file)
        try:
            subprocess.Popen(["roslaunch", "fsai_mission", launch_file])
        except Exception as e:
            rospy.logerr("Failed to start mission: %s", e)

if __name__ == '__main__':
    try:
        supervisor = MissionSupervisor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
