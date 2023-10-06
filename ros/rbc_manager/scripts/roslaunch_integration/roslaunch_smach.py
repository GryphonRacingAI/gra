#!/usr/bin/env python3

import roslaunch
import os
import threading

# Helper class monitoring a single launchfile
class MonitoredLaunchFileProcessListener(roslaunch.pmon.ProcessListener):
    def __init__(self, monitored_launchfile):
        self.monitored_launchfile = monitored_launchfile

    def process_died(self, name, exit_code):
        if self.monitored_launchfile.processes_exit_code is None:
            raise RuntimeError("Process died before launchfile was started, should not happen")
        if name not in self.monitored_launchfile.processes_exit_code:
            raise RuntimeError("Process died that was not in launchfile")
        
        self.monitored_launchfile.processes_exit_code[name] = exit_code
        self.monitored_launchfile._processes_exit_code_updated()

# ROSLaunch integration with smach
# User provides:
# - A launchfile that can launch any arbritrary number of nodes
# - A "wait_for_topics" array, which would be an array of topics that 
#   needs to start being broadcast for the system to be considered as ready
# - A "topic_ready_predicates" dict (for more advanced verification of system 
#   readiness), which maps topics to a predicate that should determine whether
#   a given message indicates that a particular part of the system is ready.
# After which, the node behaves as a nested state machine:
# - We start by default in the stage "initialization"
# - We wait for all conditions in "wait_for_topics" and "topic_ready_predicates" to return true, after which we enter stage "running"
# - If any of the nodes in this process die with a non-zero return code, we enter stage "error"
# - However, if a subset of nodes have exited with a zero return code, we assume its normal behaviour
# - If all of the nodes return with zero, we assume everything is fine, and we transition to "exited"

# A custom smach launch class that provides "initializing", "running", "initialization_timeout", "error" and "exited" states depending on the state of the processes.
# Class also exposes methods users can override, such as when it enters a state, or when a process exits.
class ROSLaunchSmach:
    def __init__(self, launchfiles=[], wait_for_topics=[], topic_ready_predicates={}):
        self.launchfiles = launchfiles
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, self.launchfiles,
                                                        process_listeners=[MonitoredLaunchFileProcessListener(self)])
        self.processes_exit_code = None
        self.observers = set()

        # wait_for_topics must be an array of tuples in the form of (topic_name, message_type)
        if not isinstance(wait_for_topics, list):
            raise TypeError("wait_for_topics must be an array")
        if not all(isinstance(x, tuple) for x in wait_for_topics):
            raise TypeError("wait_for_topics must be an array of tuples")
        if not all(len(x) == 2 for x in wait_for_topics):
            raise TypeError("wait_for_topics tuples must be of length 2")
        if not all(isinstance(x[0], str) for x in wait_for_topics):
            raise TypeError("wait_for_topics tuples first element must be a string")
        # Check if second element of tuple is a valid message type
        
        # Check if topic_ready_predicates is a dict of strings to functions
        if not isinstance(topic_ready_predicates, dict):
            raise TypeError("topic_ready_predicates must be a dict")
        if not all(isinstance(x, str) for x in topic_ready_predicates.keys()):
            raise TypeError("topic_ready_predicates keys must be strings")
        # Check if all values are functions (or None)
        if not all(callable(x) or x is None for x in topic_ready_predicates.values()):
            raise TypeError("topic_ready_predicates values must be functions or None")
        
        # Warn if wait_for_topics and topic_ready_predicates keys have overlap
        if len(set(wait_for_topics).intersection(set(topic_ready_predicates.keys()))) > 0:
            print("Warning: wait_for_topics and topic_ready_predicates keys overlap, topic_ready_predicates will take precedence")
        
        # Populate topic_ready_predicates with None if topic is in wait_for_topics but not a key in topic_ready_predicates
        for topic in wait_for_topics:
            if topic not in topic_ready_predicates:
                topic_ready_predicates[topic] = None

        # For all topics in topic_ready_predicates, subscribe
        self.topic_subscribers  = {}


        self.all_processes_exited = threading.Event() # Event that is set when all processes have exited, which is triggered by the process listener

    def start(self):
        self.launch.start()
        self.processes_exit_code = {x.process_name: None for x in self.launch.config.nodes}

    def _processes_exit_code_updated(self):
        if all(code is None for code in self.processes_exit_code.values()):
            print("All processes started")
            return
        if any(code == 1 for code in self.processes_exit_code.values()):
            print("Some processes exited with error")
            self.notify_observers(False)
            return
        if all(code == 0 for code in self.processes_exit_code.values()):
            print("All processes exited with success")
            self.notify_observers(True)
            self.all_processes_exited.set()
        else:
            print("Unknown exit code combination")

    def subscribe(self, observer):
        self.observers.add(observer)

    def unsubscribe(self, observer):
        self.observers.discard(observer)

    def notify_observers(self, result):
        for observer in self.observers:
            observer.update(result)

    def stop(self):
        self.launch.shutdown()
        self.all_processes_exited.set()

class Observer:
    def update(self, result):
        print(f"All processes exited, success: {result}")

# def main():
#     raw_launchfile = "~/catkin_ws/src/rbc_manager/launch/test.launch"
#     absolute_launchfile = os.path.expanduser(raw_launchfile)

#     monitored_launchfile = MonitoredLaunchfile(absolute_launchfile)
#     monitored_launchfile.start()

#     observer = Observer()
#     monitored_launchfile.subscribe(observer)

#     monitored_launchfile.all_processes_exited.wait()

# if __name__ == "__main__":
#     main()
    
# Todo: Passing out errors / exit codes if something goes wrong