#!/usr/bin/env python3

import smach
from base_state import *
import threading
import functools
import rospy

# TODO: Initialization timeout?

class Initialization(BaseState):
    def __init__(self, topic_monitors=[]):
        super().__init__(["running", "error"])
        # Running occurs when all topics are ready
        # Error occurs when any of the nodes exit with a non-zero return code before all topics are ready, or if timeout occurs
        # Block until all topics are ready
        self.wait_for_topics_ready(topic_monitors)

    def wait_for_topics_ready(self, topic_monitors=[]): # Topics in form of (topic name, topic type (for deserialization), predicate (optional, default to always true))
        # Deep clone topics
        self.wait_for_topics = topic_monitors.copy()
        # First check if all items are of length 2 or 3
        if not all(len(topic) in [2, 3] for topic in self.wait_for_topics):
            raise ValueError("All topics must be of length 2 or 3")
        # Check if there are duplicate topic names
        if len(self.wait_for_topics) != len(set(topic[0] for topic in self.wait_for_topics)):
            raise ValueError("There are duplicate topic names")
        # Then, if all items are of length 2, we add a dummy predicate (always true for any value)
        self.wait_for_topics = [topic + (lambda x: True,) if len(topic) == 2 else topic for topic in self.wait_for_topics]
        # Now we convert this to a dictionary using the index in the list as the key
        self.wait_for_topics = {i: topic for i, topic in enumerate(self.wait_for_topics)}

        # Now we create a new dict that stores the subscription objects
        self.topic_subscribers = {}
        for i, topic in self.wait_for_topics.items():
            # Create a subscriber that calls a lambda function that calls the predicate on the message, and if true, unsubscribes from the topic, and removes the wait_for_topics entry, then calls self.on_topic_update()
            self.topic_subscribers[i] = rospy.Subscriber(topic[0], topic[1], lambda x: self.on_topic_update(i, x))

        # Now we start blocking until all topics are ready
        self.event = threading.Event()
        self.event.wait()
        return True

    def on_topic_update(self, i, msg):
        # Check if i is a key in the dict, if not early return
        if i not in self.topic_keys:
            return
        # Evaluate the predicate
        result = self.wait_for_topics[i][2](msg)
        # If result is true, unsubscribe and remove from self.topic_subscribers
        if result:
            self.topic_subscribers[i].unregister()
            del self.topic_subscribers[i]
            # Check if the set is empty, which implies transition to running and so, we can stop the blocking
            if len(self.topic_keys) == 0:
                self.event.set()

    def unsubscribe_all(self):
        for subscriber in self.topic_subscribers.values():
            subscriber.unregister()

    def execute(self, userdata):
        # Wait for topics to be ready
        # self.wait_for_topics_ready