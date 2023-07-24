#!/usr/bin/env python3

import smach
from base_state import *
import threading
import functools
import rospy

# TODO: Initialization timeout?

class Initialization(BaseState):
    def __init__(self, topic_monitors=[]):
        super().__init__(["running", "error", "exited"])
        # Block until all topics are ready
        self.wait_for_topics_ready(topic_monitors)
    def wait_for_topics_ready(self, topics=[]): # Topics in form of (topic name, topic type (for deserialization), predicate (optional, default to always true))
        # Deep clone topics
        self.wait_for_topics = topics.copy()
        # First check if all items are of length 2 or 3
        if not all(len(topic) in [2, 3] for topic in self.wait_for_topics):
            raise ValueError("All topics must be of length 2 or 3")
        # Check if there are duplicate topic names
        if len(self.wait_for_topics) != len(set(topic[0] for topic in self.wait_for_topics)):
            raise ValueError("There are duplicate topic names")
        # Then, if all items are of length 2, we add a dummy predicate (always true for any value)
        self.wait_for_topics = [topic + (lambda x: True,) if len(topic) == 2 else topic for topic in self.wait_for_topics]
        self.topic_ready_predicates = topic_ready_predicates.copy()
        # Now, we create a set of topic keys, which is the first element of each topic
        self.topic_keys = set(topic[0] for topic in self.wait_for_topics)
        # Now we need to subscribe to all the topics and link them to a callback that on message received:
        # - First, we evaluate the message against the predicate
        # - If the value returned is true, we remove the topic from the set of topic keys
        # - We unsubscribe from the topic for the callback
        # - We then call self.on_topic_update() to check if we can transition to running
        self.topic_subscribers = {}
        for topic in self.topic_keys:
            self.topic_subscribers[topic] = rospy.Subscriber(
                self.wait_for_topics[topic][0],
                self.wait_for_topics[topic][1],
                functools.partial(self.on_topic_update, topic)
            )
        # Now we start blocking until all topics are ready
        self.event = threading.Event()
        self.event.wait()
        return True

    def on_topic_update(self, topic, msg):
        # Check if topic is in topic keys, if not, return
        if topic not in self.topic_keys:
            return
        # Evaluate the predicate
        result = self.wait_for_topics[topic][2](msg)
        # If result is true, remove topic from topic keys
        if result:
            self.topic_keys.remove(topic)
            # Unsubscribe from topic
            self.topic_subscribers[topic].unregister()
            # Check if the set is empty, which implies transition to running and so, we can stop the blocking
            if len(self.topic_keys) == 0:
                self.event.set()

    def execute(self, userdata):
        # Wait for topics to be ready
        # self.wait_for_topics_ready