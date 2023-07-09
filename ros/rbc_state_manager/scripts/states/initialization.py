import rospy
import smach
import time
import random

class Initialization(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialized', 'not_initialized'])

    def execute(self, userdata):
        rospy.loginfo('Initializing...')
        time.sleep(1)
        if random.random() < 0.1:
            return 'not_initialized'
        return 'initialized'
    
class WaitReinitialize(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['initialize'])

    def execute(self, userdata):
        rospy.loginfo('Failed to initialize. Press enter to try again...')
        _ = input()
        return 'initialize'