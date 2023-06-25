#!/usr/bin/env python3

from __future__ import annotations
import rospy

from robocock.msg import BaseState, BaseSetpoint
from std_msgs.msg import Float32

import threading

def main():
    # Subscribe to /base_state and split the message into individual topics

    # Create a publisher for each motor
    motors = 4
    properties = ['i_accumulator', 'output', 'error', 'delta_ticks', 'velocity', 'position', 'acceleration']
    pubs: list[rospy.Publisher] = []

    for i in range(motors):
        for prop in properties:
            pub = rospy.Publisher('motor_' + str(i+1) + '_' + prop, Float32, queue_size=10)
            pubs.append(pub)

    def callback(msg: BaseState):
        # Split the message into individual topics
        for i in range(motors):
            for prop in properties:
                pub = pubs[i*len(properties) + properties.index(prop)]
                # Print properties of motor 0
                pub.publish(msg.states[i].__getattribute__(prop))

    rospy.init_node('debug_topics')
    sub = rospy.Subscriber('base_state', BaseState, callback)
    rospy.loginfo(f"Subscribed to /base_state, publishing {motors*len(properties)} topics in format /motor_<motor>_<property>")
    rospy.loginfo(f"Available properties: {properties}")
    
    # Create a publisher for the BaseSetpoint message
    pub = rospy.Publisher('base_setpoint', BaseSetpoint, queue_size=10)

    # Create a BaseSetpoint message
    base_setpoint_msg : BaseSetpoint = BaseSetpoint()

    # Set all setpoints at once
    def set_setpoints(setpoint: float):
        for i in range(motors):
            base_setpoint_msg.setpoints[i].velocity = setpoint
        # rospy.loginfo(f"Setting all setpoints to {setpoint}")
        pub.publish(base_setpoint_msg)

    rospy.Subscriber('set_setpoints', Float32, lambda msg: set_setpoints(msg.data))

    # Spin
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass