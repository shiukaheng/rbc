#!/usr/bin/env python3

import rospy
from robocock.msg import TargetWheelVelocities, WheelStates
import numpy as np
import matplotlib.pyplot as plt
import json
import os
import sys

def _generate_triangular_velocities(samples):
    # Create a ramp of velocities from 0 -> +max_vel -> 0 -> -max_vel -> 0, linearly interpolated
    # samples is the total number of samples in the ramp
    return np.concatenate((np.linspace(0, 1, samples/4), np.linspace(1, 0, samples/4), np.linspace(0, -1, samples/4), np.linspace(-1, 0, samples/4)))

def _generate_n_triangular_velocities(samples, n=1):
    # Generate n triangular velocities
    return np.concatenate([_generate_triangular_velocities(samples/n) for i in range(n)])

def create_ramp(pub, max_vel, duration, n=1, transfer_function=lambda x: x, rate=30):
    # Generate a ramp of velocities
    ramp = _generate_n_triangular_velocities(duration*rate, n)
    transformed_ramp = [transfer_function(vel) * max_vel for vel in ramp]
    return transformed_ramp

def execute_ramp(pub, ramp, rate=30):
    # Create subscriber to wheel_states
    wheel_states = []
    wheel_commands = []
    def wheel_states_callback(msg):
        wheel_states.append({
            "time": rospy.Time.now().to_sec(),
            "wheel1": (msg.wheel1_velocity, msg.wheel1_setpoint, msg.wheel1_output, msg.wheel1_position),
            "wheel2": (msg.wheel2_velocity, msg.wheel2_setpoint, msg.wheel2_output, msg.wheel2_position),
            "wheel3": (msg.wheel3_velocity, msg.wheel3_setpoint, msg.wheel3_output, msg.wheel3_position),
            "wheel4": (msg.wheel4_velocity, msg.wheel4_setpoint, msg.wheel4_output, msg.wheel4_position),
        })
    sub = rospy.Subscriber('wheel_states', WheelStates, wheel_states_callback)
    # Create a ROS message
    msg = TargetWheelVelocities()
    # Create a rate object
    r = rospy.Rate(rate)
    # Loop through the ramp
    for vel in ramp:
        msg.wheel1 = vel
        msg.wheel2 = vel
        msg.wheel3 = vel
        msg.wheel4 = vel
        # Publish the message
        pub.publish(msg)
        # Save when the command was sent and the command
        wheel_commands.append({
            "time": rospy.Time.now().to_sec(),
            "wheel_velocity": vel
        })
        # Sleep for the rate
        r.sleep()
    # Wait for the last command to be executed
    rospy.sleep(0.5)
    # Unsubscribe from wheel_states
    sub.unregister()
    # Return the wheel_states and wheel_commands
    return {
        "wheel_states": wheel_states,
        "wheel_commands": wheel_commands
    }

def plot_response(data):

    wheel_states = data["wheel_states"]
    wheel_commands = data["wheel_commands"]

    # Initialize figure and axis
    fig, ax = plt.subplots()

    # Extract command times and velocities
    command_velocities = [command["wheel_velocity"] for command in wheel_commands]
    command_times = [command["time"] for command in wheel_commands]

    # Extract wheel state times and velocities for each wheel
    state_times = [state["time"] for state in wheel_states]
    wheel1_velocities = [state["wheel1"][0] for state in wheel_states]
    wheel2_velocities = [state["wheel2"][0] for state in wheel_states]
    wheel3_velocities = [state["wheel3"][0] for state in wheel_states]
    wheel4_velocities = [state["wheel4"][0] for state in wheel_states]

    # Plot commanded velocities
    ax.plot(command_times, command_velocities, label='Commanded velocities', linestyle='--', color='k')  # made dashed line for visibility

    # Plot wheel velocities
    ax.plot(state_times, wheel1_velocities, label='Wheel 1 velocities')
    ax.plot(state_times, wheel2_velocities, label='Wheel 2 velocities')
    ax.plot(state_times, wheel3_velocities, label='Wheel 3 velocities')
    ax.plot(state_times, wheel4_velocities, label='Wheel 4 velocities')

    # Set the y-axis limit to be between the min and max of the commanded velocities
    ax.set_ylim([min(command_velocities), max(command_velocities)])

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity')
    ax.legend(loc='upper right')

    # Display the plot
    plt.show()

def save_response(data, filename=None):
    # If no filename is given, use the current time
    if filename is None:
        filename = str(rospy.Time.now().to_sec()) + ".json"
    # Save the data to a file
    with open(filename, 'w') as f:
        json.dump(data, f)
    # Log full path of where the data was saved
    rospy.loginfo("Saved data to {}".format(os.path.abspath(filename)))

def main():
    rospy.init_node('controller_stress_test', anonymous=True)

    # Wait for /wheel_states to start publishing
    rospy.loginfo("Waiting for /wheel_states to start publishing...")
    rospy.wait_for_message('wheel_states', WheelStates)

    # Log start
    rospy.loginfo("Starting controller stress test")

    # Create a topic publisher
    pub = rospy.Publisher('target_wheel_velocities', TargetWheelVelocities, queue_size=10)

    # Wait for the publisher to be ready
    rospy.sleep(0.5)

    # Create a ramp
    rate = 30
    ramp = create_ramp(
        pub,
        max_vel=10,
        duration=15,
        n=1,
        transfer_function=lambda x: x,
        rate=rate
    )
    # Execute the ramp
    data = execute_ramp(pub, ramp, rate)

    # Plot the response
    plot_response(data)
    save_response(data)
    # Log done
    rospy.loginfo("Done")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
