#include "./robocock_hw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "robocock_hw");

    // Start a spinner to handle ROS callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create an instance of ControlParameterServer
    RobocockHW robocockHW;

    // Initialize the controller manager
    controller_manager::ControllerManager cm(&robocockHW, robocockHW.nh);

    while (ros::ok()) {
        // Update the robot state
        robocockHW.read();

        // Update the controllers
        cm.update(ros::Time::now(), robocockHW.control_rate->expectedCycleTime());

        // Send the new commands to the robot
        robocockHW.write();

        // Sleep for the remaining time until we hit our control rate
        robocockHW.control_rate->sleep();
    }

    spinner.stop();
    return 0;
}