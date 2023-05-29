#include "ros/ros.h" // Import ROS library
#include "std_msgs/String.h" // Import String message type
// The above includes use ' " ' instead of ' < > ' means it will prioritize local files over system files

#include <sstream>

int main(int argc, char **argv) // We are now defining a function that takes argument "argc", which is the number of arguments, and "argv", which is an array of the arguments
{
    ros::init(argc, argv, "talker"); // Initialize ROS with the node name "talker"
    ros::NodeHandle n; // Create a node handle: it is reference assigned to a new node

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000); // Create a publisher object, able to push messages
    ros::Rate loop_rate(10); // Set the loop rate

    int count = 0; // Count how many messages we have sent

    while (ros::ok()) // All ros nodes must have a ros::ok() function. This function returns false if ROS has been shutdown, either through a call to ros::shutdown() or a Ctrl-C
    {
        std_msgs::String msg; // Create a new String ROS message
        std::stringstream ss; // Create a stringstream object
        ss << "hello world " << count; // Fill the stringstream with the desired content
        msg.data = ss.str(); // Store the content of the stringstream into the message data field

        ROS_INFO("%s", msg.data.c_str()); // Print the content of the message in the terminal

        chatter_pub.publish(msg); // Publish the message

        ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
        loop_rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
        ++count; // Increase our count variable
    }

    return 0;
}

// This is a basic talker implemented without much software engineering principles
// It could be improved by using classes, and using composition to separate the different functionalities