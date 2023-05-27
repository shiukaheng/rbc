#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

void chatterCallback(const std_msgs::String::ConstPtr& msg) { // It takes in a message of type std_msgs::String, but with a ConstPtr typedef (so its shared)
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
    ros::spin();
    return 0;
}