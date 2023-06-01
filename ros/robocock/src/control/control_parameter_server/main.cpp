#include "./control_parameter_server.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_parameter_server");
    // Create an instance of ControlParameterServer
    ControlParameterServer controlServer;
    ros::spin();
    return 0;
}