#include "./robocock_hw.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "robocock_hw");
    // Create an instance of ControlParameterServer
    RobocockHW robocockHW;
    robocockHW.start();
    return 0;
}