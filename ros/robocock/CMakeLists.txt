cmake_minimum_required(VERSION 3.0.2)
project(robocock)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg message_generation controller_manager hardware_interface)

add_message_files(
    FILES
    WheelStates.msg
    TargetWheelVelocities.msg
    WheelPIDParameters.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package(
    CATKIN_DEPENDS message_runtime
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
# add_dependencies(talker robocock_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
# add_dependencies(listener robocock_generate_messages_cpp)

add_executable(control_parameter_server src/control/control_parameter_server/main.cpp)
target_link_libraries(control_parameter_server ${catkin_LIBRARIES})

add_executable(robocock_hw src/control/hardware/main.cpp)
target_link_libraries(robocock_hw ${catkin_LIBRARIES})

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.perspective"
)