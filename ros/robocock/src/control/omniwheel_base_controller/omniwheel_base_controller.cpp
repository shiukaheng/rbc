#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h> - will be used later for odometry
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "./omniwheel_kinematics.h"

// TODO: Implement cmd_vel and odometry, but remember to use realtime_tools

class OmniwheelBaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        hardware_interface::JointHandle vel_handle_1;
        hardware_interface::JointHandle vel_handle_2;
        hardware_interface::JointHandle vel_handle_3;
        hardware_interface::JointHandle vel_handle_4;
        realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
        WheelSet base;
        ros::Subscriber cmd_vel_sub;
    public:
        OmniwheelBaseController() {
            ROS_INFO_STREAM("Constructing OmniwheelBaseController");
        }
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
            ROS_INFO_STREAM("Initializing OmniwheelBaseController");
            vel_handle_1 = hw->getHandle("joint1");
            vel_handle_2 = hw->getHandle("joint2");
            vel_handle_3 = hw->getHandle("joint3");
            vel_handle_4 = hw->getHandle("joint4");
            ROS_INFO_STREAM("Got handle for joints");

            // Configure base
            base.addWheel(
                Wheel(-0.25, -0.25, 0, 0.06, vel_handle_1)
            );
            base.addWheel(
                Wheel(-0.25, 0.25, 0, 0.06, vel_handle_2)
            );
            base.addWheel(
                Wheel(0.25, 0.25, 0, 0.06, vel_handle_3)
            );
            base.addWheel(
                Wheel(0.25, -0.25, 0, 0.06, vel_handle_4)
            );

            // Subscribe to cmd_vel
            cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &OmniwheelBaseController::cmd_vel_callback, this);
            return true;
        }
        void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
            cmd_vel_buffer_.writeFromNonRT(cmd_vel);
        }
        void update(const ros::Time& time, const ros::Duration& period) {
            // Read cmd_vel
            geometry_msgs::Twist cmd_vel = *(cmd_vel_buffer_.readFromRT());
            // Calculate wheel velocities
            base.update(cmd_vel);
        }
        void starting(const ros::Time& time) {
            ROS_INFO_STREAM("Starting OmniwheelBaseController");
            // Print namespace
        }
        void stopping(const ros::Time& time) {
            ROS_INFO_STREAM("Stopping OmniwheelBaseController");
        }
};

PLUGINLIB_EXPORT_CLASS(OmniwheelBaseController, controller_interface::ControllerBase);