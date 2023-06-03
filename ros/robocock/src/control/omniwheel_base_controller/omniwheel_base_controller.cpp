#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h> - will be used later for odometry
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// TODO: Implement cmd_vel and odometry, but remember to use realtime_tools

class OmniwheelBaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        hardware_interface::JointHandle vel_handle_1;
    public:
        OmniwheelBaseController() {
            ROS_INFO_STREAM("Constructing OmniwheelBaseController");
        }
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
            ROS_INFO_STREAM("Initializing OmniwheelBaseController");
            vel_handle_1 = hw->getHandle("joint1");
            ROS_INFO_STREAM("Got handle for joint1");
            return true;
        }
        void update(const ros::Time& time, const ros::Duration& period) {
            // ROS_INFO_STREAM("Updating OmniwheelBaseController");
        }
        void starting(const ros::Time& time) {
            ROS_INFO_STREAM("Starting OmniwheelBaseController");
        }
        void stopping(const ros::Time& time) {
            ROS_INFO_STREAM("Stopping OmniwheelBaseController");
        }
};

PLUGINLIB_EXPORT_CLASS(OmniwheelBaseController, controller_interface::ControllerBase);