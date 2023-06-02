#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
// #include <realtime_tools/realtime_publisher.h> - will be used later for odometry
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

class OmniwheelBaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        // ros::Subscriber cmd_vel_sub;
        // realtime_tools::RealtimeBuffer<geometry_msgs::Twist> command_buffer;
        void cmdVelCallback(const geometry_msgs::Twist& msg) {
            // command_buffer.writeFromNonRT(msg);
        }
        hardware_interface::JointHandle vel_handle_1;
        hardware_interface::JointHandle vel_handle_2;
    public:
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
            // cmd_vel_sub = nh.subscribe("cmd_vel", 1, &OmniwheelBaseController::cmdVelCallback, this);
            return true;
        }
        void update(const ros::Time& time, const ros::Duration& period) {
            // geometry_msgs::Twist command = *(command_buffer.readFromRT());
        }
        void starting(const ros::Time& time) {
            // Print a message whenever the controller is started
            ROS_INFO_STREAM("Starting OmniwheelBaseController");
        }
};

PLUGINLIB_EXPORT_CLASS(OmniwheelBaseController, controller_interface::ControllerBase);