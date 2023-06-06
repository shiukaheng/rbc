#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include "./omniwheel_inverse_kinematics.h"
#include <tf2_ros/transform_broadcaster.h>
#include <realtime_tools/realtime_publisher.h>

// TODO: Implement cmd_vel and odometry, but remember to use realtime_tools

class OmniwheelBaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        hardware_interface::JointHandle vel_handle_1;
        hardware_interface::JointHandle vel_handle_2;
        hardware_interface::JointHandle vel_handle_3;
        hardware_interface::JointHandle vel_handle_4;
        realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
        Base base;
        ros::Subscriber cmd_vel_sub;
        realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub;
        tf2_ros::TransformBroadcaster tf_broadcaster;
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
                Wheel(-0.25, -0.25, 315, 0.06, vel_handle_1)
            );
            base.addWheel(
                Wheel(-0.25, 0.25, 225, 0.06, vel_handle_2)
            );
            base.addWheel(
                Wheel(0.25, 0.25, 135, 0.06, vel_handle_3)
            );
            base.addWheel(
                Wheel(0.25, -0.25, 45, 0.06, vel_handle_4)
            );

            // Subscribe to cmd_vel
            cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &OmniwheelBaseController::cmd_vel_callback, this);

            // Setup odometry publisher
            odom_pub.init(nh, "/odom", 1);

            return true;
        }
        void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
            cmd_vel_buffer_.writeFromNonRT(cmd_vel);
        }
        void update(const ros::Time& time, const ros::Duration& period) {
            if (cmd_vel_buffer_.readFromRT()) {
                // Read cmd_vel
                geometry_msgs::Twist cmd_vel = *(cmd_vel_buffer_.readFromRT());
                // Update wheel target velocities and get odometry
                const nav_msgs::Odometry& odom = base.update(cmd_vel, time, period);
                // Publish odometry
                if (odom_pub.trylock()) {
                    odom_pub.msg_ = odom;
                    odom_pub.unlockAndPublish();
                    geometry_msgs::TransformStamped odom_tf;
                    odom_tf.header.stamp = time;
                    odom_tf.header.frame_id = "odom";
                    odom_tf.child_frame_id = "base_link";
                    odom_tf.transform.translation.x = odom.pose.pose.position.x;
                    odom_tf.transform.translation.y = odom.pose.pose.position.y;
                    odom_tf.transform.translation.z = odom.pose.pose.position.z;
                    odom_tf.transform.rotation = odom.pose.pose.orientation;
                    tf_broadcaster.sendTransform(odom_tf);
                } else {
                    ROS_WARN_STREAM("Failed to publish odom");
                }
            }
        }
        void starting(const ros::Time& time) {
            ROS_INFO_STREAM("Starting OmniwheelBaseController");
        }
        void stopping(const ros::Time& time) {
            ROS_INFO_STREAM("Stopping OmniwheelBaseController");
            base.breakWheels();
        }
};

PLUGINLIB_EXPORT_CLASS(OmniwheelBaseController, controller_interface::ControllerBase);