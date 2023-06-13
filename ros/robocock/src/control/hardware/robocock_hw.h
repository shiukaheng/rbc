#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robocock/WheelStates.h>
#include <robocock/TargetWheelVelocities.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>

class RobocockHW : public hardware_interface::RobotHW {
    private:
        hardware_interface::JointStateInterface jnt_state_interface; // This is the interface for reading the state of the joints
        hardware_interface::VelocityJointInterface jnt_vel_interface; // This is the interface for commanding the joints

        // We then need to store the state and command of the joints
        double cmd[4] = {0.0, 0.0, 0.0, 0.0};
        double pos[4] = {0.0, 0.0, 0.0, 0.0};
        double vel[4] = {0.0, 0.0, 0.0, 0.0};
        double eff[4] = {0.0, 0.0, 0.0, 0.0};

        // ROS node
        ros::Publisher wheel_vel_pub;
        ros::Subscriber wheel_state_sub;
        // ros::Publisher joint_state_pub;

        void wheelStateCallback(const robocock::WheelStates& msg) {
            vel[0] = msg.wheel1_velocity;
            vel[1] = msg.wheel2_velocity;
            vel[2] = msg.wheel3_velocity;
            vel[3] = msg.wheel4_velocity;
            pos[0] = msg.wheel1_position;
            pos[1] = msg.wheel2_position;
            pos[2] = msg.wheel3_position;
            pos[3] = msg.wheel4_position;
            // Not sure if PWM is a good measure of effort, but it's the only thing we have for now
            eff[0] = msg.wheel1_output;
            eff[1] = msg.wheel2_output;
            eff[2] = msg.wheel3_output;
            eff[3] = msg.wheel4_output;

            // Now, we publish the joint states
            // sensor_msgs::JointState joint_state_msg;
            // joint_state_msg.header.stamp = ros::Time::now();
            // joint_state_msg.name = {"wheel_1_joint", "wheel_2_joint", "wheel_3_joint", "wheel_4_joint"};
            // joint_state_msg.position = {pos[0], pos[1], pos[2], pos[3]};
            // joint_state_msg.velocity = {vel[0], vel[1], vel[2], vel[3]};
            // joint_state_msg.effort = {eff[0], eff[1], eff[2], eff[3]};
            // joint_state_pub.publish(joint_state_msg);
        }

        float control_frequency;
    
    public:
        ros::NodeHandle nh;
        ros::Rate* control_rate;
        RobocockHW() {
            ROS_INFO("Initializing robocock hardware interface");
            // Register the joints with the state and velocity interfaces

            hardware_interface::JointStateHandle state_handle_1("wheel_1_joint", &pos[0], &vel[0], &eff[0]);
            jnt_state_interface.registerHandle(state_handle_1);
            hardware_interface::JointStateHandle state_handle_2("wheel_2_joint", &pos[1], &vel[1], &eff[1]);
            jnt_state_interface.registerHandle(state_handle_2);
            hardware_interface::JointStateHandle state_handle_3("wheel_3_joint", &pos[2], &vel[2], &eff[2]);
            jnt_state_interface.registerHandle(state_handle_3);
            hardware_interface::JointStateHandle state_handle_4("wheel_4_joint", &pos[3], &vel[3], &eff[3]);
            jnt_state_interface.registerHandle(state_handle_4);

            registerInterface(&jnt_state_interface);

            hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("wheel_1_joint"), &cmd[0]);
            jnt_vel_interface.registerHandle(vel_handle_1);
            hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("wheel_2_joint"), &cmd[1]);
            jnt_vel_interface.registerHandle(vel_handle_2);
            hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("wheel_3_joint"), &cmd[2]);
            jnt_vel_interface.registerHandle(vel_handle_3);
            hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("wheel_4_joint"), &cmd[3]);
            jnt_vel_interface.registerHandle(vel_handle_4);

            registerInterface(&jnt_vel_interface);

            // Initialize the publisher and subscriber
            wheel_vel_pub = nh.advertise<robocock::TargetWheelVelocities>("target_wheel_velocities", 1000);
            // joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1000);
            wheel_state_sub = nh.subscribe("wheel_states", 1000, &RobocockHW::wheelStateCallback, this);

            // Get the controller frequency
            nh.param<float>("control_frequency", control_frequency, 100.0);

            // Initialize the control rate
            control_rate = new ros::Rate(control_frequency);

            ROS_INFO("Robocock hardware interface initialized");
        }
        void write() {
            // Publish the target wheel velocities
            robocock::TargetWheelVelocities msg;
            msg.wheel1 = cmd[0];
            msg.wheel2 = cmd[1];
            msg.wheel3 = cmd[2];
            msg.wheel4 = cmd[3];
            wheel_vel_pub.publish(msg);
        }
        void read() {
        }
        ~RobocockHW() {
            delete control_rate;
        }
};