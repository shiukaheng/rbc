#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <robocock/WheelStates.h>
#include <robocock/TargetWheelVelocities.h>
#include <controller_manager/controller_manager.h>

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
        ros::NodeHandle nh;
        ros::Publisher wheel_vel_pub;
        ros::Subscriber wheel_state_sub;

        void wheelStateCallback(const robocock::WheelStates& msg) {
            // TODO: Position (and possibly effort) feedback
            vel[0] = msg.wheel1_velocity;
            vel[1] = msg.wheel2_velocity;
            vel[2] = msg.wheel3_velocity;
            vel[3] = msg.wheel4_velocity;
        }

        float control_frequency;
        ros::Rate* control_rate;
        controller_manager::ControllerManager cm;
    
    public:
        RobocockHW() : cm(this, nh) {
            // Register the joints with the state and velocity interfaces

            hardware_interface::JointStateHandle state_handle_1("joint1", &pos[0], &vel[0], &eff[0]);
            jnt_state_interface.registerHandle(state_handle_1);
            hardware_interface::JointStateHandle state_handle_2("joint2", &pos[1], &vel[1], &eff[1]);
            jnt_state_interface.registerHandle(state_handle_2);
            hardware_interface::JointStateHandle state_handle_3("joint3", &pos[2], &vel[2], &eff[2]);
            jnt_state_interface.registerHandle(state_handle_3);
            hardware_interface::JointStateHandle state_handle_4("joint4", &pos[3], &vel[3], &eff[3]);
            jnt_state_interface.registerHandle(state_handle_4);

            registerInterface(&jnt_state_interface);

            hardware_interface::JointHandle vel_handle_1(jnt_state_interface.getHandle("joint1"), &cmd[0]);
            jnt_vel_interface.registerHandle(vel_handle_1);
            hardware_interface::JointHandle vel_handle_2(jnt_state_interface.getHandle("joint2"), &cmd[1]);
            jnt_vel_interface.registerHandle(vel_handle_2);
            hardware_interface::JointHandle vel_handle_3(jnt_state_interface.getHandle("joint3"), &cmd[2]);
            jnt_vel_interface.registerHandle(vel_handle_3);
            hardware_interface::JointHandle vel_handle_4(jnt_state_interface.getHandle("joint4"), &cmd[3]);
            jnt_vel_interface.registerHandle(vel_handle_4);

            registerInterface(&jnt_vel_interface);

            // Initialize the publisher and subscriber
            wheel_vel_pub = nh.advertise<robocock::TargetWheelVelocities>("target_wheel_velocities", 1000);
            wheel_state_sub = nh.subscribe("wheel_states", 1000, &RobocockHW::wheelStateCallback, this);

            // Get the controller frequency
            nh.param<float>("control_frequency", control_frequency, 100.0);

            // Initialize the control rate
            control_rate = new ros::Rate(control_frequency);
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
        void start() {
            // Start the control loop
            while (ros::ok()) {
                // No read() function because it is implemented in the subscriber callback
                ros::spinOnce();
                write();
                cm.update(ros::Time::now(), control_rate->expectedCycleTime());
                control_rate->sleep();
            }
        }
        ~RobocockHW() {
            delete control_rate;
        }
};