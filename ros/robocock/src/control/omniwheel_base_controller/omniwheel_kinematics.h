#include <Eigen/Dense>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

const Eigen::Matrix2d jacobian = [](){
    Eigen::Matrix2d jacobian;
    jacobian << 0, 1, -1, 0;
    return jacobian;
}();

class Wheel {
    public:
        Wheel(double x, double y, double theta, double radius, hardware_interface::JointHandle& handle) {
            setProperties(x, y, theta, radius);
            setHandle(handle);
        }
        void setProperties(double x, double y, double theta, double radius) {
            wheel_position_vec = Eigen::Vector2d(x, y);
            wheel_direction_vec = Eigen::Vector2d(cos(theta), sin(theta));
            wheel_radius = radius;
        }
        void setHandle(hardware_interface::JointHandle& handle) {
            wheel_handle = handle;
        }
        void update(const geometry_msgs::Twist& twist) {
            // Getting variables
            Eigen::Vector2d base_linear_velocity_vec = Eigen::Vector2d(twist.linear.x, twist.linear.y);
            double base_angular_velocity = twist.angular.z;
            // Deriving tangential velocity (because of base rotation)
            Eigen::Vector2d base_tangential_velocity_vec = base_angular_velocity * ( jacobian * wheel_position_vec );
            // Deriving wheel angular velocity by adding tangential and linear velocity vectors, then projecting onto wheel direction; finally factoring in wheel radius
            double wheel_angular_velocity = (base_linear_velocity_vec + base_tangential_velocity_vec).dot(wheel_direction_vec) / wheel_radius;
            wheel_handle.setCommand(wheel_angular_velocity);
        }
    private:
        Eigen::Vector2d wheel_position_vec = Eigen::Vector2d(0, 0);
        Eigen::Vector2d wheel_direction_vec = Eigen::Vector2d(1, 0);
        double wheel_radius = 1;
        hardware_interface::JointHandle wheel_handle;
};

class WheelSet {
    public:
        void addWheel(const Wheel& wheel) {
            wheels.push_back(wheel);
        }
        void update(const geometry_msgs::Twist& twist) {
            for (auto& wheel : wheels) {
                wheel.update(twist);
            }
        }
    private:
        std::vector<Wheel> wheels;
};