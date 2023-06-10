#include <Eigen/Dense>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

/**
 * This is a new iteration of chassis kinematics that uses a robotics textbook's method, instead of my own derivation.
 * This was chosen because it is more generalizable, but the original derivation has a good geometric interpretation. However, 
 * since I am sure the current approach is correct, I will use this for now.
*/

// Watch: https://www.youtube.com/watch?v=NcOT9hOsceE

class Chassis {
    private:
        /**
         * @brief The H(0) matrix for the chassis (N rows, 3 columns)
        */
        Eigen::Matrix<double, Eigen::Dynamic, 3> H0;
        /**
         * @brief The H(phi) matrix for the chassis (N rows, 3 columns)
        */
        Eigen::Matrix<double, Eigen::Dynamic, 3> Hphi;
        /**
         * @brief The pseudo-inverse of H(0)
        */
        Eigen::Matrix<double, 3, Eigen::Dynamic> H0_pinv;
        /**
         * @brief The pseudo-inverse of H(phi)
        */
        Eigen::Matrix<double, 3, Eigen::Dynamic> Hphi_pinv;
        /**
         * @brief Vector of joint handles for the chassis
        */
        std::vector<hardware_interface::JointHandle> chassis_handles;
    public:
        void addWheel(double x, double y, double beta, double gamma, double radius, hardware_interface::JointHandle& handle) {
            Eigen::Matrix<double, 2, 3> lin_vel;
            lin_vel << -y, 1, 0,
                       x, 0, 1;

            Eigen::Matrix<double, 2, 2> rot;
            rot << cos(beta), sin(beta),
                   -sin(beta), cos(beta);
            
            Eigen::Matrix<double, 1, 2> extract_driving_comp;
            extract_driving_comp << 1, tan(gamma);
            
            Eigen::Matrix<double, 1, 3> H0_row;
            H0_row << 1/radius * extract_driving_comp * rot * lin_vel;

            // Add row to H0
            H0.conservativeResize(H0.rows()+1, Eigen::NoChange);
            H0.row(H0.rows()-1) = H0_row;

            // Add handle to chassis_handles
            chassis_handles.push_back(handle);

            // Compute pseudo-inverse of H0
            H0_pinv = (H0.transpose() * H0).inverse() * H0.transpose();

            // Update H(phi) by multipling [[1, 0, 0], [0, cos(phi), sin(phi)], [0, -sin(phi), cos(phi)]] to H0
            Eigen::Matrix<double, 3, 3> Hphi_update;
            Hphi_update << 1, 0, 0,
                           0, cos(beta), sin(beta),
                           0, -sin(beta), cos(beta);
            Hphi = Hphi_update * H0;       

            // Compute pseudo-inverse of Hphi
            Hphi_pinv = (Hphi.transpose() * Hphi).inverse() * Hphi.transpose();
        }
        void addOmniwheel(double x, double y, double beta, double radius, hardware_interface::JointHandle& handle) {
            addWheel(x, y, beta, 0, radius, handle);
        }
        void addMecanum(double x, double y, double beta, double radius, hardware_interface::JointHandle& handle) {
            addWheel(x, y, beta, M_PI/4, radius, handle);
        }
        void setChassisV_b(const Eigen::Vector3d& v_b) {
            Eigen::VectorXd wheel_velocities = H0 * v_b;
            for (int i = 0; i < chassis_handles.size(); i++) {
                chassis_handles[i].setCommand(wheel_velocities(i));
            }
            // Generated rank of H0 is always 3
        }
        void setChassisQ_dot(const Eigen::Vector3d& q_dot) {
            Eigen::VectorXd wheel_velocities = Hphi * q_dot;
            for (int i = 0; i < chassis_handles.size(); i++) {
                chassis_handles[i].setCommand(wheel_velocities(i));
            }
            // Generated rank of Hphi is always 3
        }

};