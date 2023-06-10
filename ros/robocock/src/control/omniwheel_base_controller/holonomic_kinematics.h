#include <Eigen/Dense>
#include <iostream>


/**
 * NOTES:
 * 
 * This is a new iteration of chassis kinematics that uses a robotics textbook's method, instead of my own derivation.
 * This was chosen because it is more generalizable, but the original derivation has a good geometric interpretation. However, 
 * since I am sure the current approach is correct, I will use this for now.
 * 
 * X is forward, Y is left, Z is up
*/

// Watch: https://www.youtube.com/watch?v=NcOT9hOsceE

/**
 * A struct to store the result of the inverse kinematics and the residual
*/
struct InverseKinematicsResult {
    Eigen::Vector3d result;
    Eigen::VectorXd residual;
};

/**
 * This class aims to purely do the kinematics of the chassis, and not the odometry.
*/
class HolonomicKinematics {
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
    public:
        /**
         * @brief Add a wheel to the chassis
         * 
         * @param x X coordinate of wheel in meters, in the chassis frame
         * @param y Y coordinate of wheel in meters, in the chassis frame
         * @param beta Angle of wheel in radians, from the unit vector along the x-axis
         * @param gamma Angle of free-spinning axis from the perpendicular axis to the driving axis, in radians (zero is perpendicular)
         * @param radius Radius of wheel in meters
        */
        void addWheel(double x, double y, double beta, double gamma, double radius) {
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
        /**
         * @brief Convenience function to add an omniwheel
         * 
         * @param x X coordinate of wheel in meters, in the chassis frame
         * @param y Y coordinate of wheel in meters, in the chassis frame
         * @param beta Angle of wheel in radians, from the unit vector along the x-axis
         * @param radius Radius of wheel in meters
        */
        void addOmniwheel(double x, double y, double beta, double radius) {
            addWheel(x, y, beta, 0, radius);
        }
        /**
         * @brief Convenience function to add a mecanum wheel
         * 
         * @param x X coordinate of wheel in meters, in the chassis frame
         * @param y Y coordinate of wheel in meters, in the chassis frame
         * @param beta Angle of wheel in radians, from the unit vector along the x-axis
         * @param radius Radius of wheel in meters
        */
        void addMecanum(double x, double y, double beta, double radius) {
            addWheel(x, y, beta, M_PI/4, radius);
        }
        /**
         * @brief Function to assert correctness of the kinematics, throws an error if incorrect
        */
        void assertCorrectness() {
            // Calculate rank of H0, if below 3, throw error saying too many degrees of freedom, maybe wheels are collinear, or not enough wheels
            if (H0.fullPivLu().rank() < 3) {
                throw std::runtime_error("Too many degrees of freedom, maybe wheels are collinear, or not enough wheels (3 required)");
            }
            // If above 3, throw error saying invalid configuration, maybe wheels are not coplanar
            if (H0.fullPivLu().rank() > 3) {
                throw std::runtime_error("Invalid wheel configuration");
            }
        }
        /**
         * @brief Convert a chassis velocity V_b to wheel velocities
         * X is forward, Y is left, Z is up
         * @param v_b Chassis velocity in the chassis frame
         * @return Eigen::VectorXd Wheel velocities
        */
        Eigen::VectorXd chassisV_bToWheelVelocities(const Eigen::Vector3d& v_b) {
            return H0 * v_b;
        }
        /**
         * @brief Convert a chassis command Q_dot to wheel velocities
         *
         * @param q_dot Chassis command in the chassis frame
         * @return Eigen::VectorXd Wheel velocities
        */
        Eigen::VectorXd chassisQ_dotToWheelVelocities(const Eigen::Vector3d& q_dot) {
            return Hphi * q_dot;
        }
        /**
         * @brief Calculate the least-squares estimate of the chassis velocity V_b from wheel velocities, and residual error vector
         * 
         * @param wheel_velocities Wheel velocities
         * @return InverseKinematicsResult Result of the inverse kinematics
        */
        InverseKinematicsResult wheelVelocitiesToChassisV_b(const Eigen::VectorXd& wheel_velocities) {
            InverseKinematicsResult result;
            result.result = H0_pinv * wheel_velocities;
            result.residual = wheel_velocities - H0 * result.result;
            return result;
        }
        /**
         * @brief Calculate the least-squares estimate of the chassis command Q_dot from wheel velocities, and residual error vector
         * 
         * @param wheel_velocities Wheel velocities
         * @return InverseKinematicsResult Result of the inverse kinematics
        */
        InverseKinematicsResult wheelVelocitiesToChassisQ_dot(const Eigen::VectorXd& wheel_velocities) {
            InverseKinematicsResult result;
            result.result = Hphi_pinv * wheel_velocities;
            result.residual = wheel_velocities - Hphi * result.result;
            return result;
        }
};

struct Pose2D {
    Eigen::Vector2d lin;
    double rot;
};

struct Twist2D {
    Eigen::Vector2d lin;
    double rot;
};

/**
 * @brief Update odometry using a q_dot (chassis command), last pose, and time delta
 * 
 * @param last_pose Last pose
 * @param q_dot Chassis command
 * @param dt Time delta
*/
const Pose2D& updateOdom(const Pose2D& last_pose, const Twist2D& q_dot, double dt) {
    Pose2D new_pose;
    new_pose.rot = last_pose.rot + q_dot.rot * dt;
    double delta_x, delta_y;
    if (q_dot.rot == 0) {
        delta_x = q_dot.lin.x() * dt;
        delta_y = q_dot.lin.y() * dt;
    } else {
        delta_x = (q_dot.lin.x() * sin(last_pose.rot + q_dot.rot * dt) + q_dot.lin.y() * cos(last_pose.rot + q_dot.rot * dt) - q_dot.lin.x() * sin(last_pose.rot) - q_dot.lin.y() * cos(last_pose.rot)) / q_dot.rot;
        delta_y = (q_dot.lin.y() * sin(last_pose.rot + q_dot.rot * dt) - q_dot.lin.x() * cos(last_pose.rot + q_dot.rot * dt) - q_dot.lin.y() * sin(last_pose.rot) + q_dot.lin.x() * cos(last_pose.rot)) / q_dot.rot;
    }
    new_pose.lin.x() = last_pose.lin.x() + delta_x;
    new_pose.lin.y() = last_pose.lin.y() + delta_y;
    return new_pose;
}