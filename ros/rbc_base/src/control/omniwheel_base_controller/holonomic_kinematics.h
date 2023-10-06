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
    Eigen::Vector3d twist;
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
            // std::cout << "Adding row to H0: " << H0_row << std::endl;
            H0.conservativeResize(H0.rows()+1, Eigen::NoChange);
            H0.row(H0.rows()-1) = H0_row;

            // Print H0
            // std::cout << "H0: " << std::endl << H0 << std::endl;

            // Compute pseudo-inverse of H0
            // std::cout << "Computing pseudo-inverse of H0" << std::endl;
            H0_pinv = (H0.transpose() * H0).inverse() * H0.transpose();
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
        bool isCorrectConfiguration() {
            return H0.fullPivLu().rank() >= 3;
        }
        /**
         * @brief Convert a chassis velocity V_b to wheel velocities
         * X is forward, Y is left, Z is up
         * @param v_b Chassis velocity in the chassis frame
         * @return Eigen::VectorXd Wheel velocities
        */
        Eigen::VectorXd forwardVb(const Eigen::Vector3d& v_b) {
            // Print debug
            // std::cout << "Calculating forward kinematics: " << std::endl;
            // std::cout << "V_b: " << v_b << std::endl;
            // std::cout << "H0: " << H0 << std::endl;
            return H0 * v_b;
        }
        /**
         * @brief Calculate the least-squares estimate of the chassis velocity V_b from wheel velocities, and residual error vector
         * 
         * @param wheel_velocities Wheel velocities
         * @return InverseKinematicsResult Result of the inverse kinematics
        */
        InverseKinematicsResult inverseVb(const Eigen::VectorXd& wheel_velocities) {
            // Print debug
            // std::cout << "Calculating inverse kinematics: " << std::endl;
            // std::cout << "Wheel velocities: " << wheel_velocities << std::endl;
            // std::cout << "H0_pinv: " << H0_pinv << std::endl;
            // std::cout << "H0: " << H0 << std::endl;
            InverseKinematicsResult result;
            result.twist = H0_pinv * wheel_velocities;
            result.residual = wheel_velocities - H0 * result.twist;
            return result;
        }
};

struct Pose2D {
    Eigen::Vector2d lin;
    double rot;
};

/**
 * @brief Update odometry using a v_b (chassis command), last pose, and time delta
 * 
 * @param last_pose Last pose
 * @param v_b Velocity command in the chassis frame
 * @param dt Time delta
*/
const Pose2D updateOdom(const Pose2D& last_pose, const Eigen::Vector3d& v_b, double dt) {
    Pose2D new_pose;
    new_pose.rot = last_pose.rot + v_b[0] * dt; // Update rotation

    // Extract linear velocities from v_b
    double v_x = v_b[1];
    double v_y = v_b[2];

    // Rotate the linear velocities into the global frame based on the last pose's rotation
    double c = cos(last_pose.rot);
    double s = sin(last_pose.rot);
    
    double global_v_x = c * v_x - s * v_y;
    double global_v_y = s * v_x + c * v_y;

    // Update global position using the rotated velocities and dt
    new_pose.lin[0] = last_pose.lin[0] + global_v_x * dt;
    new_pose.lin[1] = last_pose.lin[1] + global_v_y * dt;
    
    return new_pose;
}