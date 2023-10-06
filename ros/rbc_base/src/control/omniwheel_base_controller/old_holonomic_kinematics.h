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
 * This is an old formulation of the chassis kinematics that I derived myself.
 * TODO: Compress the forward kinematics into a single matrix multiplication, then use the inverse of that matrix to get the inverse kinematics.
*/

const Eigen::Matrix2d jacobian = [](){
    Eigen::Matrix2d jacobian;
    jacobian << 0., 1., -1., 0.;
    return jacobian;
}();

/**
 * @brief Generic 2D state that can be used for position and velocity
*/
struct State2D {
    Eigen::Vector2d lin;
    double rot;
};

class Wheel {
    public:
        /**
         * @brief Construct a new Wheel object
         * 
         * @param x X coordinate of wheel
         * @param y Y coordinate of wheel
         * @param theta Angle of wheel (degrees, zero is along x-axis [forward])
         * @param radius Radius of wheel
         * @param handle Handle to wheel from hardware interface
        */
        Wheel(double x, double y, double theta, double radius, hardware_interface::JointHandle& handle) {
            setProperties(x, y, theta, radius);
            setHandle(handle);
        }
        /**
         * @brief Set wheel properties
         * 
         * @param x X coordinate of wheel
         * @param y Y coordinate of wheel
         * @param theta Angle of wheel (degrees, zero is along x-axis [forward])
         * @param radius Radius of wheel
        */
        void setProperties(double x, double y, double theta, double radius) {
            wheel_position_vec = Eigen::Vector2d(x, y);
            wheel_direction_vec = Eigen::Vector2d(cos(theta*0.0174533), sin(theta*0.0174533));
            wheel_radius = radius;
        }
        /**
         * @brief Set wheel handle
         * 
         * @param handle Handle to set
        */
        void setHandle(hardware_interface::JointHandle& handle) {
            wheel_handle = handle;
        }
        /**
         * @brief Updates wheel actuating velocity and return partial base delta position, rotation for odometry calculation
         * 
         * @param twist Twist to update wheel with
         * @return base_updateState2D Base update (partial delta position, rotation)
        */
        State2D update(const geometry_msgs::Twist& twist) {
            // Calculating desired wheel command:

            // Getting variables
            Eigen::Vector2d base_linear_velocity_vec = Eigen::Vector2d(twist.linear.x, twist.linear.y);
            double base_angular_velocity = twist.angular.z;
            // Deriving tangential velocity (because of base rotation)
            Eigen::Vector2d origin_tangential_vec = (jacobian * wheel_position_vec);
            Eigen::Vector2d base_tangential_velocity_vec = base_angular_velocity * origin_tangential_vec;
            // Deriving wheel angular velocity by adding tangential and linear velocity vectors, then projecting onto wheel direction; finally factoring in wheel radius
            double wheel_angular_velocity = (base_linear_velocity_vec + base_tangential_velocity_vec).dot(wheel_direction_vec) / wheel_radius * -1.; // Negative because wheel velocity in one direction causes bot to move in opposite direction
            wheel_handle.setCommand(wheel_angular_velocity);

            // Wheel odometry calculation:

            // Calculating delta position
            double pos = wheel_handle.getPosition();
            double delta_pos = pos - last_pos;
            last_pos = pos;

            // Calculating wheel update
            State2D wheel_update;
            wheel_update.lin = wheel_direction_vec * delta_pos * wheel_radius * -1.;
            wheel_update.rot = wheel_update.lin.dot(origin_tangential_vec) * -1.;

            return wheel_update;
        }
        /**
         * @brief Breaks wheel
         * 
         * Sets wheel velocity to zero
        */
        void breakWheel() {
            wheel_handle.setCommand(0);
        }
    private:
        /**
         * @brief Wheel position vector (from center of robot)
        */
        Eigen::Vector2d wheel_position_vec = Eigen::Vector2d(0., 0.);
        /**
         * @brief Wheel direction vector (direction it actuates on the floor)
        */
        Eigen::Vector2d wheel_direction_vec = Eigen::Vector2d(1., 0.);
        /**
         * @brief Wheel radius (meters)
        */
        double wheel_radius = 1.;
        /**
         * @brief Handle to wheel from hardware interface
        */
        hardware_interface::JointHandle wheel_handle;
        /**
         * @brief Last position of wheel
        */
        double last_pos = 0.;
};

class Base {
    public:
        Base() {
            odometry.header.frame_id = "odom";
            odometry.child_frame_id = "base_link";
            base_pos.lin = Eigen::Vector2d(0, 0);
            base_pos.rot = 0;
        }
        /**
         * @brief Adds a wheel to the base
         * 
         * @param wheel Wheel to add
        */
        void addWheel(const Wheel& wheel) {
            wheels.push_back(wheel);
        }
        /**
         * @brief Updates base with twist, and updates odometry
         * 
         * @param twist Twist to update base with
         * @return nav_msgs::Odometry Odometry message
        */
        const nav_msgs::Odometry& update(const geometry_msgs::Twist& twist, const ros::Time& time, const ros::Duration& period) {
            // Update wheels, get per-wheel velocity and sum to get base twist
            base_vel.lin = Eigen::Vector2d(0, 0);
            base_vel.rot = 0;
            for (auto& wheel : wheels) {
                State2D update = wheel.update(twist);
                base_vel.lin += update.lin;
                base_vel.rot += update.rot;
            }

            // Divide base_vel by dt to get velocity
            base_vel.lin /= period.toSec();
            base_vel.rot /= period.toSec();

            // Calculate trajectory
            base_pos.rot += base_vel.rot * period.toSec();
            double delta_x, delta_y;
            if (base_vel.rot == 0) {
                delta_x = base_vel.lin.x() * period.toSec();
                delta_y = base_vel.lin.y() * period.toSec();
            } else {
                delta_x = (base_vel.lin.x() * sin(base_pos.rot + base_vel.rot * period.toSec()) + base_vel.lin.y() * cos(base_pos.rot + base_vel.rot * period.toSec()) - base_vel.lin.x() * sin(base_pos.rot) - base_vel.lin.y() * cos(base_pos.rot)) / base_vel.rot;
                delta_y = (base_vel.lin.y() * sin(base_pos.rot + base_vel.rot * period.toSec()) - base_vel.lin.x() * cos(base_pos.rot + base_vel.rot * period.toSec()) - base_vel.lin.y() * sin(base_pos.rot) + base_vel.lin.x() * cos(base_pos.rot)) / base_vel.rot;
            }
            base_pos.lin.x() += delta_x;
            base_pos.lin.y() += delta_y;

            // Print base_pos
            // ROS_INFO_STREAM("Base pos: " << base_pos.lin.x() << ", " << base_pos.lin.y() << ", " << base_pos.rot);

            updateOdometryMessage(time);
            return odometry;
        }
        /**
         * @brief Breaks all wheels
        */
        void breakWheels() {
            for (auto& wheel : wheels) {
                wheel.breakWheel();
            }
        }
    private:
        /**
         * @brief Vector of wheels on the base
        */
        std::vector<Wheel> wheels;
        /**
         * @brief Base linear and angular velocity
        */
        State2D base_vel;
        /**
         * @brief Base linear and angular position
        */
        State2D base_pos;
        /**
         * @brief Odometry message
        */
        nav_msgs::Odometry odometry;
        /**
         * @brief Updates odometry message with current base position and velocity
         * 
         * @param time Time to set header stamp to
        */
        void updateOdometryMessage(const ros::Time& time) {
            // Package into odometry message
            odometry.header.stamp = time;
            
            // Set position
            odometry.pose.pose.position.x = base_pos.lin.x();
            odometry.pose.pose.position.y = base_pos.lin.y();
            odometry.pose.pose.position.z = 0;

            // Set orientation
            tf2::Quaternion q;
            q.setRPY(0, 0, base_pos.rot);
            odometry.pose.pose.orientation = tf2::toMsg(q);

            // Set velocity
            odometry.twist.twist.linear.x = base_vel.lin.x();
            odometry.twist.twist.linear.y = base_vel.lin.y();
            odometry.twist.twist.linear.z = 0;
            odometry.twist.twist.angular.x = 0;
            odometry.twist.twist.angular.y = 0;
            odometry.twist.twist.angular.z = base_vel.rot;
        }
};