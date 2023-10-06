#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <Eigen/Dense>
#include <hardware_interface/joint_command_interface.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include "./holonomic_kinematics.h"

// TODO: Implement cmd_vel and odometry, but remember to use realtime_tools

class OmniwheelBaseController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    private:
        hardware_interface::JointHandle vel_handle_1;
        hardware_interface::JointHandle vel_handle_2;
        hardware_interface::JointHandle vel_handle_3;
        // hardware_interface::JointHandle vel_handle_4;
        realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
        HolonomicKinematics base;
        ros::Subscriber cmd_vel_sub;
        realtime_tools::RealtimePublisher<nav_msgs::Odometry> odom_pub;
        tf2_ros::TransformBroadcaster tf_broadcaster;
        Pose2D pose;
        // double last_wheel_positions[4] = {0., 0., 0., 0.};
        double last_wheel_positions[3] = {0., 0., 0.};
        // Placeholders for joint names
        std::string joint_name_1;
        std::string joint_name_2;
        std::string joint_name_3;
        // std::string joint_name_4;
    public:
        OmniwheelBaseController() {
        }
        bool init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& nh) {
            // Get joint names
            nh.param<std::string>("joint_name_1", joint_name_1, "wheel_1_joint");
            nh.param<std::string>("joint_name_2", joint_name_2, "wheel_2_joint");
            nh.param<std::string>("joint_name_3", joint_name_3, "wheel_3_joint");
            // nh.param<std::string>("joint_name_4", joint_name_4, "wheel_4_joint");

            // Log all joint names
            // ROS_INFO_STREAM("Joint names: " << joint_name_1 << ", " << joint_name_2 << ", " << joint_name_3 << ", " << joint_name_4);
            ROS_INFO_STREAM("Joint names: " << joint_name_1 << ", " << joint_name_2 << ", " << joint_name_3);
                       
            ROS_INFO_STREAM("Initializing OmniwheelBaseController");
            vel_handle_1 = hw->getHandle(joint_name_1);
            vel_handle_2 = hw->getHandle(joint_name_2);
            vel_handle_3 = hw->getHandle(joint_name_3);
            // vel_handle_4 = hw->getHandle(joint_name_4);
            ROS_INFO_STREAM("Got handle for joints");

            // Configure base
            // base.addOmniwheel(0.1575, -0.1575, 45 * M_PI / 180, 0.05);
            // base.addOmniwheel(0.1575, 0.1575, 135 * M_PI / 180, 0.05);
            // base.addOmniwheel(-0.1575, 0.1575, 225 * M_PI / 180, 0.05);
            // base.addOmniwheel(-0.1575, -0.1575, 315 * M_PI / 180, 0.05);

            base.addOmniwheel(0.20, 0, 270 * M_PI / 180, 0.05);
            base.addOmniwheel(-0.1, -0.1732050808, 150 * M_PI / 180, 0.05);
            base.addOmniwheel(-0.1, 0.1732050808, 30 * M_PI / 180, 0.05);

            // Subscribe to cmd_vel
            cmd_vel_sub = nh.subscribe("/cmd_vel", 1, &OmniwheelBaseController::cmd_vel_callback, this);

            // Setup odometry publisher
            odom_pub.init(nh, "/odom", 1);

            return true;
        }
        void cmd_vel_callback(const geometry_msgs::Twist& cmd_vel) {
            cmd_vel_buffer_.writeFromNonRT(cmd_vel);
        }
        // Get wheel velocities using delta wheel positions and delta time
        Eigen::Vector3d getWheelVelocities(double dt) {
            // Get wheel positions
            double wheel_positions[4];
            wheel_positions[0] = vel_handle_1.getPosition();
            wheel_positions[1] = vel_handle_2.getPosition();
            wheel_positions[2] = vel_handle_3.getPosition();
            // wheel_positions[3] = vel_handle_4.getPosition();
            
            // Calculate wheel velocities
            Eigen::Vector3d wheel_velocities;
            // for (int i = 0; i < 4; i++) {
            for (int i = 0; i < 3; i++) {
                wheel_velocities[i] = (wheel_positions[i] - last_wheel_positions[i]) / dt;
            }

            // Update last wheel positions
            // for (int i = 0; i < 4; i++) {
            for (int i = 0; i < 3; i++) {
                last_wheel_positions[i] = wheel_positions[i];
            }
            return wheel_velocities;
        }
        void setWheelVelocities(const Eigen::Vector3d& wheel_velocities) {
            vel_handle_1.setCommand(wheel_velocities[0]);
            vel_handle_2.setCommand(wheel_velocities[1]);
            vel_handle_3.setCommand(wheel_velocities[2]);
            // vel_handle_4.setCommand(wheel_velocities[3]);
        }
        void update(const ros::Time& time, const ros::Duration& period) {
            if (cmd_vel_buffer_.readFromRT()) {
                // Read cmd_vel
                geometry_msgs::Twist cmd_vel = *(cmd_vel_buffer_.readFromRT());
                // Get wheel velocities
                double dt = period.toSec();
                Eigen::Vector3d wheel_velocities = getWheelVelocities(dt);
                // Calculate inverse kinematics
                InverseKinematicsResult measured_twist_result = base.inverseVb(wheel_velocities);

                // Calculate forward kinematics
                Eigen::Vector3d cmd_twist; // [phi, x, y]
                cmd_twist[0] = cmd_vel.angular.z;
                cmd_twist[1] = cmd_vel.linear.x;
                cmd_twist[2] = cmd_vel.linear.y;
                Eigen::Vector3d wheel_velocities_cmd = base.forwardVb(cmd_twist);
                setWheelVelocities(wheel_velocities_cmd);
                // Calculate odometry
                pose = updateOdom(pose, measured_twist_result.twist, dt);
                // Publish odometry
                if (odom_pub.trylock()) {
                    // Craft odometry message
                    nav_msgs::Odometry odom;
                    odom.header.stamp = time;
                    odom.header.frame_id = "odom";
                    odom.child_frame_id = "base_link";
                    odom.pose.pose.position.x = pose.lin[0];
                    odom.pose.pose.position.y = pose.lin[1];
                    odom.pose.pose.position.z = 0;
                    tf2::Quaternion q;
                    q.setRPY(0, 0, pose.rot);
                    odom.pose.pose.orientation = tf2::toMsg(q);                    
                    odom.twist.twist.linear.x = measured_twist_result.twist[1];
                    odom.twist.twist.linear.y = measured_twist_result.twist[2];
                    odom.twist.twist.linear.z = 0;
                    odom.twist.twist.angular.x = 0;
                    odom.twist.twist.angular.y = 0;
                    odom.twist.twist.angular.z = measured_twist_result.twist[0];
                    
                    // Publish odometry
                    odom_pub.msg_ = odom;
                    odom_pub.unlockAndPublish();

                    // Publish odom tf
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
                    // ROS_WARN_STREAM("Failed to publish odom");
                }
            }
        }
        void starting(const ros::Time& time) {
            ROS_INFO_STREAM("Starting OmniwheelBaseController");
        }
        void stopping(const ros::Time& time) {
            ROS_INFO_STREAM("Stopping OmniwheelBaseController");
            // Brake
            vel_handle_1.setCommand(0);
            vel_handle_2.setCommand(0);
            vel_handle_3.setCommand(0);
            // vel_handle_4.setCommand(0);
        }
};

PLUGINLIB_EXPORT_CLASS(OmniwheelBaseController, controller_interface::ControllerBase);