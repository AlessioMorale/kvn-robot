// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

namespace kvn_odometry
{

/**
 * @class OdometryNode
 * @brief Computes odometry from wheel encoders and IMU data.
 *
 * This node:
 * 1. Reads wheel velocity feedback from joint_states
 * 2. Reads angular velocity (yaw rate) from IMU
 * 3. Performs dead-reckoning to estimate robot pose
 * 4. Publishes odometry on /odom topic
 * 5. Broadcasts TF frame (odom -> base_link)
 *
 * Configuration Parameters:
 *   - wheel_radius (double): Wheel radius in meters
 *   - track_width (double): Distance between left/right wheels
 *   - base_frame_id (string): Name of robot base frame
 *   - odom_frame_id (string): Name of odometry frame
 *   - update_rate (double): Odometry update rate in Hz
 *   - use_imu_yaw (bool): Use IMU for yaw rate instead of wheel-based calculation
 */
class OdometryNode : public rclcpp::Node
{
public:
  explicit OdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Parameters
  double wheel_radius_;
  double track_width_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  double update_rate_;
  bool use_imu_yaw_;

  // Subscriptions
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;

  // Publishers
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  // TF broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Pose state
  struct PoseState
  {
    double x = 0.0;      // Position X (meters)
    double y = 0.0;      // Position Y (meters)
    double theta = 0.0;  // Orientation yaw (radians)
    rclcpp::Time timestamp;
  } pose_state_;

  // Velocity state
  struct VelocityState
  {
    double v_x = 0.0;      // Linear velocity X (m/s)
    double omega_z = 0.0;  // Angular velocity Z (rad/s)
    rclcpp::Time timestamp;
  } velocity_state_;

  // Joint state tracking
  std::map<std::string, double> joint_velocities_;
  rclcpp::Time last_joint_state_time_;

  // IMU data
  double imu_yaw_rate_ = 0.0;
  rclcpp::Time last_imu_time_;

  // Callbacks
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg);
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);

  // Odometry computation
  void compute_odometry();
  void publish_odometry_and_tf();

  // Utility functions
  double normalize_angle(double angle);
  std::pair<double, double> calculate_wheel_velocities();
};

}  // namespace kvn_odometry
