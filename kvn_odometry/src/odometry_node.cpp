// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.

#include "kvn_odometry/odometry_node.hpp"

#include <cmath>
#include <map>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace kvn_odometry
{

OdometryNode::OdometryNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("odometry_node", options)
{
  // Declare parameters with defaults
  wheel_radius_ = declare_parameter<double>("wheel_radius", 0.048);
  track_width_ = declare_parameter<double>("track_width", 0.205);
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  update_rate_ = declare_parameter<double>("update_rate", 50.0);
  use_imu_yaw_ = declare_parameter<bool>("use_imu_yaw", true);

  RCLCPP_INFO(get_logger(),
    "Odometry node initialized: wheel_radius=%.3f m, track_width=%.3f m, use_imu_yaw=%s",
    wheel_radius_, track_width_, use_imu_yaw_ ? "true" : "false");

  // Initialize pose with current time
  pose_state_.timestamp = now();
  velocity_state_.timestamp = now();
  last_joint_state_time_ = now();
  last_imu_time_ = now();

  // Create subscriptions
  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::SystemDefaultsQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });

  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

  // Create publisher for odometry
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // Create TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  RCLCPP_INFO(get_logger(), "Odometry node ready. Subscribing to /joint_states and /imu");
}

void OdometryNode::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Store joint velocities by name
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (i < msg->velocity.size()) {
      joint_velocities_[msg->name[i]] = msg->velocity[i];
    }
  }
  last_joint_state_time_ = rclcpp::Time(msg->header.stamp);

  // Compute odometry with new wheel data
  compute_odometry();
}

void OdometryNode::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  // Extract yaw rate (angular velocity around Z axis)
  imu_yaw_rate_ = msg->angular_velocity.z;
  last_imu_time_ = rclcpp::Time(msg->header.stamp);
}

std::pair<double, double> OdometryNode::calculate_wheel_velocities()
{
  // Average left and right wheel velocities
  // Joint naming: front_left_joint, back_left_joint, front_right_joint, back_right_joint

  double left_vel = 0.0;
  double right_vel = 0.0;
  int left_count = 0;
  int right_count = 0;

  // Find and average left wheels
  if (joint_velocities_.count("front_left_joint")) {
    left_vel += joint_velocities_["front_left_joint"];
    left_count++;
  }
  if (joint_velocities_.count("back_left_joint")) {
    left_vel += joint_velocities_["back_left_joint"];
    left_count++;
  }

  // Find and average right wheels
  if (joint_velocities_.count("front_right_joint")) {
    right_vel += joint_velocities_["front_right_joint"];
    right_count++;
  }
  if (joint_velocities_.count("back_right_joint")) {
    right_vel += joint_velocities_["back_right_joint"];
    right_count++;
  }

  if (left_count > 0) {
    left_vel /= left_count;
  }
  if (right_count > 0) {
    right_vel /= right_count;
  }

  // Convert from angular velocity (rad/s) to linear velocity (m/s)
  left_vel *= wheel_radius_;
  right_vel *= wheel_radius_;

  return {left_vel, right_vel};
}

double OdometryNode::normalize_angle(double angle)
{
  // Normalize angle to [-pi, pi]
  while (angle > M_PI) {
    angle -= 2 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

void OdometryNode::compute_odometry()
{
  // Time since last update
  rclcpp::Time current_time = last_joint_state_time_;
  double dt = (current_time - pose_state_.timestamp).seconds();

  if (dt < 0.001) {
    // Skip if dt is too small
    return;
  }

  // Get wheel velocities (linear m/s)
  auto [v_left, v_right] = calculate_wheel_velocities();

  // Calculate body-frame velocities from wheel velocities
  double v_x = (v_left + v_right) / 2.0;  // Linear velocity (m/s)

  // Angular velocity: use IMU if available, otherwise calculate from wheels
  double omega_z;
  if (use_imu_yaw_ && (current_time - last_imu_time_).seconds() < 0.1) {
    // IMU data is recent enough, use it
    omega_z = imu_yaw_rate_;
  } else {
    // Calculate from wheel velocities
    //   omega_z = (v_right - v_left) / track_width
    omega_z = (v_right - v_left) / track_width_;
  }

  // Update velocity state
  velocity_state_.v_x = v_x;
  velocity_state_.omega_z = omega_z;
  velocity_state_.timestamp = current_time;

  // Simple integrator for pose (dead reckoning)
  // x(k+1) = x(k) + v_x * cos(theta) * dt
  // y(k+1) = y(k) + v_x * sin(theta) * dt
  // theta(k+1) = theta(k) + omega_z * dt

  if (v_x != 0.0 || omega_z != 0.0) {
    // Update position
    pose_state_.x += v_x * std::cos(pose_state_.theta) * dt;
    pose_state_.y += v_x * std::sin(pose_state_.theta) * dt;

    // Update orientation
    pose_state_.theta += omega_z * dt;
    pose_state_.theta = normalize_angle(pose_state_.theta);
  }

  pose_state_.timestamp = current_time;

  // Publish odometry
  publish_odometry_and_tf();
}

void OdometryNode::publish_odometry_and_tf()
{
  auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

  // Header
  odom_msg->header.stamp = pose_state_.timestamp;
  odom_msg->header.frame_id = odom_frame_id_;
  odom_msg->child_frame_id = base_frame_id_;

  // Position
  odom_msg->pose.pose.position.x = pose_state_.x;
  odom_msg->pose.pose.position.y = pose_state_.y;
  odom_msg->pose.pose.position.z = 0.0;

  // Orientation (convert yaw to quaternion)
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_state_.theta);
  odom_msg->pose.pose.orientation.x = q.x();
  odom_msg->pose.pose.orientation.y = q.y();
  odom_msg->pose.pose.orientation.z = q.z();
  odom_msg->pose.pose.orientation.w = q.w();

  // Pose covariance (diagonal matrix for rough estimation)
  // Set to reasonable values: position and orientation uncertainty
  for (auto & cov : odom_msg->pose.covariance) {
    cov = 0.0;
  }
  odom_msg->pose.covariance[0] = 0.01;   // x
  odom_msg->pose.covariance[7] = 0.01;   // y
  odom_msg->pose.covariance[35] = 0.02;  // theta

  // Twist (velocity)
  odom_msg->twist.twist.linear.x = velocity_state_.v_x;
  odom_msg->twist.twist.linear.y = 0.0;
  odom_msg->twist.twist.linear.z = 0.0;
  odom_msg->twist.twist.angular.x = 0.0;
  odom_msg->twist.twist.angular.y = 0.0;
  odom_msg->twist.twist.angular.z = velocity_state_.omega_z;

  // Twist covariance
  for (auto & cov : odom_msg->twist.covariance) {
    cov = 0.0;
  }
  odom_msg->twist.covariance[0] = 0.01;   // v_x
  odom_msg->twist.covariance[35] = 0.05;  // omega_z

  // Publish
  odom_publisher_->publish(std::move(odom_msg));

  // Broadcast TF: odom -> base_link
  auto transform = geometry_msgs::msg::TransformStamped();
  transform.header.stamp = pose_state_.timestamp;
  transform.header.frame_id = odom_frame_id_;
  transform.child_frame_id = base_frame_id_;

  transform.transform.translation.x = pose_state_.x;
  transform.transform.translation.y = pose_state_.y;
  transform.transform.translation.z = 0.0;

  tf2::Quaternion q_tf;
  q_tf.setRPY(0.0, 0.0, pose_state_.theta);
  transform.transform.rotation.x = q_tf.x();
  transform.transform.rotation.y = q_tf.y();
  transform.transform.rotation.z = q_tf.z();
  transform.transform.rotation.w = q_tf.w();

  tf_broadcaster_->sendTransform(transform);

  RCLCPP_DEBUG(get_logger(),
    "Odometry: x=%.3f, y=%.3f, theta=%.3f, v_x=%.3f, omega_z=%.3f",
    pose_state_.x, pose_state_.y, pose_state_.theta, velocity_state_.v_x,
    velocity_state_.omega_z);
}

}  // namespace kvn_odometry

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kvn_odometry::OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
