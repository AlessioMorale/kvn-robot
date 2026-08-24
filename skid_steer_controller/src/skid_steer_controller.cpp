// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.

#include "skid_steer_controller/skid_steer_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace skid_steer_controller
{

controller_interface::InterfaceConfiguration
SkidSteerController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // We need velocity command interfaces for 4 wheels: front_left, front_right, back_left, back_right
  config.names.push_back("front_left_joint/velocity");
  config.names.push_back("front_right_joint/velocity");
  config.names.push_back("back_left_joint/velocity");
  config.names.push_back("back_right_joint/velocity");

  return config;
}

controller_interface::InterfaceConfiguration SkidSteerController::state_interface_configuration()
  const
{
  // We don't need state interfaces for this basic controller
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::CallbackReturn SkidSteerController::on_init()
{
  try {
    auto node = get_node();

    // Read kinematics parameters
    wheel_radius_ = node->declare_parameter<double>("wheel_radius", 0.048);
    track_width_ = node->declare_parameter<double>("track_width", 0.205);
    chi_ = node->declare_parameter<double>("chi", 1.0);  // Session 5: track width tuning
    cmd_vel_timeout_ = node->declare_parameter<double>("cmd_vel_timeout", 1.0);

    // Read slip detection parameters (Session 4)
    slip_threshold_ = node->declare_parameter<double>("slip_threshold", 0.2);
    slip_clamp_factor_ = node->declare_parameter<double>("slip_clamp_factor", 0.7);
    odometry_timeout_ = node->declare_parameter<double>("odometry_timeout", 0.5);

    RCLCPP_INFO(
      node->get_logger(),
      "SkidSteerController initialized: wheel_radius=%.3f, track_width=%.3f, chi=%.3f, "
      "slip_threshold=%.3f, slip_clamp_factor=%.3f",
      wheel_radius_, track_width_, chi_, slip_threshold_, slip_clamp_factor_);

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize SkidSteerController: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn SkidSteerController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    auto node = get_node();

    // Subscribe to Twist commands
    twist_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) { twist_callback(msg); });

    // Subscribe to Odometry for ground-truth velocity (Session 4)
    odometry_subscription_ = node->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { odometry_callback(msg); });

    RCLCPP_INFO(node->get_logger(),
      "SkidSteerController configured, subscribed to cmd_vel and odom topics");

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to configure SkidSteerController: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn SkidSteerController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    // Obtain velocity command interface handles for the 4 wheels
    velocity_commands_.clear();

    // Get command interfaces - these are provided by the framework
    // Make sure we have exactly 4 command interfaces
    if (command_interfaces_.size() != 4) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "Expected 4 command interfaces, got %zu", command_interfaces_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < 4; ++i) {
      velocity_commands_.push_back(std::ref(command_interfaces_[i]));
    }

    // Initialize timestamps
    last_twist_command_.linear.x = 0.0;
    last_twist_command_.angular.z = 0.0;
    last_twist_timestamp_ = get_node()->now();
    last_odometry_timestamp_ = get_node()->now();
    last_ground_velocity_ = 0.0;

    RCLCPP_INFO(
      get_node()->get_logger(), 
      "SkidSteerController activated with 4 velocity command interfaces and slip detection");

    return controller_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to activate SkidSteerController: %s",
      e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
}

controller_interface::CallbackReturn SkidSteerController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all motors
  for (auto & cmd_interface : velocity_commands_) {
    (void)cmd_interface.get().set_value(0.0);
  }
  velocity_commands_.clear();

  RCLCPP_INFO(get_node()->get_logger(), "SkidSteerController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

void SkidSteerController::twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_twist_command_ = *msg;
  last_twist_timestamp_ = get_node()->now();
}

void SkidSteerController::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Extract ground-truth linear velocity from odometry (Session 4)
  last_ground_velocity_ = msg->twist.twist.linear.x;
  last_odometry_timestamp_ = get_node()->now();
}

void SkidSteerController::compute_wheel_velocities(
  double v_x, double omega_z, std::vector<double> & wheel_velocities)
{
  // Skid-steer kinematics using Instantaneous Center of Rotation (ICR) model
  // For a 4-wheel skid-steer rover with paired wheels:
  //
  //   Effective track width: d_eff = track_width * chi (Session 5)
  //   where chi is an empirical tuning factor for wheel slip during turns
  //
  //   Wheel velocities (in rad/s):
  //   ω_left = (v_x - ω_z * d_eff/2) / r
  //   ω_right = (v_x + ω_z * d_eff/2) / r
  //
  // Layout:
  //   wheel_velocities[0] = front_left
  //   wheel_velocities[1] = front_right
  //   wheel_velocities[2] = back_left
  //   wheel_velocities[3] = back_right
  //
  // In a skid-steer rover, both wheels on the left move together,
  // and both wheels on the right move together.

  // Apply chi tuning factor to track_width (Session 5)
  double d_eff = track_width_ * chi_;
  double half_track_width_eff = d_eff / 2.0;
  
  double omega_left = (v_x - omega_z * half_track_width_eff) / wheel_radius_;
  double omega_right = (v_x + omega_z * half_track_width_eff) / wheel_radius_;

  wheel_velocities[0] = omega_left;   // front_left
  wheel_velocities[1] = omega_right;  // front_right
  wheel_velocities[2] = omega_left;   // back_left
  wheel_velocities[3] = omega_right;  // back_right
}

void SkidSteerController::apply_slip_compensation(
  const std::vector<double> & command_velocities,
  std::vector<double> & compensated_velocities)
{
  // Session 4: Slip detection and cross-coupling compensation
  // When one wheel pair loses traction, reduce the opposite pair to maintain control
  //
  // Wheel layout:
  //   [0] front_left,   [1] front_right
  //   [2] back_left,    [3] back_right
  //
  // Pairs: left [0,2], right [1,3]

  compensated_velocities = command_velocities;

  // Check if odometry is still fresh
  rclcpp::Duration time_since_odometry = get_node()->now() - last_odometry_timestamp_;
  if (time_since_odometry.seconds() > odometry_timeout_) {
    // No fresh odometry, skip slip detection
    return;
  }

  // Avoid division by zero - skip slip detection at very low speeds
  if (std::abs(last_ground_velocity_) < 0.01) {
    return;
  }

  // Calculate slip ratio λ = (wheel_speed - ground_speed) / ground_speed
  // Wheel speed = ω * r, where ω is in rad/s and r is wheel radius
  double ground_speed = last_ground_velocity_;

  // Left wheels slip ratio: average of front_left and back_left
  double left_wheel_speed = (command_velocities[0] + command_velocities[2]) / 2.0 * wheel_radius_;
  double left_slip_ratio = (left_wheel_speed - ground_speed) / ground_speed;

  // Right wheels slip ratio: average of front_right and back_right
  double right_wheel_speed = (command_velocities[1] + command_velocities[3]) / 2.0 * wheel_radius_;
  double right_slip_ratio = (right_wheel_speed - ground_speed) / ground_speed;

  bool left_slipping = std::abs(left_slip_ratio) > slip_threshold_;
  bool right_slipping = std::abs(right_slip_ratio) > slip_threshold_;

  if (left_slipping && !right_slipping) {
    // Left wheels are slipping, clamp right wheels to maintain control
    compensated_velocities[1] *= slip_clamp_factor_;  // front_right
    compensated_velocities[3] *= slip_clamp_factor_;  // back_right

    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Left wheel slip detected (λ=%.3f, threshold=%.3f). "
      "Cross-coupling: clamping right wheels to %.1f%% of commanded velocity.",
      left_slip_ratio, slip_threshold_, slip_clamp_factor_ * 100.0);
  } else if (right_slipping && !left_slipping) {
    // Right wheels are slipping, clamp left wheels to maintain control
    compensated_velocities[0] *= slip_clamp_factor_;  // front_left
    compensated_velocities[2] *= slip_clamp_factor_;  // back_left

    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Right wheel slip detected (λ=%.3f, threshold=%.3f). "
      "Cross-coupling: clamping left wheels to %.1f%% of commanded velocity.",
      right_slip_ratio, slip_threshold_, slip_clamp_factor_ * 100.0);
  }
}

controller_interface::return_type SkidSteerController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  // Check for command timeout
  rclcpp::Duration time_since_last_command = time - last_twist_timestamp_;
  if (time_since_last_command.seconds() > cmd_vel_timeout_) {
    // Timeout: stop motors
    for (auto & cmd_interface : velocity_commands_) {
      cmd_interface.get().set_value(0.0);
    }
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 2000,
      "Velocity command timeout: stopping motors");
    return controller_interface::return_type::OK;
  }

  // Compute wheel velocities from Twist command
  std::vector<double> wheel_velocities(4);
  compute_wheel_velocities(
    last_twist_command_.linear.x, last_twist_command_.angular.z, wheel_velocities);

  // Apply slip compensation (Session 4)
  std::vector<double> compensated_velocities(4);
  apply_slip_compensation(wheel_velocities, compensated_velocities);

  // Send compensated commands to wheels
  for (size_t i = 0; i < 4; ++i) {
    if (!velocity_commands_[i].get().set_value(compensated_velocities[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Failed to set velocity command for wheel %zu", i);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace skid_steer_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  skid_steer_controller::SkidSteerController, controller_interface::ControllerInterface)
