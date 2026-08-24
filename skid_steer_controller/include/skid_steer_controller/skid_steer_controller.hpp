// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.
//
// Skid-steer kinematic controller for 4-wheel rovers.
// Maps geometry_msgs/Twist commands to 4 independent wheel velocities
// using the Instantaneous Center of Rotation (ICR) model.

#pragma once

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "hardware_interface/handle.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/subscription.hpp"
#include <array>

namespace skid_steer_controller
{

/**
 * @class SkidSteerController
 * @brief Maps Twist commands to 4-wheel skid-steer velocities with slip detection and compensation.
 *
 * This controller implements the Instantaneous Center of Rotation (ICR) model for a 4-wheel
 * skid-steer rover. It accepts geometry_msgs/Twist commands and outputs individual velocity
 * commands for each of the 4 wheels.
 *
 * Kinematic Model:
 * For a skid-steer rover with wheel radius r and effective track width d_eff:
 *   ω_left = (v_x - ω_z * d_eff/2) / r
 *   ω_right = (v_x + ω_z * d_eff/2) / r
 *
 * Slip Detection (Session 4):
 * Monitors slip ratio λᵢ = (ω_i * r - v_ground) / v_ground for each wheel pair.
 * When slip exceeds threshold, applies cross-coupling compensation:
 *   - If left wheels slip, clamp right wheels
 *   - If right wheels slip, clamp left wheels
 *   - Prevents one wheel suspension causing controlled slip on grounded wheels
 *
 * Effective Track Width (Session 5):
 * The kinematic model uses effective track width d_eff = track_width * chi
 * where chi is an empirical calibration factor to account for:
 *   - Wheel slip during turns
 *   - Suspension compliance and tire deformation
 *   - Actual contact geometry vs nominal measurements
 * Tuned via spot-turn maneuvers to minimize yaw error.
 *
 * Parameters:
 *   - wheel_radius (double): Radius of the wheels in meters
 *   - track_width (double): Distance between left and right wheels in meters
 *   - chi (double): Effective track width tuning factor (default 1.0, range 0.8-1.2)
 *   - cmd_vel_timeout (double): Timeout for velocity commands in seconds
 *   - slip_threshold (double): Slip ratio threshold for detection (default 0.2)
 *   - slip_clamp_factor (double): Reduction factor for clamped wheel velocities (default 0.7)
 *   - odometry_timeout (double): Timeout for odometry messages (default 0.5s)
 */
class SkidSteerController : public controller_interface::ControllerInterface
{
public:
  /// Controller init function
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /// Lifecycle callbacks
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /// Update function
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// Kinematics parameters
  double wheel_radius_;
  double track_width_;
  double chi_;                         // Effective track width tuning factor (Session 5)
  double cmd_vel_timeout_;

  /// Slip detection parameters (Session 4)
  double slip_threshold_;           // Threshold for slip ratio detection
  double slip_clamp_factor_;        // Reduction factor for clamped wheels
  double odometry_timeout_;         // Odometry message timeout

  /// Command interface handles for the 4 wheels
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    velocity_commands_;

  /// Subscription to Twist commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscription_;

  /// Subscription to Odometry for ground-truth velocity (Session 4)
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;

  /// Last received Twist command
  geometry_msgs::msg::Twist last_twist_command_;
  rclcpp::Time last_twist_timestamp_;

  /// Last received odometry (ground-truth velocity)
  double last_ground_velocity_ = 0.0;  // Linear x velocity from odometry
  rclcpp::Time last_odometry_timestamp_;

  /// Callback for Twist messages
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /// Callback for Odometry messages (Session 4)
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /// Computes wheel velocities from Twist using ICR kinematics
  void compute_wheel_velocities(
    double v_x, double omega_z, std::vector<double> & wheel_velocities);

  /// Detects and applies slip compensation (Session 4)
  void apply_slip_compensation(
    const std::vector<double> & command_velocities,
    std::vector<double> & compensated_velocities);
};

}  // namespace skid_steer_controller
