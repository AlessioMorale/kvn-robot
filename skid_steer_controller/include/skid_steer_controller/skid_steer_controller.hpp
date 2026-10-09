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
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/subscription.hpp"
#include <array>
#include <atomic>
#include <vector>

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
 *   - slip_compensation_enabled (bool): Enable slip cross-coupling (default true; disable
 *     during kinematic calibration so it does not distort the commanded wheel speeds)
 *
 * All parameters can be changed at runtime (ros2 param set); values are validated and
 * read lock-free from the realtime update loop.
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
  /// Kinematics parameters (atomic: written by the parameter callback, read in update())
  std::atomic<double> wheel_radius_{0.048};
  std::atomic<double> track_width_{0.205};
  std::atomic<double> chi_{1.0};                // Effective track width tuning factor (Session 5)
  std::atomic<double> cmd_vel_timeout_{1.0};

  /// Acceleration limits
  std::atomic<double> max_linear_acceleration_{2.0};
  std::atomic<double> max_angular_acceleration_{4.0};
  double current_v_x_{0.0};
  double current_omega_z_{0.0};

  /// Slip detection parameters (Session 4)
  std::atomic<double> slip_threshold_{0.2};     // Threshold for slip ratio detection
  std::atomic<double> slip_clamp_factor_{0.7};  // Reduction factor for clamped wheels
  std::atomic<double> odometry_timeout_{0.5};   // Odometry message timeout
  std::atomic<bool> slip_compensation_enabled_{true};

  /// Runtime parameter updates
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult on_parameters_set(
    const std::vector<rclcpp::Parameter> & parameters);

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
