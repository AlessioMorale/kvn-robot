// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <algorithm>
#include <cmath>

// Pure assisted-teleop geometry, no ROS dependency: turns stick deflection into a Nav2 goal
// ("carrot") placed ahead of the robot, so Nav2 plans around obstacles while the operator steers.

namespace kvn_orchestrator
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct CarrotParams
{
  /// Goal distance at full forward stick.
  double lookahead_m{1.5};
  /// Closer goals are replaced by a turn on the spot (the goal checker would accept them at once).
  double min_distance_m{0.3};
  /// Heading change at full sideways stick.
  double max_heading_rad{1.57};
  /// Stick values within +-deadzone count as centred.
  double deadzone{0.1};
};

inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

/// Stick value in [-1, 1] with the deadzone removed and the rest rescaled to [-1, 1].
inline double apply_deadzone(double value, double deadzone)
{
  value = std::clamp(value, -1.0, 1.0);
  if (std::abs(value) <= deadzone || deadzone >= 1.0) {
    return 0.0;
  }
  return std::copysign((std::abs(value) - deadzone) / (1.0 - deadzone), value);
}

/// Goal for stick input \p linear (forward +) and \p angular (left +, counter-clockwise).
///
/// The goal heading is the robot heading turned by angular * max_heading_rad; the goal lies
/// linear * lookahead_m along that heading. Backward stick is ignored (the rover's controller does
/// not reverse), so a pure sideways or backward stick turns the rover on the spot.
inline Pose2D carrot_goal(
  const Pose2D & robot, double linear, double angular, const CarrotParams & params)
{
  const double forward = std::max(0.0, apply_deadzone(linear, params.deadzone));
  const double turn = apply_deadzone(angular, params.deadzone);
  const double heading = normalize_angle(robot.yaw + turn * params.max_heading_rad);
  const double distance = forward * params.lookahead_m;
  if (distance < params.min_distance_m) {
    return Pose2D{robot.x, robot.y, heading};
  }
  return Pose2D{
    robot.x + distance * std::cos(heading), robot.y + distance * std::sin(heading), heading};
}

/// True when \p next differs from \p sent enough to be worth re-planning.
inline bool goal_moved(
  const Pose2D & sent, const Pose2D & next, double min_distance_m, double min_angle_rad)
{
  return std::hypot(next.x - sent.x, next.y - sent.y) >= min_distance_m ||
         std::abs(normalize_angle(next.yaw - sent.yaw)) >= min_angle_rad;
}

}  // namespace kvn_orchestrator
