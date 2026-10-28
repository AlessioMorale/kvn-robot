// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Autonomy plugins: assisted teleoperation through Nav2 and the battery return-to-home check.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_behavior_tree/bt_action_node.hpp>
#include <nav2_behavior_tree/bt_utils.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include "kvn_orchestrator/assisted_teleop.hpp"
#include "kvn_orchestrator/battery_model.hpp"
#include "kvn_orchestrator/bt_utils.hpp"

namespace kvn_orchestrator
{

namespace
{

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Pose of \p child in \p parent from TF, or nullopt while it is not available.
std::optional<Pose2D> lookup_pose(
  const RosContext & ctx, const std::string & parent, const std::string & child)
{
  try {
    const auto transform = ctx.tf_buffer()->lookupTransform(parent, child, tf2::TimePointZero);
    return Pose2D{
      transform.transform.translation.x, transform.transform.translation.y,
      yaw_of(transform.transform.rotation)};
  } catch (const tf2::TransformException &) {
    return std::nullopt;
  }
}

double require_finite(const BT::TreeNode & node, const std::string & port, double min_value)
{
  const double value = require_number(node, port);
  if (!std::isfinite(value) || value < min_value) {
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "' must be finite and >= ",
            std::to_string(min_value));
  }
  return value;
}

/// ';' separated list of numbers ("3.0;3.5;4.2").
std::vector<double> require_number_list(const BT::TreeNode & node, const std::string & port)
{
  const BT::Any any = read_any(node, port);
  std::string text = any.isNumber() ? std::to_string(any.cast<double>()) : any.cast<std::string>();
  std::replace(text.begin(), text.end(), ',', ';');
  std::vector<double> values;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    try {
      size_t used = 0;
      const double value = std::stod(item, &used);
      if (std::isfinite(value) && used == item.find_last_not_of(" \t") + 1) {
        values.push_back(value);
        continue;
      }
    } catch (const std::exception &) {
    }
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "' must be a ';' separated list of "
            "numbers, got '", text, "'");
  }
  return values;
}

}  // namespace

/// Nav2 NavigateToPose whose goal follows the sticks: the goal ("carrot") is placed ahead of the
/// robot in the direction the operator steers, and moved as the robot and the sticks move. Nav2
/// plans to it, so it avoids obstacles; the operator never commands velocities directly.
///
/// Runs until halted (centred sticks, mode change, teleop override): halting cancels the goal and
/// the rover stops. Nav2 succeeding (the carrot was reached, e.g. after a turn on the spot)
/// returns SUCCESS; Nav2 aborting returns FAILURE.
class AssistedTeleopNavigate
  : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::NavigateToPose>, public Preparable
{
public:
  using Action = nav2_msgs::action::NavigateToPose;

  AssistedTeleopNavigate(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf)
  : BtActionNode<Action>(xml_tag_name, action_name, conf) {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<std::string>("joy_topic", "/joy", "sensor_msgs/Joy topic"),
        BT::InputPort("linear_axis", "1", "Axis for forward speed (forward +)"),
        BT::InputPort("angular_axis", "0", "Axis for steering (left +)"),
        BT::InputPort("joy_timeout_s", "1.0", "Older Joy messages are not used"),
        BT::InputPort("deadzone", "0.1", "Stick deflection that counts as centred"),
        BT::InputPort("lookahead_m", "1.5", "Goal distance at full forward stick"),
        BT::InputPort("min_distance_m", "0.3", "Closer goals become turns on the spot"),
        BT::InputPort("max_heading_rad", "1.57", "Goal heading change at full sideways stick"),
        BT::InputPort("update_distance_m", "0.25", "Goal shift that triggers a new goal"),
        BT::InputPort("update_angle_rad", "0.2", "Goal rotation that triggers a new goal"),
        BT::InputPort("min_update_period_s", "0.5", "Minimum time between goal updates"),
        BT::InputPort<std::string>("global_frame", "map", "Frame of the goal"),
        BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot frame"),
        BT::InputPort<std::string>(
          "behavior_tree", "", "Nav2 navigator BT to use (empty: the navigator's default)"),
      });
  }

  void prepare() override
  {
    params_.deadzone = require_finite(*this, "deadzone", 0.0);
    params_.lookahead_m = require_finite(*this, "lookahead_m", 0.0);
    params_.min_distance_m = require_finite(*this, "min_distance_m", 0.0);
    params_.max_heading_rad = require_finite(*this, "max_heading_rad", 0.0);
    if (params_.deadzone >= 1.0 || params_.lookahead_m <= 0.0) {
      throw BT::RuntimeError(
              "[", fullPath(), "] input ports 'deadzone' must be < 1 and 'lookahead_m' > 0");
    }
    update_distance_m_ = require_finite(*this, "update_distance_m", 0.0);
    update_angle_rad_ = require_finite(*this, "update_angle_rad", 0.0);
    min_update_period_s_ = require_finite(*this, "min_update_period_s", 0.0);
    joy_timeout_s_ = require_finite(*this, "joy_timeout_s", 0.0);
    if (joy_timeout_s_ <= 0.0) {
      throw BT::RuntimeError("[", fullPath(), "] input port 'joy_timeout_s' must be > 0");
    }
    const auto linear = require_index_list(*this, "linear_axis");
    const auto angular = require_index_list(*this, "angular_axis");
    if (linear.size() != 1 || angular.size() != 1) {
      throw BT::RuntimeError(
              "[", fullPath(), "] input ports 'linear_axis' and 'angular_axis' must be single "
              "indices");
    }
    linear_axis_ = static_cast<size_t>(linear.front());
    angular_axis_ = static_cast<size_t>(angular.front());
    global_frame_ = require_input<std::string>(*this, "global_frame");
    base_frame_ = require_input<std::string>(*this, "robot_base_frame");
    joy_ = get_context(config())->subscribe<sensor_msgs::msg::Joy>(
      require_input<std::string>(*this, "joy_topic"), nav2::qos::StandardTopicQoS());
  }

  void on_tick() override
  {
    if (!joy_) {
      prepare();
    }
    const auto goal = compute_goal();
    if (!goal) {
      // No TF or no fresh Joy: BtActionNode returns FAILURE without sending a goal.
      should_send_goal_ = false;
      return;
    }
    goal_.pose = to_pose_stamped(*goal);
    getInput("behavior_tree", goal_.behavior_tree);
    sent_ = *goal;
    sent_at_ = SteadyClock::now();
  }

  void on_wait_for_result(std::shared_ptr<const Action::Feedback>) override
  {
    const auto now = SteadyClock::now();
    if (std::chrono::duration<double>(now - sent_at_).count() < min_update_period_s_) {
      return;
    }
    const auto goal = compute_goal();
    if (!goal || !goal_moved(sent_, *goal, update_distance_m_, update_angle_rad_)) {
      return;
    }
    goal_.pose = to_pose_stamped(*goal);
    goal_updated_ = true;
    sent_ = *goal;
    sent_at_ = now;
  }

private:
  std::optional<Pose2D> compute_goal() const
  {
    const auto & joy = joy_->latest();
    if (!joy || !joy_->fresh(joy_timeout_s_)) {
      return std::nullopt;
    }
    const auto axis = [&joy](size_t index) {
        return index < joy->axes.size() && std::isfinite(joy->axes[index]) ?
               static_cast<double>(joy->axes[index]) : 0.0;
      };
    const auto robot = lookup_pose(*get_context(config()), global_frame_, base_frame_);
    if (!robot) {
      return std::nullopt;
    }
    return carrot_goal(*robot, axis(linear_axis_), axis(angular_axis_), params_);
  }

  geometry_msgs::msg::PoseStamped to_pose_stamped(const Pose2D & pose) const
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = global_frame_;
    msg.header.stamp = node_->now();
    msg.pose.position.x = pose.x;
    msg.pose.position.y = pose.y;
    msg.pose.orientation.z = std::sin(pose.yaw / 2.0);
    msg.pose.orientation.w = std::cos(pose.yaw / 2.0);
    return msg;
  }

  std::shared_ptr<Topic<sensor_msgs::msg::Joy>> joy_;
  CarrotParams params_;
  size_t linear_axis_{1};
  size_t angular_axis_{0};
  double joy_timeout_s_{1.0};
  double update_distance_m_{0.25};
  double update_angle_rad_{0.2};
  double min_update_period_s_{0.5};
  std::string global_frame_{"map"};
  std::string base_frame_{"base_link"};
  Pose2D sent_;
  SteadyClock::time_point sent_at_;
};

/// SUCCESS once the rover has to return home because of the battery: the voltage is at or below
/// a fixed threshold, or the energy left is no more than the trip home needs (straight-line
/// distance to `home` x path_factor x wh_per_m, plus reserve_wh). The condition must hold for
/// hold_s, so short voltage dips under load do not trigger it; once triggered it stays latched
/// until the orchestrator restarts. Stale battery data fails the condition (and resets hold_s).
class BatteryReturnRequired : public BT::ConditionNode, public Preparable
{
public:
  BatteryReturnRequired(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/battery_state", "sensor_msgs/BatteryState topic"),
      BT::InputPort("timeout_s", "3.0", "Older battery messages are not used"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("home", "Home pose"),
      BT::InputPort<std::string>("robot_base_frame", "base_link", "Robot frame"),
      BT::InputPort("critical_voltage", "Return home at or below this pack voltage"),
      BT::InputPort("cells", "4", "Cells in series"),
      BT::InputPort("capacity_wh", "Usable pack energy"),
      BT::InputPort(
        "cell_voltage_curve", "3.0;3.3;3.45;3.55;3.62;3.68;3.75;3.83;3.92;4.05;4.2",
        "Cell voltage at evenly spaced states of charge, 0 % to 100 %"),
      BT::InputPort("wh_per_m", "Energy used per metre driven"),
      BT::InputPort("path_factor", "1.5", "Driven distance / straight-line distance home"),
      BT::InputPort("reserve_wh", "Energy left on arrival"),
      BT::InputPort("hold_s", "5.0", "Time the condition must hold before it triggers"),
      BT::InputPort<bool>(
        "use_percentage", false, "Trust BatteryState.percentage when it is finite"),
      BT::OutputPort<std::string>("reason", "critical_voltage or energy"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!battery_) {
      prepare();
    }
    if (latched_) {
      return BT::NodeStatus::SUCCESS;
    }
    const auto & latest = battery_->latest();
    if (!latest || !battery_->fresh(timeout_s_)) {
      pending_since_.reset();
      return BT::NodeStatus::FAILURE;
    }
    update_distance();
    const double percentage =
      use_percentage_ ? static_cast<double>(latest->percentage) : std::nan("");
    const auto check = check_battery_return(latest->voltage, percentage, distance_m_, params_);
    if (!check.required()) {
      pending_since_.reset();
      return BT::NodeStatus::FAILURE;
    }
    const auto now = SteadyClock::now();
    if (!pending_since_) {
      pending_since_ = now;
    }
    if (std::chrono::duration<double>(now - *pending_since_).count() < hold_s_) {
      return BT::NodeStatus::FAILURE;
    }
    latched_ = true;
    const std::string reason =
      check.reason == ReturnReason::kCriticalVoltage ? "critical_voltage" : "energy";
    setOutput("reason", reason);
    RCLCPP_WARN(
      get_context(config())->logger(),
      "Battery return home required (%s): %.2f V, %.1f Wh left, %.1f Wh needed for %.1f m",
      reason.c_str(), latest->voltage, check.remaining_wh, check.required_wh, distance_m_);
    return BT::NodeStatus::SUCCESS;
  }

  void prepare() override
  {
    timeout_s_ = require_finite(*this, "timeout_s", 0.0);
    params_.critical_voltage = require_finite(*this, "critical_voltage", 0.0);
    params_.cells = static_cast<int>(require_finite(*this, "cells", 1.0));
    params_.capacity_wh = require_finite(*this, "capacity_wh", 0.0);
    params_.cell_voltage_curve = require_number_list(*this, "cell_voltage_curve");
    params_.wh_per_m = require_finite(*this, "wh_per_m", 0.0);
    params_.path_factor = require_finite(*this, "path_factor", 1.0);
    params_.reserve_wh = require_finite(*this, "reserve_wh", 0.0);
    hold_s_ = require_finite(*this, "hold_s", 0.0);
    use_percentage_ = require_input<bool>(*this, "use_percentage");
    base_frame_ = require_input<std::string>(*this, "robot_base_frame");
    if (timeout_s_ <= 0.0 || params_.capacity_wh <= 0.0) {
      throw BT::RuntimeError(
              "[", fullPath(), "] input ports 'timeout_s' and 'capacity_wh' must be > 0");
    }
    const auto & curve = params_.cell_voltage_curve;
    if (curve.size() < 2 || !std::is_sorted(curve.begin(), curve.end())) {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'cell_voltage_curve' needs at least 2 increasing "
              "voltages");
    }
    battery_ = get_context(config())->subscribe<sensor_msgs::msg::BatteryState>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
  }

private:
  /// Straight-line distance to home; the last known value is kept while TF or home is missing.
  void update_distance()
  {
    geometry_msgs::msg::PoseStamped home;
    if (!getInput("home", home)) {
      return;
    }
    const auto robot = lookup_pose(*get_context(config()), home.header.frame_id, base_frame_);
    if (robot) {
      distance_m_ = std::hypot(home.pose.position.x - robot->x, home.pose.position.y - robot->y);
    }
  }

  std::shared_ptr<Topic<sensor_msgs::msg::BatteryState>> battery_;
  BatteryReturnParams params_;
  std::string base_frame_{"base_link"};
  double timeout_s_{3.0};
  double hold_s_{5.0};
  bool use_percentage_{false};
  double distance_m_{0.0};
  std::optional<SteadyClock::time_point> pending_since_;
  bool latched_{false};
};

}  // namespace kvn_orchestrator

BT_REGISTER_NODES(factory)
{
  using namespace kvn_orchestrator;  // NOLINT
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config) {
      return std::make_unique<AssistedTeleopNavigate>(name, "navigate_to_pose", config);
    };
  factory.registerBuilder<AssistedTeleopNavigate>("AssistedTeleopNavigate", builder);
  factory.registerNodeType<BatteryReturnRequired>("BatteryReturnRequired");
}
