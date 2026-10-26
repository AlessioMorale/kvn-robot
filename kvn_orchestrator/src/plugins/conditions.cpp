// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Condition and decorator plugins that observe the robot: Joy buttons, odometry, exploration.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "kvn_orchestrator/bt_utils.hpp"
#include "kvn_orchestrator/button_tracker.hpp"

namespace kvn_orchestrator
{

namespace
{

double require_positive(const BT::TreeNode & node, const std::string & port)
{
  const double value = require_number(node, port);
  if (!std::isfinite(value) || value <= 0.0) {
    throw BT::RuntimeError("[", node.fullPath(), "] input port '", port, "' must be finite and > 0");
  }
  return value;
}

double require_non_negative(const BT::TreeNode & node, const std::string & port)
{
  const double value = require_number(node, port);
  if (!std::isfinite(value) || value < 0.0) {
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "' must be finite and >= 0");
  }
  return value;
}

}  // namespace

/// SUCCESS while Joy messages are fresh and the listed buttons are all pressed (state=pressed,
/// optionally for hold_s seconds) or all released (state=released).
class JoyButton : public BT::ConditionNode, public Preparable
{
public:
  JoyButton(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joy", "sensor_msgs/Joy topic"),
      BT::InputPort("buttons", "Button indices, ';' separated"),
      BT::InputPort<std::string>("state", "pressed", "pressed (all) or released (none)"),
      BT::InputPort("hold_s", "0.0", "Minimum time the buttons must stay pressed"),
      BT::InputPort("timeout_s", "1.0", "Joy messages older than this fail the condition"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!tracker_) {
      prepare();
    }
    if (!tracker_->fresh()) {
      return BT::NodeStatus::FAILURE;
    }
    if (!want_pressed_) {
      return tracker_->none_pressed() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return tracker_->all_pressed() && tracker_->held_s() >= hold_s_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    const auto state = require_input<std::string>(*this, "state");
    if (state != "pressed" && state != "released") {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'state' must be 'pressed' or 'released', got '",
              state, "'");
    }
    want_pressed_ = state == "pressed";
    hold_s_ = require_non_negative(*this, "hold_s");
    tracker_ = std::make_unique<ButtonTracker>(
      *get_context(config()), require_input<std::string>(*this, "topic"), require_index_list(*this, "buttons"),
      require_positive(*this, "timeout_s"));
  }

private:
  std::unique_ptr<ButtonTracker> tracker_;
  bool want_pressed_{true};
  double hold_s_{0.0};
};

/// Runs its child, then refuses to run it again until the buttons have been released. The buttons
/// count as "pressed" when all of them are down; any Joy message with one of them up releases the
/// latch. Used so that one button hold triggers one action.
class OncePerPress : public BT::DecoratorNode, public Preparable
{
public:
  OncePerPress(const std::string & name, const BT::NodeConfig & config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joy", "sensor_msgs/Joy topic"),
      BT::InputPort("buttons", "Button indices, ';' separated"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!tracker_) {
      prepare();
    }
    if (latched_) {
      if (tracker_->take_release() || !tracker_->all_pressed()) {
        latched_ = false;
      } else {
        return BT::NodeStatus::FAILURE;
      }
    }
    setStatus(BT::NodeStatus::RUNNING);
    const auto status = child_node_->executeTick();
    switch (status) {
      case BT::NodeStatus::SUCCESS:
      case BT::NodeStatus::FAILURE:
        latch();
        resetChild();
        return status;
      case BT::NodeStatus::SKIPPED:
        resetChild();
        return BT::NodeStatus::SKIPPED;
      default:
        return status;
    }
  }

  void prepare() override
  {
    tracker_ = std::make_unique<ButtonTracker>(
      *get_context(config()), require_input<std::string>(*this, "topic"),
      require_index_list(*this, "buttons"), 1.0);
  }

  void halt() override
  {
    if (child_node_ && child_node_->status() == BT::NodeStatus::RUNNING && tracker_) {
      latch();
    }
    BT::DecoratorNode::halt();
  }

private:
  void latch()
  {
    latched_ = true;
    // A release that already happened (e.g. the child was halted because of it) must still count.
    tracker_->take_release();
    if (!tracker_->all_pressed()) {
      latched_ = false;
    }
  }

  std::unique_ptr<ButtonTracker> tracker_;
  bool latched_{false};
};

/// SUCCESS once odometry has been fresh and below the speed limits for hold_s seconds.
class RobotStationary : public BT::ConditionNode, public Preparable
{
public:
  RobotStationary(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/odom_ekf", "nav_msgs/Odometry topic"),
      BT::InputPort("timeout_s", "0.5", "Odometry older than this is not trusted"),
      BT::InputPort("hold_s", "1.0", "Time the robot must stay below the limits"),
      BT::InputPort("max_linear_mps", "0.03", "Linear speed limit"),
      BT::InputPort("max_angular_rps", "0.05", "Angular speed limit"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!topic_) {
      prepare();
    }
    const auto now = SteadyClock::now();
    return topic_->fresh(timeout_s_) && stationary_since_ &&
           std::chrono::duration<double>(now - *stationary_since_).count() >= hold_s_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    timeout_s_ = require_positive(*this, "timeout_s");
    hold_s_ = require_positive(*this, "hold_s");
    max_linear_ = require_non_negative(*this, "max_linear_mps");
    max_angular_ = require_non_negative(*this, "max_angular_rps");
    topic_ = get_context(config())->subscribe<nav_msgs::msg::Odometry>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
    listener_ = topic_->listen(
      [this](const nav_msgs::msg::Odometry & msg) {on_odom(msg);});
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto now = SteadyClock::now();
    if (!last_odom_ || std::chrono::duration<double>(now - *last_odom_).count() > timeout_s_) {
      stationary_since_.reset();
    }
    last_odom_ = now;
    const auto & linear = msg.twist.twist.linear;
    const auto & angular = msg.twist.twist.angular;
    if (std::hypot(linear.x, linear.y) <= max_linear_ && std::abs(angular.z) <= max_angular_) {
      if (!stationary_since_) {
        stationary_since_ = now;
      }
    } else {
      stationary_since_.reset();
    }
  }

  std::shared_ptr<Topic<nav_msgs::msg::Odometry>> topic_;
  std::shared_ptr<Topic<nav_msgs::msg::Odometry>::Listener> listener_;
  std::optional<SteadyClock::time_point> last_odom_;
  std::optional<SteadyClock::time_point> stationary_since_;
  double timeout_s_{0.5};
  double hold_s_{1.0};
  double max_linear_{0.03};
  double max_angular_{0.05};
};

/// SUCCESS while explore_lite reports that exploration is started or in progress.
class IsExploring : public BT::ConditionNode, public Preparable
{
public:
  IsExploring(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "topic", "/explore/status", "explore_lite_msgs/ExploreStatus topic"),
    };
  }

  BT::NodeStatus tick() override
  {
    using explore_lite_msgs::msg::ExploreStatus;
    if (!topic_) {
      prepare();
    }
    const auto & latest = topic_->latest();
    return latest && (latest->status == ExploreStatus::EXPLORATION_STARTED ||
           latest->status == ExploreStatus::EXPLORATION_IN_PROGRESS) ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    // explore_lite publishes status on change only, so the latched sample must be received early.
    topic_ = get_context(config())->subscribe<explore_lite_msgs::msg::ExploreStatus>(
      require_input<std::string>(*this, "topic"), nav2::qos::LatchedSubscriptionQoS());
  }

private:
  std::shared_ptr<Topic<explore_lite_msgs::msg::ExploreStatus>> topic_;
};

/// SUCCESS if the named blackboard entry has been set, e.g. the captured home pose.
class EntryExists : public BT::ConditionNode
{
public:
  EntryExists(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("key", "Blackboard entry name (without braces)")};
  }

  BT::NodeStatus tick() override
  {
    const auto key = require_input<std::string>(*this, "key");
    const auto entry = config().blackboard->getEntry(key);
    return entry && !entry->value.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace kvn_orchestrator

BT_REGISTER_NODES(factory)
{
  using namespace kvn_orchestrator;  // NOLINT
  factory.registerNodeType<JoyButton>("JoyButton");
  factory.registerNodeType<OncePerPress>("OncePerPress");
  factory.registerNodeType<RobotStationary>("RobotStationary");
  factory.registerNodeType<IsExploring>("IsExploring");
  factory.registerNodeType<EntryExists>("EntryExists");
}
