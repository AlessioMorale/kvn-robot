// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Condition and decorator plugins that observe the robot: Joy buttons and axes, odometry,
// exploration, topic and TF liveness, status strings and battery voltage.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <explore_lite_msgs/msg/explore_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

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
      BT::InputPort<std::string>(
        "state", "pressed", "pressed (all), released (none) or any (Joy is fresh)"),
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
    if (any_state_) {
      return BT::NodeStatus::SUCCESS;
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
    if (state != "pressed" && state != "released" && state != "any") {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'state' must be 'pressed', 'released' or 'any', got '",
              state, "'");
    }
    want_pressed_ = state == "pressed";
    any_state_ = state == "any";
    hold_s_ = require_non_negative(*this, "hold_s");
    tracker_ = std::make_unique<ButtonTracker>(
      *get_context(config()), require_input<std::string>(*this, "topic"), require_index_list(*this, "buttons"),
      require_positive(*this, "timeout_s"));
  }

private:
  std::unique_ptr<ButtonTracker> tracker_;
  bool want_pressed_{true};
  bool any_state_{false};
  double hold_s_{0.0};
};

namespace
{

/// Value of one Joy axis, or nullopt when the message is stale or has no such axis.
std::optional<double> joy_axis(
  const Topic<sensor_msgs::msg::Joy> & topic, int axis, double timeout_s)
{
  const auto & joy = topic.latest();
  if (!topic.fresh(timeout_s) || !joy || axis < 0 ||
    static_cast<size_t>(axis) >= joy->axes.size())
  {
    return std::nullopt;
  }
  const double value = joy->axes[static_cast<size_t>(axis)];
  return std::isfinite(value) ? std::optional<double>(value) : std::nullopt;
}

}  // namespace

/// SUCCESS while Joy messages are fresh and axis `axis` is within [min, max]. Reads a
/// multi-position switch mapped to an axis, e.g. the command-mode switch.
class JoyAxis : public BT::ConditionNode, public Preparable
{
public:
  JoyAxis(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joy", "sensor_msgs/Joy topic"),
      BT::InputPort("axis", "Axis index"),
      BT::InputPort("min", "-1.0", "Lowest accepted value"),
      BT::InputPort("max", "1.0", "Highest accepted value"),
      BT::InputPort("timeout_s", "1.0", "Joy messages older than this fail the condition"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!topic_) {
      prepare();
    }
    const auto value = joy_axis(*topic_, axis_, timeout_s_);
    return value && *value >= min_ && *value <= max_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    const auto axes = require_index_list(*this, "axis");
    if (axes.size() != 1) {
      throw BT::RuntimeError("[", fullPath(), "] input port 'axis' must be a single index");
    }
    axis_ = axes.front();
    min_ = require_number(*this, "min");
    max_ = require_number(*this, "max");
    if (!(min_ <= max_)) {
      throw BT::RuntimeError("[", fullPath(), "] input ports 'min' and 'max' must be min <= max");
    }
    timeout_s_ = require_positive(*this, "timeout_s");
    topic_ = get_context(config())->subscribe<sensor_msgs::msg::Joy>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
  }

private:
  std::shared_ptr<Topic<sensor_msgs::msg::Joy>> topic_;
  int axis_{0};
  double min_{-1.0};
  double max_{1.0};
  double timeout_s_{1.0};
};

/// SUCCESS while Joy messages are fresh and any of the listed axes is deflected beyond the
/// deadzone. Stale Joy (link or receiver lost) reads as centred sticks.
class JoyAxesActive : public BT::ConditionNode, public Preparable
{
public:
  JoyAxesActive(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/joy", "sensor_msgs/Joy topic"),
      BT::InputPort("axes", "Axis indices, ';' separated"),
      BT::InputPort("deadzone", "0.1", "Deflection that still counts as centred"),
      BT::InputPort("timeout_s", "1.0", "Joy messages older than this fail the condition"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!topic_) {
      prepare();
    }
    for (const int axis : axes_) {
      const auto value = joy_axis(*topic_, axis, timeout_s_);
      if (value && std::abs(*value) > deadzone_) {
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    axes_ = require_index_list(*this, "axes");
    deadzone_ = require_non_negative(*this, "deadzone");
    timeout_s_ = require_positive(*this, "timeout_s");
    topic_ = get_context(config())->subscribe<sensor_msgs::msg::Joy>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
  }

private:
  std::shared_ptr<Topic<sensor_msgs::msg::Joy>> topic_;
  std::vector<int> axes_;
  double deadzone_{0.1};
  double timeout_s_{1.0};
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

/// SUCCESS while messages of any type arrive on `topic` at least every `timeout_s` seconds.
class TopicFresh : public BT::ConditionNode, public Preparable
{
public:
  TopicFresh(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "Topic name"),
      BT::InputPort<std::string>("type", "Message type, e.g. nav_msgs/msg/Odometry"),
      BT::InputPort("timeout_s", "1.0", "Maximum age of the last message"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!activity_) {
      prepare();
    }
    return activity_->fresh(timeout_s_) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    timeout_s_ = require_positive(*this, "timeout_s");
    activity_ = get_context(config())->activity(
      require_input<std::string>(*this, "topic"), require_input<std::string>(*this, "type"));
  }

private:
  std::shared_ptr<TopicActivity> activity_;
  double timeout_s_{1.0};
};

/// SUCCESS while child_frame can be looked up in parent_frame and the latest transform is at
/// most max_age_s old (ROS time, so it follows /clock in simulation). Static transforms (stamp 0)
/// never age. Used to tell that localization (map -> odom) is running.
class TransformFresh : public BT::ConditionNode, public Preparable
{
public:
  TransformFresh(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("parent_frame", "map", "Target frame"),
      BT::InputPort<std::string>("child_frame", "base_link", "Source frame"),
      BT::InputPort("max_age_s", "1.0", "Maximum age of the latest transform"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (parent_.empty()) {
      prepare();
    }
    const auto ctx = get_context(config());
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = ctx->tf_buffer()->lookupTransform(parent_, child_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return BT::NodeStatus::FAILURE;
    }
    const rclcpp::Time stamp(transform.header.stamp, RCL_ROS_TIME);
    if (stamp.nanoseconds() == 0) {
      return BT::NodeStatus::SUCCESS;
    }
    const double age = (ctx->node()->now() - stamp).seconds();
    return age <= max_age_s_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    parent_ = require_input<std::string>(*this, "parent_frame");
    child_ = require_input<std::string>(*this, "child_frame");
    max_age_s_ = require_positive(*this, "max_age_s");
  }

private:
  std::string parent_;
  std::string child_;
  double max_age_s_{1.0};
};

/// Matches the latest std_msgs/String on `topic` against a list of prefixes.
/// match=any: SUCCESS if it starts with one of them; match=none: SUCCESS if it starts with none.
/// Fails while nothing fresh has arrived (timeout_s 0: any age is accepted, for latched topics).
/// enabled=false makes the node always succeed, so a check can be switched off from a blackboard
/// entry (e.g. in simulation, where /robot_status is not published).
class StringTopic : public BT::ConditionNode, public Preparable
{
public:
  StringTopic(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "std_msgs/String topic"),
      BT::InputPort<std::string>("prefixes", "Values or prefixes, ';' separated"),
      BT::InputPort<std::string>("match", "any", "any or none"),
      BT::InputPort("timeout_s", "0", "Maximum message age; 0 accepts any age"),
      BT::InputPort<std::string>("qos", "volatile", "volatile or latched"),
      BT::InputPort<bool>("enabled", true, "false: always SUCCESS"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!enabled_) {
      return BT::NodeStatus::SUCCESS;
    }
    if (!topic_) {
      prepare();
    }
    const auto & latest = topic_->latest();
    if (!latest || (timeout_s_ > 0.0 && !topic_->fresh(timeout_s_))) {
      return BT::NodeStatus::FAILURE;
    }
    const bool any = std::any_of(
      prefixes_.begin(), prefixes_.end(),
      [&latest](const std::string & prefix) {return latest->data.rfind(prefix, 0) == 0;});
    return any == match_any_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    enabled_ = require_input<bool>(*this, "enabled");
    if (!enabled_) {
      return;
    }
    const auto match = require_input<std::string>(*this, "match");
    if (match != "any" && match != "none") {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'match' must be 'any' or 'none', got '", match, "'");
    }
    match_any_ = match == "any";
    timeout_s_ = require_non_negative(*this, "timeout_s");
    prefixes_.clear();
    std::stringstream stream(require_input<std::string>(*this, "prefixes"));
    std::string item;
    while (std::getline(stream, item, ';')) {
      if (!item.empty()) {
        prefixes_.push_back(item);
      }
    }
    if (prefixes_.empty()) {
      throw BT::RuntimeError("[", fullPath(), "] input port 'prefixes' must not be empty");
    }
    const auto qos = require_input<std::string>(*this, "qos");
    if (qos != "volatile" && qos != "latched") {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'qos' must be 'volatile' or 'latched', got '", qos,
              "'");
    }
    topic_ = get_context(config())->subscribe<std_msgs::msg::String>(
      require_input<std::string>(*this, "topic"),
      qos == "latched" ? rclcpp::QoS(nav2::qos::LatchedSubscriptionQoS()) :
      rclcpp::QoS(nav2::qos::StandardTopicQoS()));
  }

private:
  std::shared_ptr<Topic<std_msgs::msg::String>> topic_;
  std::vector<std::string> prefixes_;
  bool match_any_{true};
  bool enabled_{true};
  double timeout_s_{0.0};
};

/// SUCCESS while the latest fresh sensor_msgs/BatteryState voltage is at or below `voltage`, or
/// not finite (the sensor cannot read it). Stale or missing battery data fails the condition.
class BatteryVoltageBelow : public BT::ConditionNode, public Preparable
{
public:
  BatteryVoltageBelow(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/battery_state", "sensor_msgs/BatteryState topic"),
      BT::InputPort("voltage", "Threshold in volts"),
      BT::InputPort("timeout_s", "3.0", "Battery messages older than this fail the condition"),
    };
  }

  BT::NodeStatus tick() override
  {
    if (!topic_) {
      prepare();
    }
    const auto & latest = topic_->latest();
    if (!latest || !topic_->fresh(timeout_s_)) {
      return BT::NodeStatus::FAILURE;
    }
    const double v = latest->voltage;
    return !std::isfinite(v) || v <= voltage_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  void prepare() override
  {
    voltage_ = require_positive(*this, "voltage");
    timeout_s_ = require_positive(*this, "timeout_s");
    topic_ = get_context(config())->subscribe<sensor_msgs::msg::BatteryState>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
  }

private:
  std::shared_ptr<Topic<sensor_msgs::msg::BatteryState>> topic_;
  double voltage_{0.0};
  double timeout_s_{3.0};
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
  factory.registerNodeType<JoyAxis>("JoyAxis");
  factory.registerNodeType<JoyAxesActive>("JoyAxesActive");
  factory.registerNodeType<TopicFresh>("TopicFresh");
  factory.registerNodeType<TransformFresh>("TransformFresh");
  factory.registerNodeType<StringTopic>("StringTopic");
  factory.registerNodeType<BatteryVoltageBelow>("BatteryVoltageBelow");
}
