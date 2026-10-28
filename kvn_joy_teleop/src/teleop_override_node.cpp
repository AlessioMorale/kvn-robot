// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Joystick teleop with operator override, replacing teleop_twist_joy on the rover.
//
// It is the only publisher of /cmd_vel in normal operation: with the enable switch held it sends
// the sticks, otherwise it forwards the autonomous command (Nav2) when that is fresh and the
// orchestrator permits it. The operator therefore overrides any mode here, one hop before the
// controller, without depending on the orchestrator or Nav2 being alive.

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "kvn_joy_teleop/command_arbiter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace kvn_joy_teleop
{

using SteadyClock = std::chrono::steady_clock;

class TeleopOverrideNode : public rclcpp::Node
{
public:
  explicit TeleopOverrideNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("teleop_twist_joy_node", options)
  {
    // Same names and meaning as teleop_twist_joy, so its configuration carries over.
    const bool require_enable = declare_parameter<bool>("require_enable_button", true);
    enable_button_ = declare_parameter<int64_t>("enable_button", 0);
    turbo_button_ = declare_parameter<int64_t>("enable_turbo_button", -1);
    linear_axis_ = declare_parameter<int64_t>("axis_linear.x", 1);
    linear_scale_ = declare_parameter<double>("scale_linear.x", 0.5);
    linear_scale_turbo_ = declare_parameter<double>("scale_linear_turbo.x", 1.0);
    angular_axis_ = declare_parameter<int64_t>("axis_angular.yaw", 0);
    angular_scale_ = declare_parameter<double>("scale_angular.yaw", 0.5);
    angular_scale_turbo_ = declare_parameter<double>("scale_angular_turbo.yaw", 1.0);
    const bool stamped = declare_parameter<bool>("publish_stamped_twist", false);

    // Arbitration.
    const auto auto_topic = declare_parameter<std::string>("auto_cmd_topic", "cmd_vel_auto");
    const auto permit_topic =
      declare_parameter<std::string>("autonomy_topic", "/orchestrator/autonomy_enabled");
    joy_timeout_s_ = declare_parameter<double>("joy_timeout_s", 0.5);
    auto_timeout_s_ = declare_parameter<double>("auto_timeout_s", 0.5);
    permit_timeout_s_ = declare_parameter<double>("autonomy_timeout_s", 3.0);
    config_.require_joy_for_auto = declare_parameter<bool>("require_joy_for_auto", true);
    config_.require_autonomy_permit = declare_parameter<bool>("require_autonomy_permit", true);
    const double watchdog_period_s = declare_parameter<double>("watchdog_period_s", 0.05);

    if (!require_enable) {
      throw std::invalid_argument(
              "require_enable_button must be true: the enable switch is what lets the operator "
              "override autonomy");
    }
    if (stamped) {
      throw std::invalid_argument("publish_stamped_twist is not supported (skid_steer_controller "
              "takes geometry_msgs/Twist)");
    }
    if (enable_button_ < 0 || linear_axis_ < 0 || angular_axis_ < 0) {
      throw std::invalid_argument("enable_button, axis_linear.x and axis_angular.yaw must be >= 0");
    }
    if (!(joy_timeout_s_ > 0.0) || !(auto_timeout_s_ > 0.0) || !(permit_timeout_s_ > 0.0) ||
      !(watchdog_period_s > 0.0))
    {
      throw std::invalid_argument("timeouts and watchdog_period_s must be > 0");
    }

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    source_pub_ = create_publisher<std_msgs::msg::String>(
      "cmd_vel_source", rclcpp::QoS(1).reliable().transient_local());
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) {on_joy(*msg);});
    auto_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      auto_topic, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg) {on_auto(*msg);});
    permit_sub_ = create_subscription<std_msgs::msg::Bool>(
      permit_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
        permitted_ = msg->data;
        permit_stamp_ = SteadyClock::now();
        update();
      });
    watchdog_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(watchdog_period_s)),
      [this]() {update();});

    publish_source();
    std::string conditions;
    if (config_.require_autonomy_permit) {
      conditions += " while " + std::string(permit_sub_->get_topic_name()) + " is true";
    }
    if (config_.require_joy_for_auto) {
      conditions += conditions.empty() ? " while Joy is alive" : " and Joy is alive";
    }
    RCLCPP_INFO(
      get_logger(), "Teleop with override: enable button %ld, autonomous commands from %s%s",
      static_cast<long>(enable_button_), auto_sub_->get_topic_name(), conditions.c_str());
  }

private:
  static bool button(const sensor_msgs::msg::Joy & joy, int64_t index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.buttons.size() &&
           joy.buttons[static_cast<size_t>(index)] != 0;
  }

  static double axis(const sensor_msgs::msg::Joy & joy, int64_t index)
  {
    return index >= 0 && static_cast<size_t>(index) < joy.axes.size() ?
           joy.axes[static_cast<size_t>(index)] : 0.0;
  }

  static bool fresh(const std::optional<SteadyClock::time_point> & stamp, double timeout_s)
  {
    return stamp &&
           std::chrono::duration<double>(SteadyClock::now() - *stamp).count() <= timeout_s;
  }

  void on_joy(const sensor_msgs::msg::Joy & joy)
  {
    joy_ = joy;
    joy_stamp_ = SteadyClock::now();
    update();
    if (source_ == Source::kTeleop) {
      const bool turbo = button(joy, turbo_button_);
      geometry_msgs::msg::Twist twist;
      twist.linear.x = axis(joy, linear_axis_) * (turbo ? linear_scale_turbo_ : linear_scale_);
      twist.angular.z = axis(joy, angular_axis_) * (turbo ? angular_scale_turbo_ : angular_scale_);
      cmd_pub_->publish(twist);
    }
  }

  void on_auto(const geometry_msgs::msg::Twist & twist)
  {
    auto_stamp_ = SteadyClock::now();
    update();
    if (source_ == Source::kAuto) {
      cmd_pub_->publish(twist);
    }
  }

  /// Re-evaluates who drives; on a change to nobody, sends one stop command.
  void update()
  {
    ArbiterInputs inputs;
    inputs.joy_fresh = fresh(joy_stamp_, joy_timeout_s_);
    inputs.enable_pressed = joy_ && button(*joy_, enable_button_);
    inputs.auto_fresh = fresh(auto_stamp_, auto_timeout_s_);
    inputs.autonomy_permitted = permitted_ && fresh(permit_stamp_, permit_timeout_s_);
    const Source source = select_source(inputs, config_);
    if (source == source_) {
      return;
    }
    const Source previous = source_;
    source_ = source;
    // Stop on every hand-over: the new source starts from rest instead of inheriting the old
    // command until its first message.
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    publish_source();
    RCLCPP_INFO(get_logger(), "Command source: %s -> %s", to_string(previous), to_string(source));
  }

  void publish_source()
  {
    std_msgs::msg::String msg;
    msg.data = to_string(source_);
    source_pub_->publish(msg);
  }

  int64_t enable_button_{0};
  int64_t turbo_button_{-1};
  int64_t linear_axis_{1};
  int64_t angular_axis_{0};
  double linear_scale_{0.5};
  double linear_scale_turbo_{1.0};
  double angular_scale_{0.5};
  double angular_scale_turbo_{1.0};
  double joy_timeout_s_{0.5};
  double auto_timeout_s_{0.5};
  double permit_timeout_s_{3.0};
  ArbiterConfig config_;

  std::optional<sensor_msgs::msg::Joy> joy_;
  std::optional<SteadyClock::time_point> joy_stamp_;
  std::optional<SteadyClock::time_point> auto_stamp_;
  std::optional<SteadyClock::time_point> permit_stamp_;
  bool permitted_{false};
  Source source_{Source::kNone};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr source_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr auto_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr permit_sub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
};

}  // namespace kvn_joy_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<kvn_joy_teleop::TeleopOverrideNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("teleop_twist_joy_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
