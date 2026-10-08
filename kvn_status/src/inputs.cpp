// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_status/inputs.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace kvn_status
{

namespace
{

std::string to_upper(std::string s)
{
  std::transform(
    s.begin(), s.end(), s.begin(),
    [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
  return s;
}

/**
 * `diagnostics`: statuses on /diagnostics whose name contains `name_contains`.
 * ERROR/STALE → FAULT, WARN → WARN, both with `code`. Staleness is measured
 * from the last array that contained a matching status.
 */
class DiagnosticsInput : public StatusInput
{
public:
  DiagnosticsInput(const InputContext & context, const InputCommonConfig & config)
  : StatusInput(context, config)
  {
    name_contains_ =
      context.node->declare_parameter<std::string>(context.name + ".name_contains", context.name);
    if (name_contains_.empty())
    {
      throw std::invalid_argument(context.name + ".name_contains must not be empty");
    }
    sub_ = context.node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      config_.topic, rclcpp::QoS(10),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) { on_msg(*msg); });
  }

private:
  void on_msg(const diagnostic_msgs::msg::DiagnosticArray & msg)
  {
    using diagnostic_msgs::msg::DiagnosticStatus;
    bool matched = false;
    Severity worst = Severity::kOk;
    for (const auto & status : msg.status)
    {
      if (status.name.find(name_contains_) == std::string::npos)
      {
        continue;
      }
      matched = true;
      Severity severity = Severity::kOk;
      if (status.level == DiagnosticStatus::WARN)
      {
        severity = Severity::kWarn;
      }
      else if (status.level != DiagnosticStatus::OK)
      {
        // ERROR, STALE and any unknown level
        severity = Severity::kFault;
      }
      worst = std::max(worst, severity);
    }
    if (matched)
    {
      report(worst, worst == Severity::kOk ? std::string{} : config_.code);
    }
  }

  std::string name_contains_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sub_;
};

/**
 * `battery`: sensor_msgs/BatteryState. voltage <= fault_voltage → FAULT,
 * <= warn_voltage → WARN. A non-finite voltage (sensor unreadable) → FAULT.
 */
class BatteryInput : public StatusInput
{
public:
  BatteryInput(const InputContext & context, const InputCommonConfig & config)
  : StatusInput(context, config)
  {
    const std::string p = context.name + ".";
    warn_voltage_ = context.node->declare_parameter<double>(p + "warn_voltage", 13.6);
    fault_voltage_ = context.node->declare_parameter<double>(p + "fault_voltage", 12.4);
    if (fault_voltage_ > warn_voltage_)
    {
      throw std::invalid_argument(p + "fault_voltage must be <= warn_voltage");
    }
    sub_ = context.node->create_subscription<sensor_msgs::msg::BatteryState>(
      config_.topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::BatteryState::ConstSharedPtr msg) { on_msg(*msg); });
  }

private:
  void on_msg(const sensor_msgs::msg::BatteryState & msg)
  {
    const double v = msg.voltage;
    Severity severity = Severity::kOk;
    if (!std::isfinite(v) || v <= fault_voltage_)
    {
      severity = Severity::kFault;
    }
    else if (v <= warn_voltage_)
    {
      severity = Severity::kWarn;
    }
    report(severity, severity == Severity::kOk ? std::string{} : config_.code);
  }

  double warn_voltage_{0.0};
  double fault_voltage_{0.0};
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
};

/**
 * `joy`: sensor_msgs/Joy. `buttons[enable_button]` high → DRV, otherwise RDY.
 * Only staleness produces a condition (FAULT with the stale code).
 */
class JoyInput : public StatusInput
{
public:
  JoyInput(const InputContext & context, const InputCommonConfig & config)
  : StatusInput(context, config)
  {
    const int64_t button =
      context.node->declare_parameter<int64_t>(context.name + ".enable_button", 0);
    if (button < 0)
    {
      throw std::invalid_argument(context.name + ".enable_button must be >= 0");
    }
    enable_button_ = static_cast<std::size_t>(button);
    sub_ = context.node->create_subscription<sensor_msgs::msg::Joy>(
      config_.topic, rclcpp::QoS(10),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr msg) { on_msg(*msg); });
  }

  std::optional<OperatingState> operating_state(TimePoint now) const override
  {
    if (!monitor_.has_data() || monitor_.is_stale(now))
    {
      return std::nullopt;
    }
    return enabled_ ? OperatingState::kDriving : OperatingState::kReady;
  }

private:
  void on_msg(const sensor_msgs::msg::Joy & msg)
  {
    enabled_ = enable_button_ < msg.buttons.size() && msg.buttons[enable_button_] != 0;
    report(Severity::kOk, "");
  }

  std::size_t enable_button_{0};
  bool enabled_{false};
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
};

template <typename T>
InputFactory make_factory(const char * default_topic, double default_timeout_s)
{
  return [default_topic, default_timeout_s](const InputContext & context)
  {
    const auto config = declare_common_parameters(context, default_topic, default_timeout_s);
    return std::unique_ptr<StatusInput>(std::make_unique<T>(context, config));
  };
}

}  // namespace

StatusInput::StatusInput(const InputContext & context, const InputCommonConfig & config)
: name_(context.name),
  config_(config),
  monitor_(config.stale_code, config.timeout, context.now()),
  now_(context.now),
  on_update_(context.on_update)
{
}

void StatusInput::report(Severity severity, const std::string & code)
{
  monitor_.update(severity, code, now_());
  if (on_update_)
  {
    on_update_();
  }
}

InputCommonConfig declare_common_parameters(
  const InputContext & context, const std::string & default_topic, double default_timeout_s)
{
  auto & node = *context.node;
  const std::string p = context.name + ".";
  InputCommonConfig config;
  config.topic = node.declare_parameter<std::string>(p + "topic", default_topic);
  config.code = node.declare_parameter<std::string>(p + "code", to_upper(context.name));
  config.stale_code = node.declare_parameter<std::string>(p + "stale_code", config.code);
  const double timeout_s = node.declare_parameter<double>(p + "timeout_s", default_timeout_s);
  if (!(timeout_s > 0.0))
  {
    throw std::invalid_argument(p + "timeout_s must be > 0");
  }
  config.timeout = std::chrono::duration_cast<Duration>(std::chrono::duration<double>(timeout_s));
  return config;
}

const std::map<std::string, InputFactory> & input_factories()
{
  static const std::map<std::string, InputFactory> factories = {
    {"diagnostics", make_factory<DiagnosticsInput>("/diagnostics", 3.0)},
    {"battery", make_factory<BatteryInput>("/battery_state", 3.0)},
    {"joy", make_factory<JoyInput>("/joy", 1.0)},
  };
  return factories;
}

std::unique_ptr<StatusInput> make_input(const std::string & type, const InputContext & context)
{
  const auto & factories = input_factories();
  const auto it = factories.find(type);
  if (it == factories.end())
  {
    throw std::invalid_argument("unknown input type '" + type + "' for input " + context.name);
  }
  return it->second(context);
}

}  // namespace kvn_status
