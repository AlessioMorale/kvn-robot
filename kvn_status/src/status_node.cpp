// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_status/status_node.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <utility>

namespace kvn_status
{

StatusNode::StatusNode(const rclcpp::NodeOptions & options) : Node("kvn_status_node", options)
{
  const auto status_topic = this->declare_parameter<std::string>("status_topic", "/robot_status");
  const double publish_rate_hz = this->declare_parameter<double>("publish_rate_hz", 2.0);
  const int64_t check_period_ms = this->declare_parameter<int64_t>("check_period_ms", 100);
  const auto config_fault_code = this->declare_parameter<std::string>("config_fault_code", "CFG");

  if (!(publish_rate_hz > 0.0) || check_period_ms <= 0)
  {
    throw std::invalid_argument("publish_rate_hz and check_period_ms must be > 0");
  }

  load_inputs();
  for (auto & fault : config_faults_)
  {
    fault.code = config_fault_code;
  }

  publisher_ = this->create_publisher<std_msgs::msg::String>(status_topic, rclcpp::QoS(10));

  publish_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_hz)),
    [this]() { evaluate_and_publish(true); });
  // Staleness has no event of its own: poll for it between periodic publishes
  check_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(check_period_ms), [this]() { evaluate_and_publish(false); });

  RCLCPP_INFO(
    this->get_logger(), "Publishing status on %s at %.1f Hz with %zu inputs", status_topic.c_str(),
    publish_rate_hz, inputs_.size());
}

void StatusNode::load_inputs()
{
  // Copy: as_string_array() returns a reference into the temporary Parameter
  const std::vector<std::string> names =
    this->declare_parameter<std::vector<std::string>>("inputs", std::vector<std::string>{});
  if (names.empty())
  {
    RCLCPP_WARN(this->get_logger(), "No inputs configured: status is always RDY");
  }

  for (const auto & name : names)
  {
    InputContext context;
    context.node = this;
    context.name = name;
    context.now = [this]() { return now_steady(); };
    context.on_update = [this]() { evaluate_and_publish(false); };
    try
    {
      const auto type = this->declare_parameter<std::string>(name + ".type", "");
      inputs_.push_back(make_input(type, context));
      RCLCPP_INFO(this->get_logger(), "Input '%s' of type '%s'", name.c_str(), type.c_str());
    }
    catch (const std::exception & e)
    {
      // Keep running so the radio shows the problem instead of a dead link
      RCLCPP_ERROR(this->get_logger(), "Input '%s' disabled: %s", name.c_str(), e.what());
      config_faults_.push_back(Condition{Severity::kFault, "", now_steady()});
    }
  }
}

TimePoint StatusNode::now_steady() const { return TimePoint(steady_clock_.now().nanoseconds()); }

void StatusNode::evaluate_and_publish(bool force)
{
  if (!publisher_)
  {
    // An input reported during construction
    return;
  }
  const auto now = now_steady();
  std::vector<Condition> conditions = config_faults_;
  OperatingState state = OperatingState::kReady;
  for (const auto & input : inputs_)
  {
    conditions.push_back(input->evaluate(now));
    if (input->operating_state(now) == OperatingState::kDriving)
    {
      state = OperatingState::kDriving;
    }
  }

  auto status = reduce_status(conditions, state);
  const bool changed = !published_once_ || status != last_status_;
  if (!changed && !force)
  {
    return;
  }
  if (changed)
  {
    RCLCPP_INFO(this->get_logger(), "Status: %s", status.c_str());
  }
  last_status_ = std::move(status);
  published_once_ = true;
  std_msgs::msg::String msg;
  msg.data = last_status_;
  publisher_->publish(msg);
}

}  // namespace kvn_status
