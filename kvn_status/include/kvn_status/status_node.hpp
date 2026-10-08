// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "kvn_status/inputs.hpp"
#include "kvn_status/status_reducer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace kvn_status
{

/**
 * @class StatusNode
 * @brief Reduces robot health inputs to one short status string.
 *
 * Publishes `<STATE>[:<CODE>]` (at most 15 ASCII chars) on `status_topic`
 * at `publish_rate_hz` and immediately when it changes. Inputs are listed
 * by name in `inputs` and configured under `<name>.*` (see README.md).
 */
class StatusNode : public rclcpp::Node
{
public:
  explicit StatusNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Last published status string.
  const std::string & status() const { return last_status_; }

private:
  void load_inputs();
  TimePoint now_steady() const;
  /// Recompute the status; publish if it changed or @p force is set.
  void evaluate_and_publish(bool force);

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  std::vector<std::unique_ptr<StatusInput>> inputs_;
  /// Configuration errors are reported as a permanent fault.
  std::vector<Condition> config_faults_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  std::string last_status_;
  bool published_once_{false};
};

}  // namespace kvn_status
