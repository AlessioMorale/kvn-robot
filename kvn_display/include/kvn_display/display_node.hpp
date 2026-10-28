// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "kvn_display/display.hpp"
#include "kvn_display/screen.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/string.hpp"

namespace kvn_display
{

/**
 * @class DisplayNode
 * @brief Shows the robot status on a small display (any u8g2 driver, I2C or SPI).
 *
 * Reads /robot_status, /battery_state, /orchestrator/mode and /cmd_vel_source, redraws every
 * `update_period_s` and writes to the bus only when the picture changes. A missing or failing
 * display is retried every period, so the node survives a loose connector.
 */
class DisplayNode : public rclcpp::Node
{
public:
  explicit DisplayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using Clock = std::chrono::steady_clock;

  struct Timestamped
  {
    std::string value;
    Clock::time_point time;
  };

  void update();
  void connect();
  ScreenState state(Clock::time_point now) const;

  DisplayConfig config_;
  Clock::duration status_timeout_;
  Clock::duration battery_timeout_;
  Clock::duration mode_timeout_;

  std::optional<Timestamped> status_;
  std::optional<Timestamped> mode_;
  std::optional<std::string> cmd_source_;
  std::optional<double> battery_voltage_;
  std::optional<double> battery_percentage_;
  std::optional<Clock::time_point> battery_time_;

  std::unique_ptr<Display> display_;
  std::optional<std::vector<Line>> shown_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_source_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kvn_display
