// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_display/display_node.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <system_error>

namespace kvn_display
{

namespace
{

std::chrono::steady_clock::duration seconds(double value, const std::string & name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(name + " must be finite and > 0");
  }
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(value));
}

}  // namespace

DisplayNode::DisplayNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("kvn_display_node", options)
{
  config_.driver = declare_parameter<std::string>("driver", config_.driver);
  config_.rotation = static_cast<int>(declare_parameter<int64_t>("rotation", config_.rotation));
  config_.contrast = static_cast<int>(declare_parameter<int64_t>("contrast", config_.contrast));
  config_.i2c_device = declare_parameter<std::string>("i2c_device", config_.i2c_device);
  config_.i2c_address =
    static_cast<int>(declare_parameter<int64_t>("i2c_address", config_.i2c_address));
  config_.spi.device = declare_parameter<std::string>("spi_device", config_.spi.device);
  const auto spi_speed_hz = declare_parameter<int64_t>("spi_speed_hz", 0);
  if (spi_speed_hz < 0 || spi_speed_hz > UINT32_MAX) {
    throw std::invalid_argument("spi_speed_hz must be >= 0 (0: the display's maximum)");
  }
  config_.spi.speed_hz = static_cast<std::uint32_t>(spi_speed_hz);
  config_.spi.gpio_chip = declare_parameter<std::string>("gpio_chip", config_.spi.gpio_chip);
  config_.spi.dc_line = static_cast<int>(declare_parameter<int64_t>("dc_line", -1));
  config_.spi.reset_line = static_cast<int>(declare_parameter<int64_t>("reset_line", -1));
  status_timeout_ = seconds(declare_parameter<double>("status_timeout_s", 3.0), "status_timeout_s");
  battery_timeout_ =
    seconds(declare_parameter<double>("battery_timeout_s", 3.0), "battery_timeout_s");
  mode_timeout_ = seconds(declare_parameter<double>("mode_timeout_s", 3.0), "mode_timeout_s");
  const auto update_period =
    seconds(declare_parameter<double>("update_period_s", 0.5), "update_period_s");

  validate(config_);

  const auto latched = rclcpp::QoS(1).reliable().transient_local();
  status_sub_ = create_subscription<std_msgs::msg::String>(
    declare_parameter<std::string>("status_topic", "/robot_status"), rclcpp::QoS(10),
    [this](const std_msgs::msg::String & msg) {status_ = Timestamped{msg.data, Clock::now()};});
  // Latched and re-sent every second by the orchestrator, which only runs with Nav2
  mode_sub_ = create_subscription<std_msgs::msg::String>(
    declare_parameter<std::string>("mode_topic", "/orchestrator/mode"), latched,
    [this](const std_msgs::msg::String & msg) {mode_ = Timestamped{msg.data, Clock::now()};});
  // Latched and published on change only, so it has no timeout
  cmd_source_sub_ = create_subscription<std_msgs::msg::String>(
    declare_parameter<std::string>("cmd_source_topic", "/cmd_vel_source"), latched,
    [this](const std_msgs::msg::String & msg) {cmd_source_ = msg.data;});
  battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
    declare_parameter<std::string>("battery_topic", "/battery_state"), rclcpp::QoS(10),
    [this](const sensor_msgs::msg::BatteryState & msg) {
      battery_voltage_ = msg.voltage;
      battery_percentage_ = msg.percentage;
      battery_time_ = Clock::now();
    });

  timer_ = create_wall_timer(update_period, [this]() {update();});
  update();
}

ScreenState DisplayNode::state(Clock::time_point now) const
{
  ScreenState screen;
  if (status_ && now - status_->time <= status_timeout_) {
    screen.robot_status = status_->value;
  }
  if (mode_ && now - mode_->time <= mode_timeout_) {
    screen.mode = mode_->value;
  }
  screen.cmd_source = cmd_source_;
  if (battery_time_ && now - *battery_time_ <= battery_timeout_) {
    screen.battery_voltage = battery_voltage_;
    screen.battery_percentage = battery_percentage_;
  }
  return screen;
}

void DisplayNode::connect()
{
  try {
    display_ = std::make_unique<Display>(config_);
    shown_.reset();
    if (is_i2c_driver(config_.driver)) {
      RCLCPP_INFO(
        get_logger(), "%s (%dx%d) on %s at 0x%02x", config_.driver.c_str(), display_->width(),
        display_->height(), config_.i2c_device.c_str(), config_.i2c_address);
    } else {
      RCLCPP_INFO(
        get_logger(), "%s (%dx%d) on %s", config_.driver.c_str(), display_->width(),
        display_->height(), config_.spi.device.c_str());
    }
  } catch (const std::system_error & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 10000, "Display not available: %s", error.what());
  }
}

void DisplayNode::update()
{
  if (!display_) {
    connect();
    if (!display_) {
      return;
    }
  }
  auto lines = screen_lines(state(Clock::now()));
  if (shown_ && *shown_ == lines) {
    return;
  }
  try {
    display_->show(lines);
    shown_ = std::move(lines);
  } catch (const std::system_error & error) {
    RCLCPP_WARN(get_logger(), "Display write failed, reconnecting: %s", error.what());
    display_.reset();
  }
}

}  // namespace kvn_display
