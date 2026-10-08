// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "kvn_status/status_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"

namespace kvn_status
{
namespace
{

using std::chrono::milliseconds;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;

bool spin_until(
  rclcpp::executors::SingleThreadedExecutor & executor, const std::function<bool()> & predicate,
  milliseconds timeout, const std::function<void()> & publish = nullptr)
{
  const auto start = std::chrono::steady_clock::now();
  auto last_publish = start - std::chrono::hours(1);
  while ((std::chrono::steady_clock::now() - start) < timeout)
  {
    // Republish periodically: the first messages may be lost during discovery
    if (publish && std::chrono::steady_clock::now() - last_publish > milliseconds(50))
    {
      publish();
      last_publish = std::chrono::steady_clock::now();
    }
    executor.spin_some();
    if (predicate())
    {
      return true;
    }
    std::this_thread::sleep_for(milliseconds(5));
  }
  return false;
}

DiagnosticArray make_diag(const std::string & name, uint8_t level)
{
  DiagnosticArray array;
  DiagnosticStatus status;
  status.name = name;
  status.level = level;
  array.status.push_back(status);
  return array;
}

class StatusNodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok())
    {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
    // Unique topics per test so tests do not see each other's messages
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    prefix_ = std::string("/kvn_status_test_") + info->name();
    status_topic_ = prefix_ + "/robot_status";
    options_.context(rclcpp::contexts::get_global_default_context());
    options_.append_parameter_override("status_topic", status_topic_);
  }

  void TearDown() override
  {
    executor_.reset();
    sub_.reset();
    helper_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  /// Create the node under test plus a helper node subscribed to its status.
  void start()
  {
    node_ = std::make_shared<StatusNode>(options_);
    helper_ = std::make_shared<rclcpp::Node>("kvn_status_test_helper");
    sub_ = helper_->create_subscription<std_msgs::msg::String>(
      status_topic_, 10,
      [this](const std_msgs::msg::String::ConstSharedPtr msg) { received_.push_back(msg->data); });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(helper_);
  }

  std::string last() const { return received_.empty() ? std::string{} : received_.back(); }

  bool wait_for(
    const std::string & expected, milliseconds timeout,
    const std::function<void()> & publish = nullptr)
  {
    return spin_until(*executor_, [&]() { return last() == expected; }, timeout, publish);
  }

  std::string prefix_;
  std::string status_topic_;
  rclcpp::NodeOptions options_;
  std::shared_ptr<StatusNode> node_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  // Created after rclcpp::init (in start())
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::string> received_;
};

TEST_F(StatusNodeTest, ReportsDiagnosticsLevels)
{
  const auto diag_topic = prefix_ + "/diagnostics";
  options_.append_parameter_override("inputs", std::vector<std::string>{"hwmon"});
  options_.append_parameter_override("hwmon.type", "diagnostics");
  options_.append_parameter_override("hwmon.topic", diag_topic);
  options_.append_parameter_override("hwmon.name_contains", "hwmon_status");
  options_.append_parameter_override("hwmon.timeout_s", 5.0);
  start();
  auto pub = helper_->create_publisher<DiagnosticArray>(diag_topic, 10);
  const std::string name = "hwmon_diagnostic_updater_node: hwmon_status";

  EXPECT_TRUE(wait_for(
    "FLT:HWMON", milliseconds(3000),
    [&]() { pub->publish(make_diag(name, DiagnosticStatus::ERROR)); }));
  EXPECT_TRUE(wait_for(
    "WRN:HWMON", milliseconds(3000),
    [&]() { pub->publish(make_diag(name, DiagnosticStatus::WARN)); }));
  EXPECT_TRUE(wait_for(
    "RDY", milliseconds(3000), [&]() { pub->publish(make_diag(name, DiagnosticStatus::OK)); }));
  EXPECT_TRUE(wait_for(
    "FLT:HWMON", milliseconds(3000),
    [&]() { pub->publish(make_diag(name, DiagnosticStatus::STALE)); }));

  // Statuses that do not match are ignored
  EXPECT_TRUE(wait_for(
    "RDY", milliseconds(3000), [&]() { pub->publish(make_diag(name, DiagnosticStatus::OK)); }));
  bool changed = spin_until(
    *executor_, [&]() { return last() != "RDY"; }, milliseconds(500),
    [&]() { pub->publish(make_diag("other_node: motor", DiagnosticStatus::ERROR)); });
  EXPECT_FALSE(changed) << last();
}

TEST_F(StatusNodeTest, StaleDiagnosticsBecomeFault)
{
  const auto diag_topic = prefix_ + "/diagnostics";
  options_.append_parameter_override("inputs", std::vector<std::string>{"hwmon"});
  options_.append_parameter_override("hwmon.type", "diagnostics");
  options_.append_parameter_override("hwmon.topic", diag_topic);
  options_.append_parameter_override("hwmon.name_contains", "hwmon_status");
  options_.append_parameter_override("hwmon.timeout_s", 0.5);
  start();
  auto pub = helper_->create_publisher<DiagnosticArray>(diag_topic, 10);
  const std::string name = "hwmon_diagnostic_updater_node: hwmon_status";

  ASSERT_TRUE(wait_for(
    "RDY", milliseconds(3000), [&]() { pub->publish(make_diag(name, DiagnosticStatus::OK)); }));
  // Publisher stops: fault within timeout + 0.5 s
  const auto stopped = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait_for("FLT:HWMON", milliseconds(1000)));
  EXPECT_GE(std::chrono::steady_clock::now() - stopped, milliseconds(400));
}

TEST_F(StatusNodeTest, NeverReceivedInputIsFaultAfterGrace)
{
  options_.append_parameter_override("inputs", std::vector<std::string>{"hwmon"});
  options_.append_parameter_override("hwmon.type", "diagnostics");
  options_.append_parameter_override("hwmon.topic", prefix_ + "/diagnostics");
  options_.append_parameter_override("hwmon.timeout_s", 0.5);
  options_.append_parameter_override("hwmon.stale_code", "HWMON_STALE");
  start();
  EXPECT_TRUE(wait_for("FLT:HWMON_STALE", milliseconds(2000)));
}

TEST_F(StatusNodeTest, JoyEnableButtonSelectsDriveAndStaleJoyFaults)
{
  const auto joy_topic = prefix_ + "/joy";
  options_.append_parameter_override("inputs", std::vector<std::string>{"joy"});
  options_.append_parameter_override("joy.type", "joy");
  options_.append_parameter_override("joy.topic", joy_topic);
  options_.append_parameter_override("joy.enable_button", 1);
  options_.append_parameter_override("joy.timeout_s", 0.5);
  start();
  auto pub = helper_->create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);
  auto joy = [&](int32_t enable)
  {
    sensor_msgs::msg::Joy msg;
    msg.buttons = {0, enable};
    pub->publish(msg);
  };

  EXPECT_TRUE(wait_for("DRV", milliseconds(3000), [&]() { joy(1); }));
  EXPECT_TRUE(wait_for("RDY", milliseconds(3000), [&]() { joy(0); }));
  EXPECT_TRUE(wait_for("DRV", milliseconds(3000), [&]() { joy(1); }));
  EXPECT_TRUE(wait_for("FLT:JOY", milliseconds(1500)));
}

TEST_F(StatusNodeTest, BatteryThresholds)
{
  const auto battery_topic = prefix_ + "/battery_state";
  options_.append_parameter_override("inputs", std::vector<std::string>{"battery"});
  options_.append_parameter_override("battery.type", "battery");
  options_.append_parameter_override("battery.topic", battery_topic);
  options_.append_parameter_override("battery.code", "BATT");
  options_.append_parameter_override("battery.warn_voltage", 13.6);
  options_.append_parameter_override("battery.fault_voltage", 12.4);
  start();
  auto pub = helper_->create_publisher<sensor_msgs::msg::BatteryState>(battery_topic, 10);
  auto battery = [&](float voltage)
  {
    sensor_msgs::msg::BatteryState msg;
    msg.voltage = voltage;
    pub->publish(msg);
  };

  EXPECT_TRUE(wait_for("RDY", milliseconds(3000), [&]() { battery(15.0f); }));
  EXPECT_TRUE(wait_for("WRN:BATT", milliseconds(3000), [&]() { battery(13.0f); }));
  EXPECT_TRUE(wait_for("FLT:BATT", milliseconds(3000), [&]() { battery(12.0f); }));
}

TEST_F(StatusNodeTest, PublishesPeriodicallyAtTwoHertz)
{
  start();
  ASSERT_TRUE(wait_for("RDY", milliseconds(3000)));
  received_.clear();
  spin_until(*executor_, []() { return false; }, milliseconds(2000));
  // 2 Hz over 2 s: about 4 messages
  EXPECT_GE(received_.size(), 3U);
  EXPECT_LE(received_.size(), 5U);
}

TEST_F(StatusNodeTest, UnknownInputTypeIsConfigFault)
{
  options_.append_parameter_override("inputs", std::vector<std::string>{"bogus"});
  options_.append_parameter_override("bogus.type", "no_such_type");
  start();
  EXPECT_TRUE(wait_for("FLT:CFG", milliseconds(3000)));
}

}  // namespace
}  // namespace kvn_status
