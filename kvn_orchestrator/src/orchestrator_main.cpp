// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kvn_orchestrator/tree_host.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  {
    auto node = std::make_shared<nav2::LifecycleNode>("kvn_orchestrator_node");
    try {
      kvn_orchestrator::TreeHostOptions options;
      options.tree_file = node->declare_or_get_parameter<std::string>("tree_file", "");
      options.tree_id = node->declare_or_get_parameter<std::string>("tree_id", "");
      options.plugin_libraries = node->declare_or_get_parameter<std::vector<std::string>>(
        "plugin_libraries",
        std::vector<std::string>{
          "kvn_bt_conditions", "kvn_bt_actions", "kvn_bt_control",
          "kvn_bt_autonomy",
          "nav2_behavior_tree:nav2_navigate_to_pose_action_bt_node",
          "nav2_behavior_tree:nav2_goal_reached_condition_bt_node"});
      options.signalling_tree_file =
        node->declare_or_get_parameter<std::string>("signalling_tree_file", "");
      options.heartbeat_period_s =
        node->declare_or_get_parameter<double>("heartbeat_period_s", 1.0);
      options.boot_status_topic =
        node->declare_or_get_parameter<std::string>("boot_status_topic", "/orchestrator/mode");
      options.boot_status =
        node->declare_or_get_parameter<std::string>("boot_status", "waiting_for_nav2");
      // `blackboard.<name>` parameters (from the YAML or the launch file) become root blackboard
      // entries; they are read as overrides so that any name can be used without declaring it.
      const std::string prefix = "blackboard.";
      for (const auto & [name, value] :
        node->get_node_parameters_interface()->get_parameter_overrides())
      {
        if (name.rfind(prefix, 0) == 0 && name.size() > prefix.size()) {
          options.blackboard_entries[name.substr(prefix.size())] = value;
        }
      }
      options.tick_period_s = node->declare_or_get_parameter<double>("tick_period_s", 0.1);
      options.server_timeout = std::chrono::milliseconds(
        node->declare_or_get_parameter<int>("server_timeout_ms", 20));
      options.cancel_timeout = std::chrono::milliseconds(
        node->declare_or_get_parameter<int>("cancel_timeout_ms", 1000));
      options.wait_for_service_timeout = std::chrono::milliseconds(
        node->declare_or_get_parameter<int>("wait_for_service_timeout_ms", 1000));
      if (options.tree_file.empty()) {
        throw std::invalid_argument("parameter 'tree_file' must be set");
      }
      // Standalone node (no Nav2 lifecycle manager): bring it up ourselves so lifecycle
      // publishers are active.
      node->configure();
      node->activate();
      kvn_orchestrator::TreeHost host(node, options);
      rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception & e) {
      RCLCPP_FATAL(node->get_logger(), "%s", e.what());
      result = 1;
    }
  }
  rclcpp::shutdown();
  return result;
}
