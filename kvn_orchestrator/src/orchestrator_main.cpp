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
          "nav2_behavior_tree:nav2_navigate_to_pose_action_bt_node",
          "nav2_behavior_tree:nav2_goal_reached_condition_bt_node"});
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
