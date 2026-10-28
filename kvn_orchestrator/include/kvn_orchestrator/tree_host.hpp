// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>
#include <nav2_ros_common/lifecycle_node.hpp>

#include "kvn_orchestrator/ros_context.hpp"

namespace kvn_orchestrator
{

struct TreeHostOptions
{
  /// BehaviorTree.CPP XML file; <include> paths are resolved relative to it.
  std::string tree_file;
  /// Tree to run. Empty selects main_tree_to_execute of the file.
  std::string tree_id;
  /// Plugin libraries providing BT node types. See TreeHost::resolve_plugin.
  std::vector<std::string> plugin_libraries;
  double tick_period_s{0.1};

  /// Optional second XML file, ticked next to the main tree from startup on (it must not need
  /// Nav2 servers). Used for the signalling light, so it works while Nav2 is still coming up.
  std::string signalling_tree_file;

  /// Period at which latched status/mode/flag topics are sent again (0 disables). Subscribers
  /// use the repeats to detect a dead orchestrator.
  double heartbeat_period_s{1.0};

  /// Published on boot_status_topic before the main tree is running (empty topic: nothing).
  std::string boot_status_topic;
  std::string boot_status;

  /// Root blackboard entries, e.g. hardware limits from the robot configuration, readable from
  /// the XML as {name}. Set from the node parameters `blackboard.<name>`.
  std::map<std::string, rclcpp::ParameterValue> blackboard_entries;

  // Blackboard entries that Nav2's BT nodes (nav2_behavior_tree::BtActionNode...) read.
  std::chrono::milliseconds server_timeout{20};
  std::chrono::milliseconds cancel_timeout{1000};
  std::chrono::milliseconds wait_for_service_timeout{1000};
};

/// Loads node plugins and the XML tree definitions, then ticks the trees from a Nav2 timer.
///
/// The host is the Nav2 BT "environment": it puts on the blackboard what nav2_behavior_tree nodes
/// expect (node, tf_buffer, bt_loop_duration, server/cancel/wait_for_service timeouts), so Nav2's
/// stock BT plugins can be listed in `plugin_libraries` and used from XML next to ours.
///
/// Nav2 action nodes throw std::runtime_error from their constructor while their action server is
/// missing. Building the tree is retried until the servers are up; any other error (bad XML,
/// unknown node, bad port value) is fatal and thrown to the caller.
class TreeHost
{
public:
  /// \p node must be active (or become active) so lifecycle publishers can publish.
  TreeHost(nav2::LifecycleNode::SharedPtr node, const TreeHostOptions & options);
  ~TreeHost();

  /// True once the tree has been built, i.e. all Nav2 servers it needs were found.
  bool ready() const {return ready_;}

  /// Ticks the signalling tree (if any) and, once built, the main tree; used by the timer and by
  /// tests. Returns the main tree's status, IDLE while it is not built yet.
  BT::NodeStatus tick();

  /// Resolves a plugin name to a shared-library path:
  ///   "name"      lib<name>.so from this package (kvn_orchestrator)
  ///   "pkg:name"  lib<name>.so from another ament package, e.g.
  ///               "nav2_behavior_tree:nav2_navigate_to_pose_action_bt_node"
  ///   "/a/b.so"   an explicit path
  static std::string resolve_plugin(const std::string & spec);

  BT::Blackboard::Ptr blackboard() const {return blackboard_;}

private:
  bool try_build();
  void prepare(BT::Tree & tree);
  void on_timer();

  nav2::LifecycleNode::SharedPtr node_;
  TreeHostOptions options_;
  std::shared_ptr<RosContext> ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
  BT::Tree signalling_tree_;
  bool has_signalling_{false};
  bool ready_{false};
  SteadyClock::time_point next_build_attempt_;
  SteadyClock::time_point next_heartbeat_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kvn_orchestrator
