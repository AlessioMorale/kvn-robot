// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_orchestrator/tree_host.hpp"

#include <chrono>
#include <exception>
#include <filesystem>
#include <sstream>
#include <stdexcept>

#include <ament_index_cpp/get_package_prefix.hpp>

#include "kvn_orchestrator/bt_utils.hpp"

namespace kvn_orchestrator
{

std::string TreeHost::resolve_plugin(const std::string & spec)
{
  if (spec.find('/') != std::string::npos) {
    return spec;
  }
  std::string package = "kvn_orchestrator";
  std::string library = spec;
  if (const auto colon = spec.find(':'); colon != std::string::npos) {
    package = spec.substr(0, colon);
    library = spec.substr(colon + 1);
  }
  if (library.empty()) {
    throw std::invalid_argument("empty plugin library name in '" + spec + "'");
  }
  const std::string file = library.ends_with(".so") ? library : "lib" + library + ".so";
  std::filesystem::path prefix;
  ament_index_cpp::get_package_prefix(package, prefix);
  return (prefix / "lib" / file).string();
}

namespace
{

/// Blackboard value for a ROS parameter. Arrays become ';' separated strings, the list syntax of
/// the XML ports.
BT::Any to_blackboard_value(const std::string & name, const rclcpp::ParameterValue & value)
{
  const auto join = [](const auto & items) {
      std::ostringstream text;
      for (size_t i = 0; i < items.size(); ++i) {
        text << (i ? ";" : "") << items[i];
      }
      return text.str();
    };
  switch (value.get_type()) {
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return BT::Any(value.get<bool>());
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return BT::Any(static_cast<int>(value.get<int64_t>()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return BT::Any(value.get<double>());
    case rclcpp::ParameterType::PARAMETER_STRING:
      return BT::Any(value.get<std::string>());
    case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      return BT::Any(join(value.get<std::vector<int64_t>>()));
    case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
      return BT::Any(join(value.get<std::vector<double>>()));
    case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
      return BT::Any(join(value.get<std::vector<std::string>>()));
    default:
      throw std::invalid_argument(
              "blackboard entry '" + name + "' has an unsupported parameter type");
  }
}

}  // namespace

TreeHost::TreeHost(nav2::LifecycleNode::SharedPtr node, const TreeHostOptions & options)
: node_(std::move(node)), options_(options)
{
  if (options_.tick_period_s <= 0.0) {
    throw std::invalid_argument("tick_period_s must be > 0");
  }
  if (options_.heartbeat_period_s < 0.0) {
    throw std::invalid_argument("heartbeat_period_s must be >= 0");
  }
  if (!std::filesystem::exists(options_.tree_file)) {
    throw std::invalid_argument("behaviour tree file not found: '" + options_.tree_file + "'");
  }
  if (!options_.signalling_tree_file.empty() &&
    !std::filesystem::exists(options_.signalling_tree_file))
  {
    throw std::invalid_argument(
            "signalling tree file not found: '" + options_.signalling_tree_file + "'");
  }

  ctx_ = std::make_shared<RosContext>(node_);

  for (const auto & spec : options_.plugin_libraries) {
    const auto path = resolve_plugin(spec);
    RCLCPP_INFO(node_->get_logger(), "Loading behaviour-tree plugin %s", path.c_str());
    factory_.registerFromPlugin(path);
  }

  blackboard_ = BT::Blackboard::create();
  for (const auto & [name, value] : options_.blackboard_entries) {
    blackboard_->set(name, to_blackboard_value(name, value));
  }
  blackboard_->set(RosContext::kBlackboardKey, ctx_);
  // The entries nav2_behavior_tree nodes look up.
  blackboard_->set<nav2::LifecycleNode::SharedPtr>("node", node_);
  blackboard_->set<nav2::TransformBuffer::SharedPtr>("tf_buffer", ctx_->tf_buffer());
  blackboard_->set<std::chrono::milliseconds>(
    "bt_loop_duration",
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(options_.tick_period_s)));
  blackboard_->set<std::chrono::milliseconds>("server_timeout", options_.server_timeout);
  blackboard_->set<std::chrono::milliseconds>("cancel_timeout", options_.cancel_timeout);
  blackboard_->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", options_.wait_for_service_timeout);

  if (!options_.boot_status_topic.empty()) {
    ctx_->publish_status(options_.boot_status_topic, options_.boot_status);
  }

  // Needs no Nav2 server, so it is built (and its errors reported) right away.
  if (!options_.signalling_tree_file.empty()) {
    signalling_tree_ = factory_.createTreeFromFile(options_.signalling_tree_file, blackboard_);
    prepare(signalling_tree_);
    has_signalling_ = true;
    RCLCPP_INFO(
      node_->get_logger(), "Signalling tree loaded from %s",
      options_.signalling_tree_file.c_str());
  }

  // A bad tree throws from here; a missing Nav2 server is retried from the timer.
  try_build();
  const auto now = SteadyClock::now();
  next_build_attempt_ = now + std::chrono::seconds(1);
  next_heartbeat_ = now;
  timer_ = node_->create_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(options_.tick_period_s)),
    [this]() {
      try {
        on_timer();
      } catch (const std::exception & e) {
        RCLCPP_FATAL(node_->get_logger(), "Behaviour tree failed: %s", e.what());
        rclcpp::shutdown();
      }
    });
}

TreeHost::~TreeHost()
{
  timer_.reset();
  if (ready_) {
    // Running actions (e.g. an active Nav2 goal) get their halt/cancel callbacks.
    tree_.haltTree();
  }
  if (has_signalling_) {
    signalling_tree_.haltTree();
  }
}

bool TreeHost::try_build()
{
  try {
    if (options_.tree_id.empty()) {
      tree_ = factory_.createTreeFromFile(options_.tree_file, blackboard_);
    } else {
      factory_.registerBehaviorTreeFromFile(options_.tree_file);
      tree_ = factory_.createTree(options_.tree_id, blackboard_);
    }
  } catch (const std::runtime_error & e) {
    // Thrown by Nav2 action nodes whose server is not up yet. BT errors do not derive from
    // std::runtime_error and propagate.
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000, "Waiting for Nav2: %s", e.what());
    return false;
  }
  prepare(tree_);
  ready_ = true;
  RCLCPP_INFO(
    node_->get_logger(), "Behaviour tree '%s' loaded from %s",
    options_.tree_id.empty() ? "main" : options_.tree_id.c_str(), options_.tree_file.c_str());
  return true;
}

void TreeHost::prepare(BT::Tree & tree)
{
  for (const auto & subtree : tree.subtrees) {
    for (const auto & tree_node : subtree->nodes) {
      if (auto * preparable = dynamic_cast<Preparable *>(tree_node.get())) {
        preparable->prepare();
      }
    }
  }
}

void TreeHost::on_timer()
{
  const auto now = SteadyClock::now();
  if (!ready_ && now >= next_build_attempt_) {
    next_build_attempt_ = now + std::chrono::seconds(1);
    try_build();
  }
  tick();
  if (options_.heartbeat_period_s > 0.0 && now >= next_heartbeat_) {
    next_heartbeat_ = now + std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(options_.heartbeat_period_s));
    ctx_->republish_latched();
  }
}

BT::NodeStatus TreeHost::tick()
{
  if (has_signalling_) {
    signalling_tree_.tickOnce();
  }
  return ready_ ? tree_.tickOnce() : BT::NodeStatus::IDLE;
}

}  // namespace kvn_orchestrator
