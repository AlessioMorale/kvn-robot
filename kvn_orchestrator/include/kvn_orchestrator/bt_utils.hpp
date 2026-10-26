// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <behaviortree_cpp/bt_factory.h>

#include "kvn_orchestrator/ros_context.hpp"

namespace kvn_orchestrator
{

/// Implemented by nodes that must create ROS endpoints and validate their configuration before
/// the first tick. The tree host calls prepare() on every node once the tree is built, so that
/// subscriptions are already discovered (latched topics included) and publishers are connected
/// when the first button press arrives, and so that a bad XML port fails at startup.
class Preparable
{
public:
  virtual ~Preparable() = default;
  virtual void prepare() = 0;
};

/// The RosContext the tree host placed on the blackboard.
inline std::shared_ptr<RosContext> get_context(const BT::NodeConfig & config)
{
  std::shared_ptr<RosContext> ctx;
  if (!config.blackboard || !config.blackboard->get(RosContext::kBlackboardKey, ctx) || !ctx) {
    throw BT::RuntimeError(
            "blackboard entry '", RosContext::kBlackboardKey,
            "' is missing; the tree must be created by kvn_orchestrator::TreeHost");
  }
  return ctx;
}

/// Reads a required input port, turning a missing/ill-formed value into a descriptive error.
template<typename T>
T require_input(const BT::TreeNode & node, const std::string & port)
{
  auto value = node.getInput<T>(port);
  if (!value) {
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "': ", value.error());
  }
  return value.value();
}

/// Like require_input, additionally rejecting values that fail \p ok with \p requirement.
template<typename T, typename Pred>
T require_input(
  const BT::TreeNode & node, const std::string & port, Pred ok, const std::string & requirement)
{
  T value = require_input<T>(node, port);
  if (!ok(value)) {
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "' must be ", requirement);
  }
  return value;
}

/// Value of an input port whatever its type: a literal is returned as a string, a blackboard
/// reference ("{key}") as the entry's own value. (getInput<T> would reject entries whose inferred
/// type differs from T, which is what happens to constants passed to a SubTree.)
inline BT::Any read_any(const BT::TreeNode & node, const std::string & port)
{
  const std::string raw(node.getRawPortValue(port));
  BT::StringView key;
  if (BT::TreeNode::isBlackboardPointer(raw, &key)) {
    const auto entry = node.config().blackboard->getAnyLocked(std::string(key));
    if (!entry || entry.get()->empty()) {
      throw BT::RuntimeError(
              "[", node.fullPath(), "] input port '", port, "': blackboard entry '", key,
              "' is not set");
    }
    return *entry.get();
  }
  if (raw.empty()) {
    throw BT::RuntimeError("[", node.fullPath(), "] input port '", port, "' is missing");
  }
  return BT::Any(raw);
}

/// Reads a numeric input port. BehaviorTree.CPP infers the type of constants passed to a SubTree
/// ("2" is an int, "2.0" a double), so numeric ports are declared untyped and converted here;
/// a string such as "1.5" is accepted too.
inline double require_number(const BT::TreeNode & node, const std::string & port)
{
  const BT::Any any = read_any(node, port);
  if (any.isNumber()) {
    return any.cast<double>();
  }
  if (any.isString()) {
    const auto text = any.cast<std::string>();
    try {
      size_t used = 0;
      const double value = std::stod(text, &used);
      if (used == text.size()) {
        return value;
      }
    } catch (const std::exception &) {
    }
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port, "' is not a number: '", text, "'");
  }
  throw BT::RuntimeError("[", node.fullPath(), "] input port '", port, "' is not a number");
}

/// Reads a list of non-negative integers written as "2", "2;3" or "2,3".
inline std::vector<int> require_index_list(const BT::TreeNode & node, const std::string & port)
{
  const BT::Any any = read_any(node, port);
  std::vector<int> indices;
  std::string text = any.isNumber() ? std::to_string(any.cast<int>()) : any.cast<std::string>();
  std::replace(text.begin(), text.end(), ',', ';');
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, ';')) {
    try {
      size_t used = 0;
      const int value = std::stoi(item, &used);
      if (value >= 0 && used == item.find_last_not_of(" \t") + 1) {
        indices.push_back(value);
        continue;
      }
    } catch (const std::exception &) {
    }
    throw BT::RuntimeError(
            "[", node.fullPath(), "] input port '", port,
            "' must be a list of indices >= 0, e.g. \"2\" or \"2;3\", got '", text, "'");
  }
  if (indices.empty()) {
    throw BT::RuntimeError("[", node.fullPath(), "] input port '", port, "' must not be empty");
  }
  return indices;
}

}  // namespace kvn_orchestrator
