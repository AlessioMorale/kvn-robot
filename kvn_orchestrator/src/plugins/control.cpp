// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Control-flow plugins that the stock BehaviorTree.CPP node set lacks.

#include <chrono>
#include <cmath>
#include <string>

#include <behaviortree_cpp/bt_factory.h>

#include "kvn_orchestrator/bt_utils.hpp"

namespace kvn_orchestrator
{

/// Two children: a body and a handler.
///
/// The body runs like a single-child decorator. If it FAILS, or is halted while running (for
/// example by a reactive parent whose guard condition went false), the handler is ticked so it can
/// undo side effects. A halted handler is ticked once and then halted if it does not finish
/// immediately, so keep it synchronous. The node returns the body's result; the handler's result
/// is ignored. The handler is skipped when the body SUCCEEDS.
class OnAbort : public BT::ControlNode
{
public:
  OnAbort(const std::string & name, const BT::NodeConfig & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (children_nodes_.size() != 2) {
      throw BT::LogicError("[", name(), "]: OnAbort needs exactly 2 children: body and handler");
    }
    setStatus(BT::NodeStatus::RUNNING);

    if (!in_handler_) {
      const auto status = children_nodes_[0]->executeTick();
      switch (status) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        case BT::NodeStatus::SUCCESS:
          resetChildren();
          return BT::NodeStatus::SUCCESS;
        case BT::NodeStatus::SKIPPED:
          resetChildren();
          return BT::NodeStatus::SKIPPED;
        default:
          in_handler_ = true;
          break;
      }
    }

    if (children_nodes_[1]->executeTick() == BT::NodeStatus::RUNNING) {
      return BT::NodeStatus::RUNNING;
    }
    in_handler_ = false;
    resetChildren();
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    if (in_handler_) {
      haltChild(1);
    } else if (children_nodes_.size() == 2 &&
      children_nodes_[0]->status() == BT::NodeStatus::RUNNING)
    {
      haltChild(0);
      if (children_nodes_[1]->executeTick() == BT::NodeStatus::RUNNING) {
        haltChild(1);
      }
    }
    in_handler_ = false;
    BT::ControlNode::halt();
  }

private:
  bool in_handler_{false};
};

/// RUNNING for `seconds`, then SUCCESS. Unlike the stock Sleep node it accepts any number
/// ("1", "0.5"), so it can be parameterized from a SubTree.
class WaitSeconds : public BT::StatefulActionNode
{
public:
  WaitSeconds(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort("seconds", "Time to wait")};
  }

  BT::NodeStatus onStart() override
  {
    const double seconds = require_number(*this, "seconds");
    if (!std::isfinite(seconds) || seconds < 0.0) {
      throw BT::RuntimeError(
              "[", fullPath(), "] input port 'seconds' must be finite and >= 0");
    }
    deadline_ = std::chrono::steady_clock::now() + std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
    return onRunning();
  }

  BT::NodeStatus onRunning() override
  {
    return std::chrono::steady_clock::now() >= deadline_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace kvn_orchestrator

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<kvn_orchestrator::OnAbort>("OnAbort");
  factory.registerNodeType<kvn_orchestrator::WaitSeconds>("WaitSeconds");
}
