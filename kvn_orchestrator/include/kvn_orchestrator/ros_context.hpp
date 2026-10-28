// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <utility>
#include <vector>

#include <nav2_ros_common/lifecycle_node.hpp>
#include <nav2_ros_common/qos_profiles.hpp>
#include <nav2_ros_common/tf2_factories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace kvn_orchestrator
{

using SteadyClock = std::chrono::steady_clock;

/// Cached subscription: keeps the latest message with its arrival time and lets behaviour-tree
/// nodes observe every message through listeners.
///
/// Everything runs on the node's (single-threaded) executor, the same thread that ticks the tree,
/// so no locking is needed.
template<typename MsgT>
class Topic
{
public:
  using Listener = std::function<void (const MsgT &)>;

  Topic(nav2::LifecycleNode & node, const std::string & name, const rclcpp::QoS & qos)
  {
    subscription_ = node.create_subscription<MsgT>(
      name, [this](const typename MsgT::ConstSharedPtr msg) {on_message(*msg);}, qos);
  }

  const std::optional<MsgT> & latest() const {return latest_;}

  bool fresh(double timeout_s) const
  {
    return latest_.has_value() &&
           std::chrono::duration<double>(SteadyClock::now() - stamp_).count() <= timeout_s;
  }

  /// The listener stays registered for as long as the returned handle is alive.
  std::shared_ptr<Listener> listen(Listener listener)
  {
    auto handle = std::make_shared<Listener>(std::move(listener));
    listeners_.push_back(handle);
    return handle;
  }

private:
  void on_message(const MsgT & msg)
  {
    latest_ = msg;
    stamp_ = SteadyClock::now();
    listeners_.erase(
      std::remove_if(
        listeners_.begin(), listeners_.end(),
        [](const std::weak_ptr<Listener> & w) {return w.expired();}),
      listeners_.end());
    const auto snapshot = listeners_;
    for (const auto & weak : snapshot) {
      if (auto listener = weak.lock()) {
        (*listener)(msg);
      }
    }
  }

  std::optional<MsgT> latest_;
  SteadyClock::time_point stamp_;
  std::vector<std::weak_ptr<Listener>> listeners_;
  typename nav2::Subscription<MsgT>::SharedPtr subscription_;
};

/// Subscription of any message type that only records when the last message arrived. Used for
/// liveness checks (is the EKF / lidar publishing?) without compiling in the message type.
class TopicActivity
{
public:
  TopicActivity(
    nav2::LifecycleNode & node, const std::string & name, const std::string & type,
    const rclcpp::QoS & qos)
  {
    subscription_ = node.create_generic_subscription(
      name, type, qos,
      [this](std::shared_ptr<const rclcpp::SerializedMessage>) {
        received_ = true;
        stamp_ = SteadyClock::now();
      });
  }

  bool fresh(double timeout_s) const
  {
    return received_ &&
           std::chrono::duration<double>(SteadyClock::now() - stamp_).count() <= timeout_s;
  }

private:
  bool received_{false};
  SteadyClock::time_point stamp_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

/// ROS plumbing shared by every behaviour-tree plugin. The tree host stores one instance on the
/// root blackboard under the key "ctx".
///
/// Built on the Nav2 API (nav2::LifecycleNode, nav2 QoS profiles and TF factories) so that our
/// plugins and Nav2's own BT plugins share one node, one TF buffer and the same conventions.
class RosContext
{
public:
  static constexpr const char * kBlackboardKey = "ctx";

  explicit RosContext(nav2::LifecycleNode::SharedPtr node);

  const nav2::LifecycleNode::SharedPtr & node() const {return node_;}
  rclcpp::Logger logger() const {return node_->get_logger();}
  const nav2::TransformBuffer::SharedPtr & tf_buffer() const {return tf_buffer_;}

  /// Returns the shared subscription for (type, topic), creating it on first use. The QoS of the
  /// first request wins.
  template<typename MsgT>
  std::shared_ptr<Topic<MsgT>> subscribe(const std::string & topic, const rclcpp::QoS & qos)
  {
    if (topic.empty()) {
      throw std::invalid_argument("topic name must not be empty");
    }
    const auto key = std::make_pair(std::type_index(typeid(MsgT)), topic);
    auto it = subscriptions_.find(key);
    if (it == subscriptions_.end()) {
      it = subscriptions_.emplace(key, std::make_shared<Topic<MsgT>>(*node_, topic, qos)).first;
    }
    return std::static_pointer_cast<Topic<MsgT>>(it->second);
  }

  /// Returns the shared publisher for (type, topic), creating it on first use.
  template<typename MsgT>
  typename nav2::Publisher<MsgT>::SharedPtr publisher(
    const std::string & topic, const rclcpp::QoS & qos)
  {
    if (topic.empty()) {
      throw std::invalid_argument("topic name must not be empty");
    }
    const auto key = std::make_pair(std::type_index(typeid(MsgT)), topic);
    auto it = publishers_.find(key);
    if (it == publishers_.end()) {
      it = publishers_.emplace(key, node_->create_publisher<MsgT>(topic, qos)).first;
    }
    return std::static_pointer_cast<nav2::Publisher<MsgT>>(it->second);
  }

  /// Returns the shared activity monitor for \p topic, creating it on first use. Best-effort QoS,
  /// so it matches sensor publishers as well as reliable ones.
  std::shared_ptr<TopicActivity> activity(const std::string & topic, const std::string & type);

  /// Publishes a latched status string unless it equals the last one sent on that topic.
  bool publish_status(const std::string & topic, const std::string & value);

  /// Publishes a latched std_msgs/Bool unless it equals the last one sent on that topic.
  bool publish_flag(const std::string & topic, bool value);

  /// Sends the last value of every latched status and flag topic again. Called periodically so
  /// that subscribers can tell a live orchestrator (fresh messages) from a dead one.
  void republish_latched();

private:
  using Key = std::pair<std::type_index, std::string>;

  nav2::LifecycleNode::SharedPtr node_;
  nav2::TransformBuffer::SharedPtr tf_buffer_;
  nav2::TransformListener::SharedPtr tf_listener_;
  std::map<Key, std::shared_ptr<void>> subscriptions_;
  std::map<Key, std::shared_ptr<void>> publishers_;
  std::map<std::string, std::shared_ptr<TopicActivity>> activities_;
  std::map<std::string, std::string> last_status_;
  std::map<std::string, bool> last_flag_;
};

}  // namespace kvn_orchestrator
