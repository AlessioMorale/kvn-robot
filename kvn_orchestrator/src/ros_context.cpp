// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_orchestrator/ros_context.hpp"

namespace kvn_orchestrator
{

RosContext::RosContext(nav2::LifecycleNode::SharedPtr node)
: node_(std::move(node))
{
  tf_buffer_ = nav2::create_transform_buffer(node_);
  tf_listener_ = nav2::create_transform_listener(*tf_buffer_, node_, false);
}

bool RosContext::publish_status(const std::string & topic, const std::string & value)
{
  const auto it = last_status_.find(topic);
  if (it != last_status_.end() && it->second == value) {
    return false;
  }
  last_status_[topic] = value;
  std_msgs::msg::String message;
  message.data = value;
  publisher<std_msgs::msg::String>(topic, nav2::qos::LatchedPublisherQoS())->publish(message);
  return true;
}

}  // namespace kvn_orchestrator
