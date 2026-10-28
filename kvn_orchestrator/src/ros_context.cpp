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

std::shared_ptr<TopicActivity> RosContext::activity(
  const std::string & topic, const std::string & type)
{
  if (topic.empty() || type.empty()) {
    throw std::invalid_argument("topic name and type must not be empty");
  }
  auto it = activities_.find(topic);
  if (it == activities_.end()) {
    it = activities_.emplace(
      topic, std::make_shared<TopicActivity>(*node_, topic, type, rclcpp::SensorDataQoS())).first;
  }
  return it->second;
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

bool RosContext::publish_flag(const std::string & topic, bool value)
{
  const auto it = last_flag_.find(topic);
  if (it != last_flag_.end() && it->second == value) {
    return false;
  }
  last_flag_[topic] = value;
  std_msgs::msg::Bool message;
  message.data = value;
  publisher<std_msgs::msg::Bool>(topic, nav2::qos::LatchedPublisherQoS())->publish(message);
  return true;
}

void RosContext::republish_latched()
{
  for (const auto & [topic, value] : last_status_) {
    std_msgs::msg::String message;
    message.data = value;
    publisher<std_msgs::msg::String>(topic, nav2::qos::LatchedPublisherQoS())->publish(message);
  }
  for (const auto & [topic, value] : last_flag_) {
    std_msgs::msg::Bool message;
    message.data = value;
    publisher<std_msgs::msg::Bool>(topic, nav2::qos::LatchedPublisherQoS())->publish(message);
  }
}

}  // namespace kvn_orchestrator
