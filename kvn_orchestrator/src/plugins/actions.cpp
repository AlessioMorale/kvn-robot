// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Action plugins that act on the robot: status/mode reporting, topic publishing and TF capture.
// Navigation is not implemented here: the tree uses Nav2's own NavigateToPose BT plugin.

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>

#include "kvn_orchestrator/bt_utils.hpp"

namespace kvn_orchestrator
{

/// Publishes a latched std_msgs/String status; repeats of the same value are not re-sent.
class SetStatus : public BT::SyncActionNode, public Preparable
{
public:
  SetStatus(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "/orchestrator/status", "std_msgs/String topic"),
      BT::InputPort<std::string>("value", "Status text"),
    };
  }

  void prepare() override
  {
    // Creates the latched publisher now so it is connected before the first status is sent.
    const auto ctx = get_context(config());
    ctx->publisher<std_msgs::msg::String>(
      require_input<std::string>(*this, "topic"), nav2::qos::LatchedPublisherQoS());
  }

  BT::NodeStatus tick() override
  {
    get_context(config())->publish_status(
      require_input<std::string>(*this, "topic"), require_input<std::string>(*this, "value"));
    return BT::NodeStatus::SUCCESS;
  }
};

/// Publishes a latched std_msgs/Bool; repeats of the same value are not re-sent. The host
/// re-sends it every heartbeat, so subscribers can require it to be fresh (e.g. the teleop node's
/// autonomy permit).
class SetFlag : public BT::SyncActionNode, public Preparable
{
public:
  SetFlag(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "std_msgs/Bool topic"),
      BT::InputPort<bool>("value", "Value to publish"),
    };
  }

  void prepare() override
  {
    get_context(config())->publisher<std_msgs::msg::Bool>(
      require_input<std::string>(*this, "topic"), nav2::qos::LatchedPublisherQoS());
  }

  BT::NodeStatus tick() override
  {
    get_context(config())->publish_flag(
      require_input<std::string>(*this, "topic"), require_input<bool>(*this, "value"));
    return BT::NodeStatus::SUCCESS;
  }
};

/// Publishes one std_msgs/Bool.
class PublishBool : public BT::SyncActionNode, public Preparable
{
public:
  PublishBool(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic", "std_msgs/Bool topic"),
      BT::InputPort<bool>("value", "Value to publish"),
    };
  }

  void prepare() override
  {
    get_context(config())->publisher<std_msgs::msg::Bool>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS());
  }

  BT::NodeStatus tick() override
  {
    std_msgs::msg::Bool message;
    message.data = require_input<bool>(*this, "value");
    get_context(config())->publisher<std_msgs::msg::Bool>(
      require_input<std::string>(*this, "topic"), nav2::qos::StandardTopicQoS())->publish(message);
    return BT::NodeStatus::SUCCESS;
  }
};

/// Looks up child_frame in parent_frame and stores it as a PoseStamped on the blackboard.
/// FAILURE while the transform is not available.
class CaptureRobotPose : public BT::SyncActionNode
{
public:
  CaptureRobotPose(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("parent_frame", "map", "Frame the pose is expressed in"),
      BT::InputPort<std::string>("child_frame", "base_link", "Frame whose pose is captured"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Captured pose"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto ctx = get_context(config());
    const auto parent = require_input<std::string>(*this, "parent_frame");
    const auto child = require_input<std::string>(*this, "child_frame");
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = ctx->tf_buffer()->lookupTransform(parent, child, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = parent;
    pose.header.stamp = ctx->node()->get_clock()->now();
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;
    setOutput("pose", pose);
    RCLCPP_INFO(
      ctx->logger(), "Captured pose of %s in %s: x=%.2f y=%.2f", child.c_str(), parent.c_str(),
      pose.pose.position.x, pose.pose.position.y);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace kvn_orchestrator

BT_REGISTER_NODES(factory)
{
  using namespace kvn_orchestrator;  // NOLINT
  factory.registerNodeType<SetStatus>("SetStatus");
  factory.registerNodeType<PublishBool>("PublishBool");
  factory.registerNodeType<SetFlag>("SetFlag");
  factory.registerNodeType<CaptureRobotPose>("CaptureRobotPose");
}
