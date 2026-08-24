// Copyright (c) 2024, Alessio Morale
// Licensed under the MIT License.

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoyToTwistNode : public rclcpp::Node
{
public:
  explicit JoyToTwistNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("joy_to_twist", options)
  {
    // Declare parameters
    declare_parameter<double>("max_linear_vel", 1.0);
    declare_parameter<double>("max_angular_vel", 2.0);
    declare_parameter<int>("linear_axis", 1);      // Left analog stick Y-axis (forward/backward)
    declare_parameter<int>("angular_axis", 3);     // Right analog stick X-axis (rotation)
    declare_parameter<bool>("linear_inverted", false);
    declare_parameter<bool>("angular_inverted", false);

    // Get parameters
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();
    linear_axis_ = get_parameter("linear_axis").as_int();
    angular_axis_ = get_parameter("angular_axis").as_int();
    linear_inverted_ = get_parameter("linear_inverted").as_bool();
    angular_inverted_ = get_parameter("angular_inverted").as_bool();

    RCLCPP_INFO(get_logger(),
      "Joy to Twist: linear_axis=%d (max=%.2f m/s), angular_axis=%d (max=%.2f rad/s)",
      linear_axis_, max_linear_vel_, angular_axis_, max_angular_vel_);

    // Subscribe to joy messages
    joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
      "joy", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) { on_joy(msg); });

    // Publisher for cmd_vel
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  void on_joy(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    // Create Twist message
    auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();

    // Extract linear and angular velocities from the joystick axes
    if (linear_axis_ >= 0 && linear_axis_ < static_cast<int>(joy_msg->axes.size())) {
      double linear_input = joy_msg->axes[linear_axis_];
      if (linear_inverted_) {
        linear_input = -linear_input;
      }
      twist_msg->linear.x = linear_input * max_linear_vel_;
    }

    if (angular_axis_ >= 0 && angular_axis_ < static_cast<int>(joy_msg->axes.size())) {
      double angular_input = joy_msg->axes[angular_axis_];
      if (angular_inverted_) {
        angular_input = -angular_input;
      }
      twist_msg->angular.z = angular_input * max_angular_vel_;
    }

    // Publish the Twist message
    twist_publisher_->publish(*twist_msg);
  }

  double max_linear_vel_;
  double max_angular_vel_;
  int linear_axis_;
  int angular_axis_;
  bool linear_inverted_;
  bool angular_inverted_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JoyToTwistNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
