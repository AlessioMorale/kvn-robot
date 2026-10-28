// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <memory>

#include "kvn_display/display_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kvn_display::DisplayNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
