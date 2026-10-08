// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <memory>

#include "kvn_status/status_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<kvn_status::StatusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
