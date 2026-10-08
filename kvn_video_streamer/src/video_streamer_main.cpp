// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <memory>

#include "kvn_video_streamer/video_streamer_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kvn_video_streamer::VideoStreamerNode>());
  rclcpp::shutdown();
  return 0;
}
