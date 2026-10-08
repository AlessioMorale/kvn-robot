// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Publishes on-demand H.264 (Annex B) as foxglove_msgs/CompressedVideo.
// The GStreamer pipeline runs only while the topic has subscribers.

#ifndef KVN_VIDEO_STREAMER__VIDEO_STREAMER_NODE_HPP_
#define KVN_VIDEO_STREAMER__VIDEO_STREAMER_NODE_HPP_

#include <gst/app/gstappsink.h>
#include <gst/gst.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "foxglove_msgs/msg/compressed_video.hpp"
#include "kvn_video_streamer/pipeline_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kvn_video_streamer
{

class VideoStreamerNode : public rclcpp::Node
{
public:
  explicit VideoStreamerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~VideoStreamerNode() override;

  VideoStreamerNode(const VideoStreamerNode &) = delete;
  VideoStreamerNode & operator=(const VideoStreamerNode &) = delete;

  bool is_streaming() const { return streaming_.load(); }
  uint64_t frames_published() const { return frames_published_.load(); }
  const std::string & pipeline_description() const { return pipeline_description_; }

private:
  void on_poll();
  bool start_pipeline();
  void stop_pipeline();
  void request_keyframe();
  bool check_bus();
  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data);
  GstFlowReturn handle_sample(GstAppSink * sink);

  StreamConfig config_;
  std::string frame_id_;
  std::string pipeline_description_;
  std::chrono::milliseconds retry_delay_;

  rclcpp::Publisher<foxglove_msgs::msg::CompressedVideo>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr poll_timer_;

  GstElement * pipeline_{nullptr};
  GstElement * appsink_{nullptr};
  std::size_t last_subscriber_count_{0};
  std::chrono::steady_clock::time_point retry_after_{};
  std::atomic<bool> streaming_{false};
  std::atomic<uint64_t> frames_published_{0};
};

}  // namespace kvn_video_streamer

#endif  // KVN_VIDEO_STREAMER__VIDEO_STREAMER_NODE_HPP_
