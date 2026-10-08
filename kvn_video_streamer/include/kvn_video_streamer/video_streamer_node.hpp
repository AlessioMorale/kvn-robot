// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Publishes on-demand H.264 (Annex B) as foxglove_msgs/CompressedVideo.
// The GStreamer pipeline runs only while the topic has subscribers, unless the WebRTC branch is
// enabled: then the pipeline (and the camera) stays up so webrtcsink can accept viewers, the
// Foxglove branch is gated by a valve, and webrtcsink only encodes while a viewer is connected.
// A WebRTC failure (for example no signalling server) never takes the Foxglove stream down: the
// pipeline falls back to Foxglove only and retries WebRTC later.

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
  bool start_pipeline(bool with_webrtc);
  void stop_pipeline();
  void request_keyframe();
  void set_foxglove_open(bool open);
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
  GstElement * valve_{nullptr};
  bool foxglove_open_{false};
  // True while the running pipeline includes the WebRTC branch. After a WebRTC error the pipeline
  // restarts without it (Foxglove only, on demand) and WebRTC is retried after webrtc_retry_.
  bool webrtc_active_{false};
  bool webrtc_error_{false};
  std::chrono::seconds webrtc_retry_{10};
  std::chrono::steady_clock::time_point webrtc_retry_after_{};
  std::size_t last_subscriber_count_{0};
  std::chrono::steady_clock::time_point retry_after_{};
  std::atomic<bool> streaming_{false};
  std::atomic<uint64_t> frames_published_{0};
};

}  // namespace kvn_video_streamer

#endif  // KVN_VIDEO_STREAMER__VIDEO_STREAMER_NODE_HPP_
