// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_video_streamer/video_streamer_node.hpp"

#include <gst/video/video-event.h>

#include <memory>
#include <stdexcept>
#include <utility>

namespace kvn_video_streamer
{
namespace
{
constexpr const char * kSinkName = "kvn_sink";
}  // namespace

VideoStreamerNode::VideoStreamerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("video_streamer", options)
{
  gst_init(nullptr, nullptr);

  config_.source_pipeline =
    declare_parameter<std::string>("source_pipeline", config_.source_pipeline);
  config_.width = static_cast<int>(declare_parameter<int64_t>("width", config_.width));
  config_.height = static_cast<int>(declare_parameter<int64_t>("height", config_.height));
  config_.framerate = static_cast<int>(declare_parameter<int64_t>("framerate", config_.framerate));
  config_.bitrate_kbps =
    static_cast<int>(declare_parameter<int64_t>("bitrate_kbps", config_.bitrate_kbps));
  config_.keyframe_interval_s =
    declare_parameter<double>("keyframe_interval_s", config_.keyframe_interval_s);
  config_.encoder = declare_parameter<std::string>("encoder", config_.encoder);
  config_.raw_format = declare_parameter<std::string>("raw_format", config_.raw_format);
  config_.x264_profile = declare_parameter<std::string>("x264_profile", config_.x264_profile);
  frame_id_ = declare_parameter<std::string>("frame_id", "camera");
  const auto topic = declare_parameter<std::string>("topic", "/video/compressed");
  const auto qos_depth = declare_parameter<int64_t>("qos_depth", 5);
  const auto poll_period_ms = declare_parameter<int64_t>("poll_period_ms", 200);
  retry_delay_ = std::chrono::milliseconds(declare_parameter<int64_t>("retry_delay_ms", 2000));

  if (const auto error = validate(config_))
  {
    throw std::invalid_argument("kvn_video_streamer: " + *error);
  }
  if (config_.keyframe_interval_s > 1.0)
  {
    RCLCPP_WARN(
      get_logger(), "keyframe_interval_s=%.2f > 1 s: reconnecting viewers resume slowly",
      config_.keyframe_interval_s);
  }
  pipeline_description_ = build_pipeline_description(config_, kSinkName);
  RCLCPP_INFO(get_logger(), "pipeline: %s", pipeline_description_.c_str());

  publisher_ = create_publisher<foxglove_msgs::msg::CompressedVideo>(
    topic, rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(qos_depth))).reliable());
  poll_timer_ =
    create_wall_timer(std::chrono::milliseconds(poll_period_ms), [this]() { on_poll(); });
}

VideoStreamerNode::~VideoStreamerNode() { stop_pipeline(); }

void VideoStreamerNode::on_poll()
{
  if (pipeline_ != nullptr && !check_bus())
  {
    stop_pipeline();
    retry_after_ = std::chrono::steady_clock::now() + retry_delay_;
    last_subscriber_count_ = 0;
  }

  const std::size_t count = publisher_->get_subscription_count();
  if (count == 0)
  {
    if (pipeline_ != nullptr)
    {
      RCLCPP_INFO(get_logger(), "no subscribers: stopping pipeline");
      stop_pipeline();
    }
  }
  else if (pipeline_ == nullptr)
  {
    if (std::chrono::steady_clock::now() >= retry_after_)
    {
      RCLCPP_INFO(get_logger(), "%zu subscriber(s): starting pipeline", count);
      if (!start_pipeline())
      {
        retry_after_ = std::chrono::steady_clock::now() + retry_delay_;
      }
    }
  }
  else if (count > last_subscriber_count_)
  {
    // A viewer joined a running stream: give it a key frame now instead of
    // making it wait for the next periodic one.
    request_keyframe();
  }
  last_subscriber_count_ = pipeline_ != nullptr ? count : 0;
}

bool VideoStreamerNode::start_pipeline()
{
  GError * error = nullptr;
  GstElement * pipeline = gst_parse_launch(pipeline_description_.c_str(), &error);
  if (error != nullptr)
  {
    RCLCPP_ERROR(get_logger(), "gst_parse_launch failed: %s", error->message);
    g_error_free(error);
    if (pipeline != nullptr)
    {
      gst_object_unref(pipeline);
    }
    return false;
  }
  GstElement * sink = gst_bin_get_by_name(GST_BIN(pipeline), kSinkName);
  if (sink == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "appsink '%s' not found in pipeline", kSinkName);
    gst_object_unref(pipeline);
    return false;
  }

  GstAppSinkCallbacks callbacks{};
  callbacks.new_sample = &VideoStreamerNode::on_new_sample;
  gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);

  pipeline_ = pipeline;
  appsink_ = sink;
  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
  {
    RCLCPP_ERROR(get_logger(), "pipeline failed to start (source device busy or missing?)");
    check_bus();  // log the error message, if any
    stop_pipeline();
    return false;
  }
  streaming_.store(true);
  return true;
}

void VideoStreamerNode::stop_pipeline()
{
  if (pipeline_ == nullptr)
  {
    return;
  }
  // Blocks until the streaming threads stopped: no sample callback after this.
  gst_element_set_state(pipeline_, GST_STATE_NULL);
  gst_object_unref(appsink_);
  gst_object_unref(pipeline_);
  appsink_ = nullptr;
  pipeline_ = nullptr;
  streaming_.store(false);
}

void VideoStreamerNode::request_keyframe()
{
  if (appsink_ == nullptr)
  {
    return;
  }
  // Upstream event from the sink: h264parse forwards it to the encoder, which
  // emits an IDR; all_headers=TRUE makes SPS/PPS precede it.
  GstEvent * event = gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0);
  if (!gst_element_send_event(appsink_, event))
  {
    RCLCPP_DEBUG(get_logger(), "force-key-unit event not handled");
  }
  else
  {
    RCLCPP_INFO(get_logger(), "new subscriber: key frame requested");
  }
}

bool VideoStreamerNode::check_bus()
{
  GstBus * bus = gst_element_get_bus(pipeline_);
  bool healthy = true;
  while (
    GstMessage * msg = gst_bus_pop_filtered(
      bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_WARNING)))
  {
    switch (GST_MESSAGE_TYPE(msg))
    {
      case GST_MESSAGE_ERROR:
      {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(
          get_logger(), "pipeline error from %s: %s (%s)", GST_OBJECT_NAME(msg->src), err->message,
          debug != nullptr ? debug : "");
        g_clear_error(&err);
        g_free(debug);
        healthy = false;
        break;
      }
      case GST_MESSAGE_WARNING:
      {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_warning(msg, &err, &debug);
        RCLCPP_WARN(
          get_logger(), "pipeline warning from %s: %s", GST_OBJECT_NAME(msg->src), err->message);
        g_clear_error(&err);
        g_free(debug);
        break;
      }
      case GST_MESSAGE_EOS:
        RCLCPP_ERROR(get_logger(), "pipeline reached end of stream");
        healthy = false;
        break;
      default:
        break;
    }
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
  return healthy;
}

GstFlowReturn VideoStreamerNode::on_new_sample(GstAppSink * sink, gpointer user_data)
{
  return static_cast<VideoStreamerNode *>(user_data)->handle_sample(sink);
}

GstFlowReturn VideoStreamerNode::handle_sample(GstAppSink * sink)
{
  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (sample == nullptr)
  {
    return GST_FLOW_EOS;
  }
  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstMapInfo map{};
  if (buffer != nullptr && gst_buffer_map(buffer, &map, GST_MAP_READ))
  {
    auto msg = std::make_unique<foxglove_msgs::msg::CompressedVideo>();
    msg->timestamp = now();
    msg->frame_id = frame_id_;
    msg->format = "h264";
    msg->data.assign(map.data, map.data + map.size);
    gst_buffer_unmap(buffer, &map);
    publisher_->publish(std::move(msg));
    frames_published_.fetch_add(1);
  }
  gst_sample_unref(sample);
  return GST_FLOW_OK;
}

}  // namespace kvn_video_streamer
