// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Pure helpers that turn the stream settings into a gst-launch style pipeline
// description. No ROS or GStreamer dependency, so they are unit-testable.

#ifndef KVN_VIDEO_STREAMER__PIPELINE_CONFIG_HPP_
#define KVN_VIDEO_STREAMER__PIPELINE_CONFIG_HPP_

#include <optional>
#include <string>

namespace kvn_video_streamer
{

// Optional WebRTC branch (gst-plugins-rs webrtcsink). It encodes the raw video itself so that
// its congestion control can steer the encoder bitrate. Needs a signalling server.
struct WebrtcConfig
{
  bool enabled{false};
  std::string signaller_uri{"ws://127.0.0.1:8443"};
  // Empty: do not set the property, which leaves the plugin default (a public STUN server).
  std::string stun_server{};
  int start_bitrate_kbps{1000};
  int min_bitrate_kbps{200};
  int max_bitrate_kbps{2000};
};

struct StreamConfig
{
  // gst-launch fragment producing raw video (or anything videoconvert/videoscale accept).
  std::string source_pipeline{"v4l2src device=/dev/video0"};
  int width{640};
  int height{480};
  int framerate{15};
  int bitrate_kbps{1000};
  double keyframe_interval_s{1.0};
  // Empty: built-in x264enc low-latency settings. Otherwise a full gst-launch
  // encoder fragment; {bitrate_kbps}, {bitrate_bps} and {keyint} are substituted.
  std::string encoder{};
  // Raw format forced before the encoder; empty lets the encoder negotiate.
  std::string raw_format{"I420"};
  // H.264 profile requested from the built-in x264enc (empty = encoder default).
  std::string x264_profile{"baseline"};
  WebrtcConfig webrtc{};
};

inline constexpr const char * kValveName = "kvn_valve";

// Key frame interval in frames: round(framerate * keyframe_interval_s), at least 1.
int keyframe_interval_frames(const StreamConfig & config);

// Returns an error message if the configuration is unusable.
std::optional<std::string> validate(const StreamConfig & config);

// Encoder fragment, either the built-in x264enc one or the custom string with
// its placeholders substituted.
std::string build_encoder_description(const StreamConfig & config);

// webrtcsink element description (H.264 forced, homegrown congestion control).
std::string build_webrtc_description(const WebrtcConfig & webrtc, const std::string & sink_name);

// Full pipeline: source ! scale ! rate ! caps ! encoder ! h264parse ! caps ! appsink.
// With webrtc.enabled the raw video is split by a tee: one branch is the pipeline above behind
// a valve named kValveName (closed until the Foxglove topic has subscribers), the other feeds
// webrtcsink.
std::string build_pipeline_description(const StreamConfig & config, const std::string & sink_name);

}  // namespace kvn_video_streamer

#endif  // KVN_VIDEO_STREAMER__PIPELINE_CONFIG_HPP_
