// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <string>

#include "kvn_video_streamer/pipeline_config.hpp"

using kvn_video_streamer::build_encoder_description;
using kvn_video_streamer::build_pipeline_description;
using kvn_video_streamer::keyframe_interval_frames;
using kvn_video_streamer::StreamConfig;
using kvn_video_streamer::validate;
using kvn_video_streamer::build_webrtc_description;

namespace
{
bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
}
}  // namespace

TEST(PipelineConfig, KeyframeInterval)
{
  StreamConfig c;
  EXPECT_EQ(keyframe_interval_frames(c), 15);
  c.keyframe_interval_s = 0.5;
  EXPECT_EQ(keyframe_interval_frames(c), 8);
  c.keyframe_interval_s = 0.01;
  EXPECT_EQ(keyframe_interval_frames(c), 1);
}

TEST(PipelineConfig, DefaultX264Encoder)
{
  StreamConfig c;
  c.bitrate_kbps = 1500;
  const std::string enc = build_encoder_description(c);
  EXPECT_TRUE(contains(enc, "x264enc "));
  EXPECT_TRUE(contains(enc, "tune=zerolatency"));
  EXPECT_TRUE(contains(enc, "speed-preset=ultrafast"));
  EXPECT_TRUE(contains(enc, "bframes=0"));
  EXPECT_TRUE(contains(enc, "key-int-max=15"));
  EXPECT_TRUE(contains(enc, "bitrate=1500"));
  EXPECT_TRUE(contains(enc, "profile=baseline"));
}

TEST(PipelineConfig, CustomEncoderPlaceholders)
{
  StreamConfig c;
  c.encoder =
    "v4l2h264enc extra-controls=\"c,video_bitrate={bitrate_bps},h264_i_frame_period={keyint}\"";
  c.bitrate_kbps = 800;
  const std::string enc = build_encoder_description(c);
  EXPECT_EQ(enc, "v4l2h264enc extra-controls=\"c,video_bitrate=800000,h264_i_frame_period=15\"");
}

TEST(PipelineConfig, FullPipeline)
{
  StreamConfig c;
  c.source_pipeline = "videotestsrc is-live=true";
  c.width = 320;
  c.height = 240;
  c.framerate = 10;
  const std::string p = build_pipeline_description(c, "sink");
  EXPECT_EQ(p.rfind("videotestsrc is-live=true ! videoscale", 0), 0U);
  EXPECT_TRUE(contains(p, "videorate"));
  EXPECT_TRUE(contains(p, "format=I420,width=320,height=240"));
  EXPECT_TRUE(contains(p, "framerate=10/1"));
  EXPECT_TRUE(contains(p, "key-int-max=10"));
  EXPECT_TRUE(contains(p, "h264parse config-interval=-1"));
  EXPECT_TRUE(contains(p, "video/x-h264,stream-format=byte-stream,alignment=au"));
  EXPECT_TRUE(contains(p, "appsink name=sink"));
  // Order: scale before encoder before parser before sink.
  EXPECT_LT(p.find("videoscale"), p.find("x264enc"));
  EXPECT_LT(p.find("x264enc"), p.find("h264parse"));
  EXPECT_LT(p.find("h264parse"), p.find("appsink"));

  c.raw_format.clear();
  EXPECT_FALSE(contains(build_pipeline_description(c, "sink"), "x-raw,format="));
}

TEST(PipelineConfig, Validation)
{
  StreamConfig c;
  EXPECT_FALSE(validate(c).has_value());
  c.width = 641;
  EXPECT_TRUE(validate(c).has_value());
  c = StreamConfig{};
  c.framerate = 0;
  EXPECT_TRUE(validate(c).has_value());
  c = StreamConfig{};
  c.bitrate_kbps = 0;
  EXPECT_TRUE(validate(c).has_value());
  c = StreamConfig{};
  c.keyframe_interval_s = 0.0;
  EXPECT_TRUE(validate(c).has_value());
  c = StreamConfig{};
  c.source_pipeline.clear();
  EXPECT_TRUE(validate(c).has_value());
}

TEST(PipelineConfig, WebrtcDisabledByDefault)
{
  StreamConfig c;
  const std::string p = build_pipeline_description(c, "sink");
  EXPECT_FALSE(contains(p, "webrtcsink"));
  EXPECT_FALSE(contains(p, "tee"));
  EXPECT_FALSE(contains(p, "valve"));
}

TEST(PipelineConfig, WebrtcBranch)
{
  StreamConfig c;
  c.webrtc.enabled = true;
  c.webrtc.stun_server = "stun://127.0.0.1:3478";
  const std::string p = build_pipeline_description(c, "sink");
  // Raw video is split before the encoder; the Foxglove branch sits behind a closed valve.
  EXPECT_LT(p.find("videoscale"), p.find("tee name=kvn_tee"));
  EXPECT_LT(p.find("tee name=kvn_tee"), p.find("valve name=kvn_valve drop=true"));
  EXPECT_LT(p.find("valve name=kvn_valve"), p.find("x264enc"));
  EXPECT_LT(p.find("appsink name=sink"), p.find("webrtcsink name=kvn_rtc"));
  EXPECT_TRUE(contains(p, "video-caps=\"video/x-h264\""));
  EXPECT_TRUE(contains(p, "congestion-control=homegrown"));
  EXPECT_TRUE(contains(p, "start-bitrate=1000000 min-bitrate=200000 max-bitrate=2000000"));
  EXPECT_TRUE(contains(p, "signaller::uri=ws://127.0.0.1:8443"));
  EXPECT_TRUE(contains(p, "stun-server=stun://127.0.0.1:3478"));

  c.webrtc.stun_server.clear();
  EXPECT_FALSE(contains(build_pipeline_description(c, "sink"), "stun-server"));
  EXPECT_TRUE(contains(build_webrtc_description(c.webrtc, "x"), "webrtcsink name=x"));
}

TEST(PipelineConfig, WebrtcValidation)
{
  StreamConfig c;
  c.webrtc.min_bitrate_kbps = 5000;  // ignored while WebRTC is off
  EXPECT_FALSE(validate(c).has_value());
  c.webrtc.enabled = true;
  EXPECT_TRUE(validate(c).has_value());  // min > start
  c.webrtc = kvn_video_streamer::WebrtcConfig{};
  c.webrtc.enabled = true;
  EXPECT_FALSE(validate(c).has_value());
  c.webrtc.signaller_uri.clear();
  EXPECT_TRUE(validate(c).has_value());
}
