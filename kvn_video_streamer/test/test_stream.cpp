// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Integration test: runs the node with videotestsrc + x264enc, subscribes, and
// checks the stream properties Lichtblick relies on.

#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "foxglove_msgs/msg/compressed_video.hpp"
#include "kvn_video_streamer/h264_inspect.hpp"
#include "kvn_video_streamer/video_streamer_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <gst/gst.h>

using namespace std::chrono_literals;
using foxglove_msgs::msg::CompressedVideo;
using kvn_video_streamer::VideoStreamerNode;
namespace h264 = kvn_video_streamer::h264;

namespace
{
constexpr int kFps = 15;

struct Frame
{
  double stamp_s;
  std::string format;
  std::string frame_id;
  h264::AccessUnitInfo info;
};

class Collector
{
public:
  Collector(const rclcpp::Node::SharedPtr & node, const std::string & topic)
  {
    sub_ = node->create_subscription<CompressedVideo>(
      topic, rclcpp::QoS(10).reliable(),
      [this](CompressedVideo::ConstSharedPtr msg)
      {
        frames.push_back(Frame{
          rclcpp::Time(msg->timestamp).seconds(), msg->format, msg->frame_id,
          h264::inspect_access_unit(msg->data)});
      });
  }
  std::vector<Frame> frames;

private:
  rclcpp::Subscription<CompressedVideo>::SharedPtr sub_;
};

class StreamTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    topic_ = "/test_video_" + std::to_string(getpid()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name();
    client_ = std::make_shared<rclcpp::Node>("video_test_client");
    executor_.add_node(client_);
  }

  void TearDown() override
  {
    if (streamer_)
    {
      executor_.remove_node(streamer_);
    }
    executor_.remove_node(client_);
  }

  void start_streamer(double keyframe_interval_s, bool webrtc = false)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      {"webrtc.enabled", webrtc},
      {"webrtc.stun_server", "stun://127.0.0.1:3478"},
      {"source_pipeline", "videotestsrc is-live=true pattern=ball"},
      {"width", 640},
      {"height", 480},
      {"framerate", kFps},
      {"bitrate_kbps", 1000},
      {"keyframe_interval_s", keyframe_interval_s},
      {"frame_id", "test_cam"},
      {"topic", topic_},
      {"poll_period_ms", 100},
    });
    streamer_ = std::make_shared<VideoStreamerNode>(options);
    executor_.add_node(streamer_);
  }

  // Spins until pred() is true or the timeout elapses; returns pred().
  bool spin_until(const std::function<bool()> & pred, std::chrono::milliseconds timeout)
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!pred() && std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some(10ms);
      std::this_thread::sleep_for(5ms);
    }
    return pred();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    spin_until([]() { return false; }, duration);
  }

  std::string topic_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Node::SharedPtr client_;
  std::shared_ptr<VideoStreamerNode> streamer_;
};

}  // namespace

TEST_F(StreamTest, StreamPropertiesAndOnDemand)
{
  start_streamer(1.0);

  // No subscriber: nothing is encoded.
  spin_for(600ms);
  EXPECT_FALSE(streamer_->is_streaming());
  EXPECT_EQ(streamer_->frames_published(), 0U);

  auto collector = std::make_unique<Collector>(client_, topic_);
  ASSERT_TRUE(spin_until([&]() { return !collector->frames.empty(); }, 15s))
    << "no frame received; pipeline: " << streamer_->pipeline_description();
  spin_for(3000ms);
  const std::vector<Frame> frames = collector->frames;
  ASSERT_GE(frames.size(), static_cast<std::size_t>(kFps * 2));

  // The stream starts with a decodable key frame.
  EXPECT_TRUE(frames.front().info.has_idr) << "first frame is not an IDR";

  std::vector<std::size_t> idr_indices;
  for (std::size_t i = 0; i < frames.size(); ++i)
  {
    const Frame & f = frames[i];
    EXPECT_EQ(f.format, "h264");
    EXPECT_EQ(f.frame_id, "test_cam");
    EXPECT_TRUE(f.info.starts_with_start_code) << "frame " << i << " is not Annex B";
    EXPECT_FALSE(f.info.has_b_slice) << "frame " << i << " has a B slice";
    if (f.info.has_idr)
    {
      idr_indices.push_back(i);
      EXPECT_TRUE(f.info.sps_pps_before_idr) << "IDR frame " << i << " lacks SPS/PPS before it";
    }
  }

  // At least 3 key frames in ~3 s, each at most 1 s (15 frames) apart.
  ASSERT_GE(idr_indices.size(), 3U);
  for (std::size_t k = 1; k < idr_indices.size(); ++k)
  {
    const std::size_t gap_frames = idr_indices[k] - idr_indices[k - 1];
    const double gap_s = frames[idr_indices[k]].stamp_s - frames[idr_indices[k - 1]].stamp_s;
    EXPECT_LE(gap_frames, static_cast<std::size_t>(kFps)) << "key frame gap in frames";
    EXPECT_LE(gap_s, 1.0 + 0.15) << "key frame gap in seconds";
  }
  // No long tail without a key frame either.
  EXPECT_LE(frames.size() - 1 - idr_indices.back(), static_cast<std::size_t>(kFps));

  // Last subscriber gone: the pipeline stops.
  collector.reset();
  EXPECT_TRUE(spin_until([&]() { return !streamer_->is_streaming(); }, 3s));
}

TEST_F(StreamTest, NewSubscriberGetsKeyFrameQuickly)
{
  // Long natural GOP (4 s) so only the forced key frame can arrive quickly.
  start_streamer(4.0);
  Collector first(client_, topic_);
  ASSERT_TRUE(spin_until([&]() { return !first.frames.empty(); }, 15s));
  spin_for(1500ms);
  ASSERT_TRUE(streamer_->is_streaming());

  Collector second(client_, topic_);
  ASSERT_TRUE(spin_until([&]() { return !second.frames.empty(); }, 5s));
  const double first_stamp = second.frames.front().stamp_s;
  ASSERT_TRUE(spin_until(
    [&]()
    {
      for (const auto & f : second.frames)
      {
        if (f.info.has_idr)
        {
          return true;
        }
      }
      return false;
    },
    2s))
    << "no key frame for the new subscriber";
  for (const auto & f : second.frames)
  {
    if (f.info.has_idr)
    {
      EXPECT_LE(f.stamp_s - first_stamp, 0.7) << "forced key frame came too late";
      EXPECT_TRUE(f.info.sps_pps_before_idr);
      break;
    }
  }
}

TEST_F(StreamTest, BrokenSourceDoesNotCrash)
{
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"source_pipeline", "v4l2src device=/dev/does_not_exist"},
    {"topic", topic_},
    {"poll_period_ms", 100},
    {"retry_delay_ms", 300},
  });
  streamer_ = std::make_shared<VideoStreamerNode>(options);
  executor_.add_node(streamer_);
  Collector collector(client_, topic_);
  spin_for(1500ms);
  EXPECT_TRUE(collector.frames.empty());
  EXPECT_EQ(streamer_->frames_published(), 0U);
}

namespace
{
bool webrtcsink_available()
{
  gst_init(nullptr, nullptr);
  GstElementFactory * factory = gst_element_factory_find("webrtcsink");
  if (factory == nullptr)
  {
    return false;
  }
  gst_object_unref(factory);
  return true;
}
}  // namespace

// With the WebRTC branch on, the pipeline stays up without subscribers but the Foxglove branch
// is closed; it opens (with a key frame first) when a subscriber appears and closes again.
TEST_F(StreamTest, WebrtcBranchGatesFoxgloveStream)
{
  if (!webrtcsink_available())
  {
    GTEST_SKIP() << "gst-plugins-rs webrtcsink not installed";
  }
  start_streamer(1.0, true);
  ASSERT_TRUE(spin_until([&]() { return streamer_->is_streaming(); }, 10s))
    << streamer_->pipeline_description();
  spin_for(800ms);
  EXPECT_TRUE(streamer_->is_streaming());
  EXPECT_EQ(streamer_->frames_published(), 0U) << "Foxglove branch must stay closed";

  auto collector = std::make_unique<Collector>(client_, topic_);
  ASSERT_TRUE(spin_until([&]() { return collector->frames.size() >= 10; }, 10s));
  EXPECT_TRUE(collector->frames.front().info.has_idr);
  EXPECT_TRUE(collector->frames.front().info.starts_with_start_code);

  collector.reset();
  spin_for(600ms);
  const uint64_t after_close = streamer_->frames_published();
  spin_for(800ms);
  EXPECT_EQ(streamer_->frames_published(), after_close) << "branch did not close";
  EXPECT_TRUE(streamer_->is_streaming()) << "pipeline must stay up for WebRTC viewers";
}

// No signalling server: webrtcsink fails, but the Foxglove stream must still work (on demand),
// and the node must keep running instead of crash-looping the pipeline.
TEST_F(StreamTest, WebrtcFailureFallsBackToFoxglove)
{
  if (!webrtcsink_available())
  {
    GTEST_SKIP() << "gst-plugins-rs webrtcsink not installed";
  }
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"source_pipeline", "videotestsrc is-live=true pattern=ball"},
    {"topic", topic_},
    {"poll_period_ms", 100},
    {"retry_delay_ms", 300},
    {"webrtc.enabled", true},
    {"webrtc.signaller_uri", "ws://127.0.0.1:1"},
    {"webrtc.stun_server", "stun://127.0.0.1:3478"},
    {"webrtc.retry_s", 3},
  });
  streamer_ = std::make_shared<VideoStreamerNode>(options);
  executor_.add_node(streamer_);

  // Let the first WebRTC attempt fail, then subscribe.
  spin_for(1500ms);
  Collector collector(client_, topic_);
  ASSERT_TRUE(spin_until([&]() { return collector.frames.size() >= 15; }, 10s))
    << "Foxglove stream did not come up after the WebRTC failure";
  EXPECT_TRUE(collector.frames.front().info.has_idr);

  // While a Foxglove viewer is connected the pipeline is not restarted to retry WebRTC.
  const std::size_t before = collector.frames.size();
  spin_for(4s);
  EXPECT_GE(collector.frames.size(), before + static_cast<std::size_t>(kFps * 3));
}
