// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_video_streamer/pipeline_config.hpp"

#include <cmath>
#include <sstream>

namespace kvn_video_streamer
{
namespace
{
void replace_all(std::string & text, const std::string & from, const std::string & to)
{
  std::size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos)
  {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
}
}  // namespace

int keyframe_interval_frames(const StreamConfig & config)
{
  const double frames =
    std::round(static_cast<double>(config.framerate) * config.keyframe_interval_s);
  return frames < 1.0 ? 1 : static_cast<int>(frames);
}

std::optional<std::string> validate(const StreamConfig & config)
{
  if (config.source_pipeline.empty())
  {
    return "source_pipeline must not be empty";
  }
  if (config.width <= 0 || config.height <= 0)
  {
    return "width and height must be > 0";
  }
  if (config.width % 2 != 0 || config.height % 2 != 0)
  {
    return "width and height must be even (4:2:0 chroma)";
  }
  if (config.framerate <= 0 || config.framerate > 120)
  {
    return "framerate must be in 1..120";
  }
  if (config.bitrate_kbps <= 0)
  {
    return "bitrate_kbps must be > 0";
  }
  if (!(config.keyframe_interval_s > 0.0))
  {
    return "keyframe_interval_s must be > 0";
  }
  return std::nullopt;
}

std::string build_encoder_description(const StreamConfig & config)
{
  const int keyint = keyframe_interval_frames(config);
  if (!config.encoder.empty())
  {
    std::string enc = config.encoder;
    replace_all(enc, "{bitrate_kbps}", std::to_string(config.bitrate_kbps));
    replace_all(enc, "{bitrate_bps}", std::to_string(config.bitrate_kbps * 1000));
    replace_all(enc, "{keyint}", std::to_string(keyint));
    return enc;
  }
  std::ostringstream out;
  out << "x264enc tune=zerolatency speed-preset=ultrafast bframes=0 b-adapt=false"
      << " key-int-max=" << keyint << " bitrate=" << config.bitrate_kbps
      << " vbv-buf-capacity=" << 1000 / config.framerate * 2 << " byte-stream=true aud=true";
  if (!config.x264_profile.empty())
  {
    out << " ! video/x-h264,profile=" << config.x264_profile;
  }
  return out.str();
}

std::string build_pipeline_description(const StreamConfig & config, const std::string & sink_name)
{
  std::ostringstream out;
  out << config.source_pipeline << " ! videoscale ! videorate drop-only=true ! videoconvert"
      << " ! video/x-raw";
  if (!config.raw_format.empty())
  {
    out << ",format=" << config.raw_format;
  }
  out << ",width=" << config.width << ",height=" << config.height
      << ",pixel-aspect-ratio=1/1,framerate=" << config.framerate << "/1"
      << " ! queue max-size-buffers=2 leaky=downstream"
      << " ! " << build_encoder_description(config) << " ! h264parse config-interval=-1"
      << " ! video/x-h264,stream-format=byte-stream,alignment=au"
      << " ! appsink name=" << sink_name << " sync=false max-buffers=4 drop=true";
  return out.str();
}

}  // namespace kvn_video_streamer
