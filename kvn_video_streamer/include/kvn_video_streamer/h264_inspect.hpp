// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
//
// Pure (ROS-free, GStreamer-free) helpers to inspect an H.264 Annex B byte
// stream: split it into NAL units, read NAL types, and parse the slice type
// from the slice header to detect B slices and IDR access units.

#ifndef KVN_VIDEO_STREAMER__H264_INSPECT_HPP_
#define KVN_VIDEO_STREAMER__H264_INSPECT_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace kvn_video_streamer::h264
{

// NAL unit types used here (ITU-T H.264 Table 7-1).
enum class NalType : uint8_t
{
  kSliceNonIdr = 1,
  kSliceDataA = 2,
  kSliceIdr = 5,
  kSei = 6,
  kSps = 7,
  kPps = 8,
  kAud = 9,
};

// slice_type % 5 (ITU-T H.264 Table 7-6).
enum class SliceType : uint8_t
{
  kP = 0,
  kB = 1,
  kI = 2,
  kSp = 3,
  kSi = 4,
};

struct NalUnit
{
  std::size_t offset;           // first byte of the NAL header (after the start code)
  std::size_t size;             // bytes from the header to the next start code / end
  std::size_t start_code_size;  // 3 or 4
  uint8_t type;                 // nal_unit_type (5 bits)
};

// True if `data` begins with a 3-byte (00 00 01) or 4-byte (00 00 00 01) start code.
bool starts_with_start_code(std::span<const uint8_t> data);

// Splits an Annex B byte stream into NAL units. Bytes before the first start
// code are ignored. Trailing zero bytes of a NAL (trailing_zero_8bits or the
// leading zero of a following 4-byte start code) are not part of its size.
std::vector<NalUnit> split_nal_units(std::span<const uint8_t> data);

// Removes emulation prevention bytes (00 00 03 -> 00 00) from a NAL payload.
// At most `max_out` output bytes are produced (0 = no limit).
std::vector<uint8_t> unescape_rbsp(std::span<const uint8_t> nal, std::size_t max_out = 0);

// Reads unsigned Exp-Golomb codes MSB-first from an RBSP buffer.
class BitReader
{
public:
  explicit BitReader(std::span<const uint8_t> data);
  std::optional<uint32_t> read_bit();
  std::optional<uint32_t> read_ue();

private:
  std::span<const uint8_t> data_;
  std::size_t bit_pos_{0};
};

// For a slice NAL (type 1, 2 or 5), including its 1-byte header, returns
// slice_type % 5. Returns nullopt for non-slice NALs or truncated headers.
std::optional<SliceType> parse_slice_type(std::span<const uint8_t> nal);

struct AccessUnitInfo
{
  bool starts_with_start_code{false};
  bool has_sps{false};
  bool has_pps{false};
  bool has_idr{false};
  bool has_b_slice{false};
  bool sps_pps_before_idr{false};  // SPS and PPS both appear before the first IDR slice
  std::vector<uint8_t> nal_types;
};

// Inspects one access unit (e.g. one CompressedVideo message).
AccessUnitInfo inspect_access_unit(std::span<const uint8_t> data);

}  // namespace kvn_video_streamer::h264

#endif  // KVN_VIDEO_STREAMER__H264_INSPECT_HPP_
