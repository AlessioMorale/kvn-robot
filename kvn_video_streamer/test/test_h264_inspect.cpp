// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "kvn_video_streamer/h264_inspect.hpp"

using kvn_video_streamer::h264::AccessUnitInfo;
using kvn_video_streamer::h264::BitReader;
using kvn_video_streamer::h264::inspect_access_unit;
using kvn_video_streamer::h264::parse_slice_type;
using kvn_video_streamer::h264::SliceType;
using kvn_video_streamer::h264::split_nal_units;
using kvn_video_streamer::h264::starts_with_start_code;
using kvn_video_streamer::h264::unescape_rbsp;

namespace
{
// Slice header first bytes (after the NAL header), first_mb_in_slice = 0 ("1"):
//   P  (0): 1 1        -> 0xC0      B (1): 1 010     -> 0xA0
//   I  (2): 1 011      -> 0xB0      P (5): 1 00110   -> 0x98
//   B  (6): 1 00111    -> 0x9C      I (7): 1 0001000 -> 0x88
const std::vector<uint8_t> kSps{0x67, 0x42, 0xC0, 0x1E, 0xDA};
const std::vector<uint8_t> kPps{0x68, 0xCE, 0x3C, 0x80};

std::vector<uint8_t> annexb(const std::vector<std::vector<uint8_t>> & nals, bool long_code = true)
{
  std::vector<uint8_t> out;
  for (const auto & nal : nals)
  {
    if (long_code)
    {
      out.push_back(0x00);
    }
    out.insert(out.end(), {0x00, 0x00, 0x01});
    out.insert(out.end(), nal.begin(), nal.end());
  }
  return out;
}
}  // namespace

TEST(H264Inspect, StartCodes)
{
  EXPECT_TRUE(starts_with_start_code(std::vector<uint8_t>{0x00, 0x00, 0x01, 0x09}));
  EXPECT_TRUE(starts_with_start_code(std::vector<uint8_t>{0x00, 0x00, 0x00, 0x01, 0x09}));
  EXPECT_FALSE(starts_with_start_code(std::vector<uint8_t>{0x00, 0x00, 0x02, 0x09}));
  // AVCC (length-prefixed) data must not look like Annex B.
  EXPECT_FALSE(starts_with_start_code(std::vector<uint8_t>{0x00, 0x00, 0x00, 0x05, 0x65}));
  EXPECT_FALSE(starts_with_start_code(std::vector<uint8_t>{0x00, 0x01}));
  EXPECT_FALSE(starts_with_start_code(std::vector<uint8_t>{}));
}

TEST(H264Inspect, SplitMixedStartCodesAndTrailingZeros)
{
  const std::vector<uint8_t> data{0x00, 0x00, 0x00, 0x01, 0x67, 0xAA, 0xBB, 0x00, 0x00, 0x01,
                                  0x68, 0xCC, 0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x80, 0x00};
  const auto nals = split_nal_units(data);
  ASSERT_EQ(nals.size(), 3U);
  EXPECT_EQ(nals[0].type, 7);
  EXPECT_EQ(nals[0].offset, 4U);
  EXPECT_EQ(nals[0].size, 3U);
  EXPECT_EQ(nals[0].start_code_size, 4U);
  EXPECT_EQ(nals[1].type, 8);
  EXPECT_EQ(nals[1].offset, 10U);
  EXPECT_EQ(nals[1].size, 2U);
  EXPECT_EQ(nals[1].start_code_size, 3U);
  EXPECT_EQ(nals[2].type, 5);
  EXPECT_EQ(nals[2].size, 3U);
  EXPECT_EQ(nals[2].start_code_size, 4U);
}

TEST(H264Inspect, SplitIgnoresLeadingGarbageAndEmpty)
{
  EXPECT_TRUE(split_nal_units(std::vector<uint8_t>{}).empty());
  EXPECT_TRUE(split_nal_units(std::vector<uint8_t>{0x12, 0x34}).empty());
  const auto nals = split_nal_units(std::vector<uint8_t>{0x12, 0x00, 0x00, 0x01, 0x09, 0xF0});
  ASSERT_EQ(nals.size(), 1U);
  EXPECT_EQ(nals[0].type, 9);
}

TEST(H264Inspect, UnescapeRemovesEmulationPrevention)
{
  const std::vector<uint8_t> esc{0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03};
  EXPECT_EQ(unescape_rbsp(esc), (std::vector<uint8_t>{0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00}));
  EXPECT_EQ(unescape_rbsp(esc, 3).size(), 3U);
  // A 03 not preceded by two zeros is data.
  EXPECT_EQ(
    unescape_rbsp(std::vector<uint8_t>{0x00, 0x03, 0x03}),
    (std::vector<uint8_t>{0x00, 0x03, 0x03}));
}

TEST(H264Inspect, ExpGolomb)
{
  // 1 | 010 | 011 | 00100 | 00111 | 0001000 -> 0,1,2,3,6,7
  // bits: 1010 0110 0100 0011 1000 1000 -> 0xA6 0x43 0x88
  const std::vector<uint8_t> data{0xA6, 0x43, 0x88};
  BitReader reader(data);
  EXPECT_EQ(reader.read_ue(), 0U);
  EXPECT_EQ(reader.read_ue(), 1U);
  EXPECT_EQ(reader.read_ue(), 2U);
  EXPECT_EQ(reader.read_ue(), 3U);
  EXPECT_EQ(reader.read_ue(), 6U);
  EXPECT_EQ(reader.read_ue(), 7U);
  EXPECT_FALSE(reader.read_ue().has_value());  // out of bits
}

TEST(H264Inspect, ExpGolombRejectsOverlongCode)
{
  const std::vector<uint8_t> zeros(8, 0x00);
  BitReader reader(zeros);
  EXPECT_FALSE(reader.read_ue().has_value());
}

TEST(H264Inspect, SliceTypes)
{
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x41, 0xC0}), SliceType::kP);
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x41, 0xA0}), SliceType::kB);
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x65, 0xB0}), SliceType::kI);
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x41, 0x98}), SliceType::kP);
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x01, 0x9C}), SliceType::kB);
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x65, 0x88, 0x80}), SliceType::kI);
  // first_mb_in_slice = 3 ("00100"), slice_type = 1 ("010"): 0010 0010 -> 0x22
  EXPECT_EQ(parse_slice_type(std::vector<uint8_t>{0x41, 0x22}), SliceType::kB);
}

TEST(H264Inspect, SliceTypeAfterEmulationPrevention)
{
  // first_mb_in_slice with 24 leading zeros: RBSP 00 00 00 80 00 00 28
  // escaped as 00 00 03 00 80 00 00 28. Remaining bits: 0 | 010 -> B.
  const std::vector<uint8_t> nal{0x41, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x28};
  EXPECT_EQ(parse_slice_type(nal), SliceType::kB);
}

TEST(H264Inspect, SliceTypeRejectsNonSliceAndTruncated)
{
  EXPECT_FALSE(parse_slice_type(kSps).has_value());
  EXPECT_FALSE(parse_slice_type(kPps).has_value());
  EXPECT_FALSE(parse_slice_type(std::vector<uint8_t>{}).has_value());
  EXPECT_FALSE(parse_slice_type(std::vector<uint8_t>{0x41}).has_value());
  EXPECT_FALSE(parse_slice_type(std::vector<uint8_t>{0x41, 0x00}).has_value());
  // slice_type = 10 is out of range: "1" + "0001011" -> 0x8B
  EXPECT_FALSE(parse_slice_type(std::vector<uint8_t>{0x41, 0x8B}).has_value());
}

TEST(H264Inspect, IdrAccessUnitWithParameterSets)
{
  const AccessUnitInfo info =
    inspect_access_unit(annexb({{0x09, 0xF0}, kSps, kPps, {0x65, 0x88, 0x80}}));
  EXPECT_TRUE(info.starts_with_start_code);
  EXPECT_TRUE(info.has_sps);
  EXPECT_TRUE(info.has_pps);
  EXPECT_TRUE(info.has_idr);
  EXPECT_TRUE(info.sps_pps_before_idr);
  EXPECT_FALSE(info.has_b_slice);
  EXPECT_EQ(info.nal_types, (std::vector<uint8_t>{9, 7, 8, 5}));
}

TEST(H264Inspect, IdrWithoutParameterSetsIsFlagged)
{
  const AccessUnitInfo info = inspect_access_unit(annexb({{0x65, 0x88, 0x80}}, false));
  EXPECT_TRUE(info.starts_with_start_code);
  EXPECT_TRUE(info.has_idr);
  EXPECT_FALSE(info.sps_pps_before_idr);

  const AccessUnitInfo late = inspect_access_unit(annexb({{0x65, 0x88}, kSps, kPps}));
  EXPECT_TRUE(late.has_sps && late.has_pps);
  EXPECT_FALSE(late.sps_pps_before_idr);
}

TEST(H264Inspect, DetectsBSliceInAnySlice)
{
  // Two slices of one picture: P then B.
  const AccessUnitInfo info = inspect_access_unit(annexb({{0x41, 0xC0}, {0x41, 0x22}}));
  EXPECT_FALSE(info.has_idr);
  EXPECT_TRUE(info.has_b_slice);

  const AccessUnitInfo p_only = inspect_access_unit(annexb({{0x41, 0xC0}, {0x41, 0x98}}));
  EXPECT_FALSE(p_only.has_b_slice);
}
