// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_video_streamer/h264_inspect.hpp"

namespace kvn_video_streamer::h264
{

bool starts_with_start_code(std::span<const uint8_t> data)
{
  if (data.size() >= 3 && data[0] == 0 && data[1] == 0 && data[2] == 1)
  {
    return true;
  }
  return data.size() >= 4 && data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 1;
}

std::vector<NalUnit> split_nal_units(std::span<const uint8_t> data)
{
  // Positions of every "00 00 01" sequence.
  std::vector<std::size_t> marks;
  for (std::size_t i = 0; i + 2 < data.size(); ++i)
  {
    if (data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1)
    {
      marks.push_back(i);
      i += 2;
    }
  }

  std::vector<NalUnit> nals;
  nals.reserve(marks.size());
  for (std::size_t m = 0; m < marks.size(); ++m)
  {
    const std::size_t begin = marks[m] + 3;
    std::size_t end = (m + 1 < marks.size()) ? marks[m + 1] : data.size();
    while (end > begin && data[end - 1] == 0)
    {
      --end;  // trailing_zero_8bits / leading zero of a 4-byte start code
    }
    if (end <= begin)
    {
      continue;  // empty NAL
    }
    NalUnit nal{};
    nal.offset = begin;
    nal.size = end - begin;
    nal.start_code_size = (marks[m] > 0 && data[marks[m] - 1] == 0) ? 4U : 3U;
    nal.type = static_cast<uint8_t>(data[begin] & 0x1FU);
    nals.push_back(nal);
  }
  return nals;
}

std::vector<uint8_t> unescape_rbsp(std::span<const uint8_t> nal, std::size_t max_out)
{
  std::vector<uint8_t> out;
  out.reserve(max_out != 0 ? max_out : nal.size());
  std::size_t zeros = 0;
  for (const uint8_t b : nal)
  {
    if (max_out != 0 && out.size() >= max_out)
    {
      break;
    }
    if (zeros >= 2 && b == 0x03)
    {
      zeros = 0;  // emulation_prevention_three_byte: drop it
      continue;
    }
    out.push_back(b);
    zeros = (b == 0) ? zeros + 1 : 0;
  }
  return out;
}

BitReader::BitReader(std::span<const uint8_t> data) : data_(data) {}

std::optional<uint32_t> BitReader::read_bit()
{
  if (bit_pos_ >= data_.size() * 8U)
  {
    return std::nullopt;
  }
  const uint8_t byte = data_[bit_pos_ / 8U];
  const unsigned shift = 7U - static_cast<unsigned>(bit_pos_ % 8U);
  ++bit_pos_;
  return static_cast<uint32_t>((byte >> shift) & 1U);
}

std::optional<uint32_t> BitReader::read_ue()
{
  unsigned leading_zeros = 0;
  while (true)
  {
    const auto bit = read_bit();
    if (!bit)
    {
      return std::nullopt;
    }
    if (*bit == 1U)
    {
      break;
    }
    if (++leading_zeros > 31U)
    {
      return std::nullopt;  // malformed: value does not fit 32 bits
    }
  }
  uint32_t suffix = 0;
  for (unsigned i = 0; i < leading_zeros; ++i)
  {
    const auto bit = read_bit();
    if (!bit)
    {
      return std::nullopt;
    }
    suffix = (suffix << 1U) | *bit;
  }
  return ((uint32_t{1} << leading_zeros) - 1U) + suffix;
}

std::optional<SliceType> parse_slice_type(std::span<const uint8_t> nal)
{
  if (nal.empty())
  {
    return std::nullopt;
  }
  const uint8_t type = nal[0] & 0x1FU;
  if (
    type != static_cast<uint8_t>(NalType::kSliceNonIdr) &&
    type != static_cast<uint8_t>(NalType::kSliceDataA) &&
    type != static_cast<uint8_t>(NalType::kSliceIdr))
  {
    return std::nullopt;
  }
  // first_mb_in_slice and slice_type are at most 2 * 32 + 2 bits: 16 bytes is plenty.
  const std::vector<uint8_t> rbsp = unescape_rbsp(nal.subspan(1), 16);
  BitReader reader(rbsp);
  const auto first_mb_in_slice = reader.read_ue();
  if (!first_mb_in_slice)
  {
    return std::nullopt;
  }
  const auto slice_type = reader.read_ue();
  if (!slice_type || *slice_type > 9U)
  {
    return std::nullopt;
  }
  return static_cast<SliceType>(*slice_type % 5U);
}

AccessUnitInfo inspect_access_unit(std::span<const uint8_t> data)
{
  AccessUnitInfo info;
  info.starts_with_start_code = starts_with_start_code(data);
  for (const NalUnit & nal : split_nal_units(data))
  {
    info.nal_types.push_back(nal.type);
    const auto payload = data.subspan(nal.offset, nal.size);
    switch (static_cast<NalType>(nal.type))
    {
      case NalType::kSps:
        info.has_sps = true;
        break;
      case NalType::kPps:
        info.has_pps = true;
        break;
      case NalType::kSliceIdr:
        if (!info.has_idr)
        {
          info.sps_pps_before_idr = info.has_sps && info.has_pps;
        }
        info.has_idr = true;
        [[fallthrough]];
      case NalType::kSliceNonIdr:
      case NalType::kSliceDataA:
        if (parse_slice_type(payload) == SliceType::kB)
        {
          info.has_b_slice = true;
        }
        break;
      default:
        break;
    }
  }
  return info;
}

}  // namespace kvn_video_streamer::h264
