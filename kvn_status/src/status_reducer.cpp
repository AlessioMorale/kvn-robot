// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_status/status_reducer.hpp"

#include <utility>

namespace kvn_status
{

namespace
{

/// Oldest active condition of the given severity, or nullptr.
const Condition * oldest_of(const std::vector<Condition> & conditions, Severity severity)
{
  const Condition * best = nullptr;
  for (const auto & condition : conditions)
  {
    if (condition.severity != severity)
    {
      continue;
    }
    // Strict '<': equal onsets keep the earlier entry
    if (best == nullptr || condition.onset < best->onset)
    {
      best = &condition;
    }
  }
  return best;
}

}  // namespace

std::string format_status(
  const std::string & state, const std::string & code, std::size_t max_length)
{
  std::string out = state;
  if (!code.empty())
  {
    out += ':';
    out += code;
  }
  for (auto & c : out)
  {
    const auto u = static_cast<unsigned char>(c);
    if (u < 0x20 || u > 0x7E)
    {
      c = '?';
    }
  }
  if (out.size() > max_length)
  {
    out.resize(max_length);
  }
  return out;
}

std::string reduce_status(
  const std::vector<Condition> & conditions, OperatingState state, std::size_t max_length)
{
  if (const auto * fault = oldest_of(conditions, Severity::kFault))
  {
    return format_status("FLT", fault->code, max_length);
  }
  if (const auto * warn = oldest_of(conditions, Severity::kWarn))
  {
    return format_status("WRN", warn->code, max_length);
  }
  return format_status(state == OperatingState::kDriving ? "DRV" : "RDY", "", max_length);
}

InputMonitor::InputMonitor(std::string stale_code, Duration timeout, TimePoint start)
: stale_code_(std::move(stale_code)), timeout_(timeout), last_update_(start)
{
}

void InputMonitor::update(Severity severity, const std::string & code, TimePoint now)
{
  // A new condition (or one returning after a stale period) starts now
  const bool changed = severity != current_.severity || code != current_.code;
  if (!has_data_ || changed || is_stale(now))
  {
    current_.onset = now;
  }
  current_.severity = severity;
  current_.code = code;
  last_update_ = now;
  has_data_ = true;
}

bool InputMonitor::is_stale(TimePoint now) const { return now - last_update_ > timeout_; }

Condition InputMonitor::evaluate(TimePoint now) const
{
  if (is_stale(now))
  {
    return Condition{Severity::kFault, stale_code_, last_update_ + timeout_};
  }
  if (!has_data_)
  {
    return Condition{Severity::kOk, "", last_update_};
  }
  return current_;
}

}  // namespace kvn_status
