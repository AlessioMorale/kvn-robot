// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

// Pure status logic, no ROS dependency: staleness tracking per input and the
// reduction of all inputs to one short status string (`<STATE>[:<CODE>]`).

namespace kvn_status
{

/// Time on a monotonic clock. Only differences matter.
using TimePoint = std::chrono::nanoseconds;
using Duration = std::chrono::nanoseconds;

/// Maximum length of the status string (ELRS FLIGHT_MODE payload).
inline constexpr std::size_t kMaxStatusLength = 15;

enum class Severity
{
  kOk = 0,
  kWarn = 1,
  kFault = 2,
};

/// Operating state, reported when no WARN or FAULT is active.
enum class OperatingState
{
  kReady,
  kDriving,
};

/// One active (or inactive, when severity is kOk) condition from an input.
struct Condition
{
  Severity severity{Severity::kOk};
  std::string code;
  /// When the condition started; among conditions of equal severity the oldest wins.
  TimePoint onset{0};
};

/**
 * @brief Reduce conditions and the operating state to the status string.
 *
 * FAULT > WARN > operating state (DRV/RDY). Among conditions of the same
 * severity the one with the earliest onset wins; equal onsets go to the
 * earliest in @p conditions. The code is made printable ASCII and the result
 * is truncated to @p max_length characters.
 */
std::string reduce_status(
  const std::vector<Condition> & conditions, OperatingState state,
  std::size_t max_length = kMaxStatusLength);

/// Format a single state/code pair (code may be empty), sanitised and truncated.
std::string format_status(
  const std::string & state, const std::string & code, std::size_t max_length = kMaxStatusLength);

/**
 * @brief Tracks one input: its last reported severity/code and its staleness.
 *
 * An input that has not been updated within @p timeout (counting from
 * @p start for an input never received) is reported as FAULT with the
 * stale code. Until then a never-received input reports kOk.
 */
class InputMonitor
{
public:
  InputMonitor(std::string stale_code, Duration timeout, TimePoint start);

  /// Record a fresh reading.
  void update(Severity severity, const std::string & code, TimePoint now);

  /// Current condition of this input at @p now (stale → FAULT).
  Condition evaluate(TimePoint now) const;

  bool is_stale(TimePoint now) const;
  bool has_data() const { return has_data_; }
  Duration timeout() const { return timeout_; }

private:
  std::string stale_code_;
  Duration timeout_;
  TimePoint last_update_;
  bool has_data_{false};
  Condition current_;
};

}  // namespace kvn_status
