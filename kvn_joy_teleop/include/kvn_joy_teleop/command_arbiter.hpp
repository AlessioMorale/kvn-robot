// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <string>

// Pure command arbitration, no ROS dependency: decides who drives the rover.

namespace kvn_joy_teleop
{

enum class Source
{
  /// Nobody: the rover is stopped.
  kNone,
  /// The operator, with the enable (deadman) switch held.
  kTeleop,
  /// Nav2 (through the orchestrator's autonomy permit).
  kAuto,
};

inline const char * to_string(Source source)
{
  switch (source) {
    case Source::kTeleop:
      return "teleop";
    case Source::kAuto:
      return "auto";
    default:
      return "none";
  }
}

struct ArbiterInputs
{
  /// A Joy message arrived within joy_timeout_s.
  bool joy_fresh{false};
  /// The enable switch is held in the latest Joy message.
  bool enable_pressed{false};
  /// An autonomous command arrived within auto_timeout_s.
  bool auto_fresh{false};
  /// The autonomy permit is true and arrived within autonomy_timeout_s.
  bool autonomy_permitted{false};
};

struct ArbiterConfig
{
  /// Forward autonomous commands only while Joy is alive, i.e. while the operator can override.
  bool require_joy_for_auto{true};
  /// Forward autonomous commands only while the orchestrator permits it.
  bool require_autonomy_permit{true};
};

/// The operator always wins: enable held on a live Joy link means teleop, whatever else is going
/// on. Otherwise autonomous commands pass if they are fresh and allowed; else nobody drives.
inline Source select_source(const ArbiterInputs & in, const ArbiterConfig & config)
{
  if (in.joy_fresh && in.enable_pressed) {
    return Source::kTeleop;
  }
  if (in.auto_fresh &&
    (in.joy_fresh || !config.require_joy_for_auto) &&
    (in.autonomy_permitted || !config.require_autonomy_permit))
  {
    return Source::kAuto;
  }
  return Source::kNone;
}

}  // namespace kvn_joy_teleop
