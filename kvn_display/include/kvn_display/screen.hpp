// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "u8g2.h"

namespace kvn_display
{

/// What the screen shows. An empty optional means the input is missing or stale.
struct ScreenState
{
  std::optional<std::string> robot_status;   ///< /robot_status, e.g. "RDY" or "FLT:BATT"
  std::optional<double> battery_voltage;     ///< /battery_state voltage
  std::optional<double> battery_percentage;  ///< /battery_state percentage, 0..1
  std::optional<std::string> mode;           ///< /orchestrator/mode
  std::optional<std::string> cmd_source;     ///< /cmd_vel_source: teleop, auto or none
};

struct Line
{
  std::string text;
  bool inverted = false;  ///< Light background, dark text

  bool operator==(const Line & other) const = default;
};

/// Lines on the screen; the panel height is divided between them.
inline constexpr int kScreenLines = 4;

/// Status (inverted for FLT, WRN or no status), battery, orchestrator mode, command source.
std::vector<Line> screen_lines(const ScreenState & state);

/// "14.8V 72%", "14.8V" without a valid percentage, "--.-V" without a voltage.
std::string battery_text(std::optional<double> voltage, std::optional<double> percentage);

/// Short label (at most 10 characters) for an orchestrator mode; unknown modes in upper case.
std::string mode_label(std::string_view mode);

/// The largest built-in font whose characters fit a line of `line_height` pixels (changes the
/// font set on `u8g2`).
const uint8_t * screen_font(u8g2_t * u8g2, int line_height);

/// Clears the u8g2 buffer and draws the lines from the top, kScreenLines per panel height. Text
/// is cut at the panel edge; lines beyond kScreenLines are dropped.
void draw_lines(u8g2_t * u8g2, const std::vector<Line> & lines);

}  // namespace kvn_display
