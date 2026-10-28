// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_display/screen.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>


namespace kvn_display
{

namespace
{

std::string upper(std::string_view text)
{
  std::string result(text);
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {
    return static_cast<char>(c == '_' ? ' ' : std::toupper(c));
  });
  return result;
}

bool is_alert(std::string_view status)
{
  const auto state = status.substr(0, status.find(':'));
  return state == "FLT" || state == "WRN";
}

}  // namespace

std::string battery_text(std::optional<double> voltage, std::optional<double> percentage)
{
  if (!voltage || !std::isfinite(*voltage)) {
    return "--.-V";
  }
  char text[16];
  if (percentage && std::isfinite(*percentage) && *percentage >= 0.0 && *percentage <= 1.0) {
    std::snprintf(text, sizeof(text), "%.1fV %d%%", *voltage,
      static_cast<int>(std::lround(*percentage * 100.0)));
  } else {
    std::snprintf(text, sizeof(text), "%.1fV", *voltage);
  }
  return text;
}

std::string mode_label(std::string_view mode)
{
  // Modes published by kvn_orchestrator (see its README, /orchestrator/mode)
  static const std::map<std::string_view, std::string> labels = {
    {"waiting_for_nav2", "NAV2 WAIT"},
    {"initialising", "INIT"},
    {"fault", "BLOCKED"},
    {"teleop", "TELEOP"},
    {"teleop_override", "OVERRIDE"},
    {"assisted_teleop", "ASSIST"},
    {"autonomous", "AUTO"},
    {"exploring", "EXPLORE"},
    {"returning_home", "RTH"},
    {"battery_rth", "BATT RTH"},
    {"battery_rth_complete", "HOME"},
  };
  const auto it = labels.find(mode);
  return it != labels.end() ? it->second : upper(mode);
}

std::vector<Line> screen_lines(const ScreenState & state)
{
  std::vector<Line> lines;
  if (state.robot_status) {
    lines.push_back({*state.robot_status, is_alert(*state.robot_status)});
  } else {
    lines.push_back({"NO STATUS", true});
  }
  lines.push_back({battery_text(state.battery_voltage, state.battery_percentage)});
  lines.push_back({state.mode ? mode_label(*state.mode) : ""});
  lines.push_back({state.cmd_source ? "CMD " + upper(*state.cmd_source) : ""});
  return lines;
}

const uint8_t * screen_font(u8g2_t * u8g2, int line_height)
{
  // X11 fixed fonts, largest first; built from u8g2_fonts.c by CMakeLists.txt (KVN_U8G2_FONTS)
  static const std::array<const uint8_t *, 5> fonts = {
    u8g2_font_10x20_tr, u8g2_font_9x15_tr, u8g2_font_7x13_tr, u8g2_font_6x10_tr, u8g2_font_5x7_tr};
  for (const uint8_t * font : fonts) {
    u8g2_SetFont(u8g2, font);
    if (u8g2_GetMaxCharHeight(u8g2) <= line_height) {
      return font;
    }
  }
  return fonts.back();
}

void draw_lines(u8g2_t * u8g2, const std::vector<Line> & lines)
{
  const int width = u8g2_GetDisplayWidth(u8g2);
  const int height = u8g2_GetDisplayHeight(u8g2);
  const int pitch = std::max(height / kScreenLines, 1);
  u8g2_ClearBuffer(u8g2);
  u8g2_SetFont(u8g2, screen_font(u8g2, pitch));
  u8g2_SetFontMode(u8g2, 1);  // transparent, so text can be drawn in colour 0 on a lit box
  u8g2_SetFontPosTop(u8g2);
  const int char_height = u8g2_GetMaxCharHeight(u8g2);
  const auto count = std::min<std::size_t>(lines.size(), kScreenLines);
  for (std::size_t i = 0; i < count; ++i) {
    const int top = static_cast<int>(i) * pitch;
    if (lines[i].inverted) {
      u8g2_SetDrawColor(u8g2, 1);
      u8g2_DrawBox(u8g2, 0, static_cast<u8g2_uint_t>(top), static_cast<u8g2_uint_t>(width),
        static_cast<u8g2_uint_t>(pitch));
      u8g2_SetDrawColor(u8g2, 0);
    }
    u8g2_DrawStr(u8g2, 0, static_cast<u8g2_uint_t>(top + std::max(pitch - char_height, 0) / 2),
      lines[i].text.c_str());
    u8g2_SetDrawColor(u8g2, 1);
  }
}

}  // namespace kvn_display
