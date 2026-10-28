// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>

#include "kvn_display/display.hpp"
#include "kvn_display/screen.hpp"
#include "recording_transport.hpp"

namespace kvn_display
{
namespace
{

TEST(Screen, ShowsStatusBatteryModeAndCommandSource)
{
  ScreenState state;
  state.robot_status = "RDY";
  state.battery_voltage = 14.84;
  state.battery_percentage = 0.724;
  state.mode = "assisted_teleop";
  state.cmd_source = "auto";
  const auto lines = screen_lines(state);
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(lines[0].text, "RDY");
  EXPECT_FALSE(lines[0].inverted);
  EXPECT_EQ(lines[1].text, "14.8V 72%");
  EXPECT_EQ(lines[2].text, "ASSIST");
  EXPECT_EQ(lines[3].text, "CMD AUTO");
}

TEST(Screen, FaultsWarningsAndMissingStatusAreInverted)
{
  ScreenState state;
  state.robot_status = "FLT:BATT";
  EXPECT_TRUE(screen_lines(state)[0].inverted);
  state.robot_status = "WRN:HWMON";
  EXPECT_TRUE(screen_lines(state)[0].inverted);
  state.robot_status.reset();
  EXPECT_EQ(screen_lines(state)[0].text, "NO STATUS");
  EXPECT_TRUE(screen_lines(state)[0].inverted);
}

TEST(Screen, MissingInputsLeaveTheirLinesEmpty)
{
  const auto lines = screen_lines(ScreenState{});
  EXPECT_EQ(lines[1].text, "--.-V");
  EXPECT_EQ(lines[2].text, "");
  EXPECT_EQ(lines[3].text, "");
}

TEST(Screen, BatteryWithoutAValidPercentage)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(battery_text(12.4, nan), "12.4V");
  EXPECT_EQ(battery_text(12.4, std::nullopt), "12.4V");
  EXPECT_EQ(battery_text(nan, 0.5), "--.-V");
}

TEST(Screen, EveryOrchestratorModeFitsTheNarrowestPanel)
{
  // 64 px / 6 px per glyph
  for (const char * mode : {"waiting_for_nav2", "initialising", "fault", "teleop",
      "teleop_override", "assisted_teleop", "autonomous", "exploring", "returning_home",
      "battery_rth", "battery_rth_complete"})
  {
    EXPECT_LE(mode_label(mode).size(), 10u) << mode;
  }
  EXPECT_EQ(mode_label("new_mode"), "NEW MODE");
}

/// u8g2 full-buffer pixel of an SSD1306: one byte per column of 8 rows, bit 0 at the top.
bool pixel(const u8g2_t & u8g2, int x, int y)
{
  const int buffer_width = u8g2_GetBufferTileWidth(&u8g2) * 8;
  const auto index = static_cast<std::size_t>((y / 8) * buffer_width + x);
  return (u8g2.tile_buf_ptr[index] >> (y % 8)) & 1;
}

int lit_pixels(const u8g2_t & u8g2, int top, int bottom)
{
  int count = 0;
  for (int y = top; y < bottom; ++y) {
    for (int x = 0; x < u8g2_GetDisplayWidth(&u8g2); ++x) {
      count += pixel(u8g2, x, y) ? 1 : 0;
    }
  }
  return count;
}

std::unique_ptr<Display> display_of(const char * driver)
{
  DisplayConfig config;
  config.driver = driver;
  return std::make_unique<Display>(
    config, std::make_unique<RecordingTransport>(std::make_shared<TransportLog>()));
}

TEST(Screen, DrawsFourLinesOnA64x32Panel)
{
  auto display = display_of("ssd1306_i2c_64x32_1f");
  display->show({{"FLT:BATT", true}, {"14.8V"}, {""}, {"CMD AUTO"}, {"dropped"}});
  const auto & u8g2 = display->u8g2();
  // Inverted first line: lit background with dark text, rows 0..7
  EXPECT_TRUE(pixel(u8g2, 63, 0));
  EXPECT_TRUE(pixel(u8g2, 63, 7));
  EXPECT_GT(lit_pixels(u8g2, 0, 8), 64 * 8 / 2);
  EXPECT_LT(lit_pixels(u8g2, 0, 8), 64 * 8);
  // Normal lines: dark background, some text
  EXPECT_GT(lit_pixels(u8g2, 8, 16), 0);
  EXPECT_FALSE(pixel(u8g2, 63, 12));
  EXPECT_EQ(lit_pixels(u8g2, 16, 24), 0);
  EXPECT_GT(lit_pixels(u8g2, 24, 32), 0);
}

TEST(Screen, FontGrowsWithThePanel)
{
  auto small = display_of("ssd1306_i2c_64x32_1f");
  auto large = display_of("ssd1306_i2c_128x64_noname");
  u8g2_t small_u8g2 = small->u8g2();
  u8g2_t large_u8g2 = large->u8g2();
  EXPECT_EQ(screen_font(&small_u8g2, 32 / kScreenLines), u8g2_font_5x7_tr);
  EXPECT_EQ(screen_font(&large_u8g2, 64 / kScreenLines), u8g2_font_9x15_tr);
  EXPECT_EQ(screen_font(&small_u8g2, 1), u8g2_font_5x7_tr);
}

}  // namespace
}  // namespace kvn_display
