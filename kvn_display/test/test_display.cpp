// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include <gtest/gtest.h>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <vector>

#include "kvn_display/display.hpp"
#include "recording_transport.hpp"

namespace kvn_display
{
namespace
{

std::unique_ptr<Display> make(const std::shared_ptr<TransportLog> & log, DisplayConfig config = {})
{
  return std::make_unique<Display>(config, std::make_unique<RecordingTransport>(log));
}

bool any_transfer_contains(const TransportLog & log, std::uint8_t byte)
{
  return std::any_of(log.transfers.begin(), log.transfers.end(), [&](const auto & transfer) {
    return std::find(transfer.begin() + 1, transfer.end(), byte) != transfer.end();
  });
}

TEST(Drivers, FindsU8g2SetupFunctionsByName)
{
  EXPECT_NE(find_setup("ssd1306_i2c_64x32_1f"), nullptr);
  EXPECT_NE(find_setup("ssd1306_i2c_128x64_noname"), nullptr);
  EXPECT_NE(find_setup("sh1106_i2c_128x64_noname"), nullptr);
  EXPECT_NE(find_setup("st7920_s_128x64"), nullptr);
  EXPECT_EQ(find_setup("ssd1306_i2c_64x32_1f_f"), nullptr);
  EXPECT_EQ(find_setup("no_such_display"), nullptr);
}

TEST(Drivers, BusFollowsTheDriverName)
{
  EXPECT_TRUE(is_i2c_driver("ssd1306_i2c_64x32_1f"));
  EXPECT_FALSE(is_i2c_driver("ssd1306_64x32_1f"));
}

TEST(Drivers, ValidatesTheConfiguration)
{
  DisplayConfig config;
  EXPECT_NO_THROW(validate(config));
  config.driver = "no_such_display";
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = DisplayConfig{};
  config.rotation = 45;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = DisplayConfig{};
  config.contrast = 256;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = DisplayConfig{};
  config.i2c_address = 0x78;
  EXPECT_THROW(validate(config), std::invalid_argument);
  config = DisplayConfig{};
  config.driver = "ssd1306_64x32_1f";  // SPI without a D/C line
  EXPECT_THROW(validate(config), std::invalid_argument);
  config.spi.dc_line = 25;
  EXPECT_NO_THROW(validate(config));
}

TEST(I2cDisplay, InitialisesAndSwitchesOnThe64x32Ssd1306)
{
  auto log = std::make_shared<TransportLog>();
  auto display = make(log);
  EXPECT_EQ(display->width(), 64);
  EXPECT_EQ(display->height(), 32);
  ASSERT_FALSE(log->transfers.empty());
  // SSD1306 over I2C: every transfer starts with the control byte, commands or data
  for (const auto & transfer : log->transfers) {
    ASSERT_FALSE(transfer.empty());
    EXPECT_TRUE(transfer[0] == 0x00 || transfer[0] == 0x40);
  }
  EXPECT_TRUE(any_transfer_contains(*log, 0x8D));  // charge pump
  EXPECT_TRUE(any_transfer_contains(*log, 0xAF));  // display on
}

TEST(I2cDisplay, SendsTheFrameAndPowerSavesWhenDestroyed)
{
  auto log = std::make_shared<TransportLog>();
  auto display = make(log);
  log->transfers.clear();
  display->show({{"RDY"}});
  // 64 x 32 pixels: 256 bytes of data after the control bytes
  std::size_t data_bytes = 0;
  for (const auto & transfer : log->transfers) {
    if (transfer[0] == 0x40) {
      data_bytes += transfer.size() - 1;
    }
  }
  EXPECT_EQ(data_bytes, 256u);

  log->transfers.clear();
  display.reset();
  EXPECT_TRUE(any_transfer_contains(*log, 0xAE));  // display off
}

TEST(I2cDisplay, ContrastIsSentWhenSet)
{
  auto log = std::make_shared<TransportLog>();
  DisplayConfig config;
  config.contrast = 0x42;
  make(log, config);
  const std::vector<std::uint8_t> contrast_command{0x81, 0x42};
  EXPECT_TRUE(std::any_of(log->transfers.begin(), log->transfers.end(), [&](const auto & t) {
    return std::search(t.begin(), t.end(), contrast_command.begin(), contrast_command.end()) !=
           t.end();
  }));
}

TEST(I2cDisplay, BusErrorsReachTheCaller)
{
  auto log = std::make_shared<TransportLog>();
  auto display = make(log);
  log->fail = true;
  EXPECT_THROW(display->show({{"RDY"}}), std::system_error);
  // Power save on destruction fails quietly
  display.reset();

  auto failing = std::make_shared<TransportLog>();
  failing->fail = true;
  EXPECT_THROW(make(failing), std::system_error);
}

TEST(SpiDisplay, UsesTheDcLineResetAndTheControllerSpiSettings)
{
  auto log = std::make_shared<TransportLog>();
  DisplayConfig config;
  config.driver = "ssd1306_64x32_1f";
  config.spi.dc_line = 25;
  auto display = make(log, config);
  EXPECT_GT(log->max_clock_hz, 0u);
  EXPECT_GE(log->spi_mode, 0);
  // Commands with D/C low, frame data with D/C high
  EXPECT_NE(std::find(log->dc.begin(), log->dc.end(), false), log->dc.end());
  EXPECT_NE(std::find(log->dc.begin(), log->dc.end(), true), log->dc.end());
  // Reset pulsed low, then released
  ASSERT_FALSE(log->reset.empty());
  EXPECT_NE(std::find(log->reset.begin(), log->reset.end(), false), log->reset.end());
  EXPECT_TRUE(log->reset.back());
  // No I2C control bytes over SPI: the frame is 256 bytes of pixels
  log->transfers.clear();
  display->show({});
  std::size_t total = 0;
  for (const auto & transfer : log->transfers) {
    total += transfer.size();
  }
  EXPECT_GE(total, 256u);
}

}  // namespace
}  // namespace kvn_display
