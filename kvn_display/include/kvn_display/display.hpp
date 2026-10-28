// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "kvn_display/screen.hpp"
#include "kvn_display/transport.hpp"
#include "u8g2.h"

namespace kvn_display
{

using U8g2Setup = void (*)(u8g2_t *, const u8g2_cb_t *, u8x8_msg_cb, u8x8_msg_cb);

/// The u8g2 full-buffer setup function of a driver, named as u8g2 does without the
/// `u8g2_Setup_` prefix and `_f` suffix (`ssd1306_i2c_64x32_1f` is
/// `u8g2_Setup_ssd1306_i2c_64x32_1f_f`); nullptr for an unknown name.
U8g2Setup find_setup(std::string_view driver);

/// u8g2's I2C drivers have `_i2c_` in their name; all others are driven over 4-wire SPI.
bool is_i2c_driver(std::string_view driver);

struct DisplayConfig
{
  std::string driver = "ssd1306_i2c_64x32_1f";
  int rotation = 0;   ///< Degrees: 0, 90, 180 or 270
  int contrast = -1;  ///< 0..255, -1 keeps the driver's default
  std::string i2c_device = "/dev/i2c-1";
  int i2c_address = 0x3C;
  SpiConfig spi;
};

/// Throws std::invalid_argument for an unknown driver, or a rotation, contrast, I2C address or
/// SPI D/C line that cannot work.
void validate(const DisplayConfig & config);

/**
 * @class Display
 * @brief A display driven by u8g2 through a Transport.
 */
class Display
{
public:
  /// Initialises and clears the display. Throws std::invalid_argument (see validate()) or
  /// std::system_error on a bus error.
  Display(const DisplayConfig & config, std::unique_ptr<Transport> transport);
  /// Opens the transport the driver needs (I2C or SPI), then as above.
  explicit Display(const DisplayConfig & config);
  /// Puts the display into power save (errors ignored).
  ~Display();
  // u8g2 keeps a pointer to the transport
  Display(const Display &) = delete;
  Display & operator=(const Display &) = delete;

  int width() const;
  int height() const;
  /// Draws the lines (see draw_lines()) and sends the picture; throws std::system_error.
  void show(const std::vector<Line> & lines);

  const u8g2_t & u8g2() const {return u8g2_;}

private:
  void check(const char * action);

  u8g2_t u8g2_{};
  std::unique_ptr<Transport> transport_;
};

}  // namespace kvn_display
