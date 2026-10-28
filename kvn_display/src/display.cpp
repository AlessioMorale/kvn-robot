// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_display/display.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#ifndef U8X8_WITH_USER_PTR
#error "u8g2 must be built with U8X8_WITH_USER_PTR (defined by u8x8.h on Linux)"
#endif

namespace kvn_display
{

namespace
{

Transport & transport_of(u8x8_t * u8x8)
{
  return *static_cast<Transport *>(u8x8_GetUserPtr(u8x8));
}

// u8g2 byte callback: hands the bytes of each transfer to the transport
uint8_t byte_callback(u8x8_t * u8x8, uint8_t msg, uint8_t arg_int, void * arg_ptr)
{
  auto & transport = transport_of(u8x8);
  try {
    switch (msg) {
      case U8X8_MSG_BYTE_INIT:
        transport.init(u8x8->display_info->spi_mode, u8x8->display_info->sck_clock_hz);
        return 1;
      case U8X8_MSG_BYTE_SET_DC:
        transport.set_dc(arg_int != 0);
        return 1;
      case U8X8_MSG_BYTE_START_TRANSFER:
        transport.start();
        return 1;
      case U8X8_MSG_BYTE_SEND:
        transport.send(static_cast<const std::uint8_t *>(arg_ptr), arg_int);
        return 1;
      case U8X8_MSG_BYTE_END_TRANSFER:
        transport.end();
        return 1;
      default:
        return 0;
    }
  } catch (...) {
    // Nothing may propagate into u8g2's C code
    return 0;
  }
}

// u8g2 GPIO and delay callback: the reset line and the controller's timing
uint8_t gpio_and_delay_callback(u8x8_t * u8x8, uint8_t msg, uint8_t arg_int, void *)
{
  switch (msg) {
    case U8X8_MSG_DELAY_MILLI:
      std::this_thread::sleep_for(std::chrono::milliseconds(arg_int));
      return 1;
    case U8X8_MSG_DELAY_10MICRO:
      std::this_thread::sleep_for(std::chrono::microseconds(10 * arg_int));
      return 1;
    case U8X8_MSG_GPIO_RESET:
      transport_of(u8x8).set_reset(arg_int != 0);
      return 1;
    default:
      // Shorter delays, and pins handled by the transport (chip select, D/C)
      return 1;
  }
}

const u8g2_cb_t * rotation_of(int degrees)
{
  switch (degrees) {
    case 0:
      return U8G2_R0;
    case 90:
      return U8G2_R1;
    case 180:
      return U8G2_R2;
    case 270:
      return U8G2_R3;
    default:
      return nullptr;
  }
}

std::unique_ptr<Transport> open_transport(const DisplayConfig & config)
{
  validate(config);
  if (is_i2c_driver(config.driver)) {
    return std::make_unique<I2cTransport>(config.i2c_device, config.i2c_address);
  }
  return std::make_unique<SpiTransport>(config.spi);
}

}  // namespace

bool is_i2c_driver(std::string_view driver)
{
  return driver.find("_i2c_") != std::string_view::npos;
}

void validate(const DisplayConfig & config)
{
  if (find_setup(config.driver) == nullptr) {
    throw std::invalid_argument(
            "unknown u8g2 display driver '" + config.driver +
            "' (a u8g2 full-buffer setup function name without u8g2_Setup_ and _f, e.g. "
            "ssd1306_i2c_128x64_noname)");
  }
  if (rotation_of(config.rotation) == nullptr) {
    throw std::invalid_argument("rotation must be 0, 90, 180 or 270");
  }
  if (config.contrast < -1 || config.contrast > 255) {
    throw std::invalid_argument("contrast must be in 0..255, or -1 for the driver default");
  }
  if (is_i2c_driver(config.driver)) {
    // 7-bit addresses outside the reserved ranges
    if (config.i2c_address < 0x03 || config.i2c_address > 0x77) {
      throw std::invalid_argument("i2c_address must be in 0x03..0x77");
    }
  } else if (config.spi.dc_line < 0) {
    throw std::invalid_argument(config.driver + " is an SPI display: set dc_line");
  }
}

Display::Display(const DisplayConfig & config, std::unique_ptr<Transport> transport)
: transport_(std::move(transport))
{
  validate(config);
  find_setup(config.driver)(
    &u8g2_, rotation_of(config.rotation), byte_callback, gpio_and_delay_callback);
  u8x8_SetUserPtr(u8g2_GetU8x8(&u8g2_), transport_.get());
  // Reset and initialisation sequence; the display stays in power save until cleared
  u8g2_InitDisplay(&u8g2_);
  check("initialise");
  if (config.contrast >= 0) {
    u8g2_SetContrast(&u8g2_, static_cast<uint8_t>(config.contrast));
  }
  u8g2_ClearBuffer(&u8g2_);
  u8g2_SendBuffer(&u8g2_);
  u8g2_SetPowerSave(&u8g2_, 0);
  check("switch on");
}

Display::Display(const DisplayConfig & config)
: Display(config, open_transport(config))
{
}

Display::~Display()
{
  u8g2_SetPowerSave(&u8g2_, 1);
  transport_->take_error();
}

int Display::width() const
{
  return u8g2_.width;
}

int Display::height() const
{
  return u8g2_.height;
}

void Display::show(const std::vector<Line> & lines)
{
  draw_lines(&u8g2_, lines);
  u8g2_SendBuffer(&u8g2_);
  check("update");
}

void Display::check(const char * action)
{
  if (auto error = transport_->take_error()) {
    throw std::system_error(error->code(), std::string("display ") + action + ": " + error->what());
  }
}

}  // namespace kvn_display
