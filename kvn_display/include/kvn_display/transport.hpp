// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

namespace kvn_display
{

/**
 * @class Transport
 * @brief Moves u8g2's bytes to the display: one transfer is start(), send()..., end().
 *
 * u8g2 calls the transport from C, so bus errors are not thrown from these calls: the first one
 * is kept and returned by take_error().
 */
class Transport
{
public:
  virtual ~Transport() = default;

  /// Once, before the first transfer: the controller's SPI mode (CPOL << 1 | CPHA) and clock.
  virtual void init(int spi_mode, std::uint32_t max_clock_hz);
  /// SPI data/command line: true for display data, false for commands.
  virtual void set_dc(bool data);
  /// Reset line level (low resets the controller); ignored without a reset line.
  virtual void set_reset(bool level);

  void start();
  void send(const std::uint8_t * data, std::size_t size);
  void end();

  /// The first bus error since the last call, if any.
  std::optional<std::system_error> take_error();

protected:
  /// Sends one transfer; throws std::system_error.
  virtual void transfer(std::span<const std::uint8_t> data) = 0;
  /// Sends the bytes collected since start(), if any.
  void flush();
  void record(const std::system_error & error);

private:
  std::vector<std::uint8_t> buffer_;
  std::optional<std::system_error> error_;
};

/// A device on a Linux i2c-dev bus (/dev/i2c-N).
class I2cTransport : public Transport
{
public:
  /// Opens the bus and selects the device; throws std::system_error.
  I2cTransport(const std::string & device, int address);
  ~I2cTransport() override;
  I2cTransport(const I2cTransport &) = delete;
  I2cTransport & operator=(const I2cTransport &) = delete;

protected:
  void transfer(std::span<const std::uint8_t> data) override;

private:
  int fd_;
  std::string name_;
};

struct SpiConfig
{
  std::string device = "/dev/spidev1.0";
  std::uint32_t speed_hz = 0;  ///< 0: the controller's maximum
  std::string gpio_chip = "/dev/gpiochip0";
  int dc_line = -1;            ///< Data/command GPIO line offset, required
  int reset_line = -1;         ///< Reset GPIO line offset, -1 without one
};

/// A display on a Linux spidev bus (4-wire: chip select by the kernel, D/C and reset by GPIO).
class SpiTransport : public Transport
{
public:
  /// Opens the bus and requests the GPIO lines; throws std::system_error.
  explicit SpiTransport(const SpiConfig & config);
  ~SpiTransport() override;
  SpiTransport(const SpiTransport &) = delete;
  SpiTransport & operator=(const SpiTransport &) = delete;

  void init(int spi_mode, std::uint32_t max_clock_hz) override;
  void set_dc(bool data) override;
  void set_reset(bool level) override;

protected:
  void transfer(std::span<const std::uint8_t> data) override;

private:
  class Gpio;

  SpiConfig config_;
  int fd_;
  std::unique_ptr<Gpio> gpio_;
};

}  // namespace kvn_display
