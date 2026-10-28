// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#include "kvn_display/transport.hpp"

#include <fcntl.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <utility>

namespace kvn_display
{

namespace
{

std::system_error errno_error(const std::string & what)
{
  return std::system_error(errno, std::generic_category(), what);
}

void write_all(int fd, std::span<const std::uint8_t> data, const std::string & name)
{
  const auto written = ::write(fd, data.data(), data.size());
  if (written < 0) {
    throw errno_error("write " + name);
  }
  if (static_cast<std::size_t>(written) != data.size()) {
    throw std::system_error(EIO, std::generic_category(), "short write " + name);
  }
}

}  // namespace

// --- Transport ---

void Transport::init(int, std::uint32_t) {}

void Transport::set_dc(bool) {}

void Transport::set_reset(bool) {}

void Transport::start()
{
  buffer_.clear();
}

void Transport::send(const std::uint8_t * data, std::size_t size)
{
  buffer_.insert(buffer_.end(), data, data + size);
}

void Transport::end()
{
  flush();
}

std::optional<std::system_error> Transport::take_error()
{
  return std::exchange(error_, std::nullopt);
}

void Transport::flush()
{
  if (buffer_.empty()) {
    return;
  }
  try {
    transfer(buffer_);
  } catch (const std::system_error & error) {
    record(error);
  }
  buffer_.clear();
}

void Transport::record(const std::system_error & error)
{
  if (!error_) {
    error_ = error;
  }
}

// --- I2C ---

I2cTransport::I2cTransport(const std::string & device, int address)
{
  char name[64];
  std::snprintf(name, sizeof(name), "%s@0x%02x", device.c_str(), address);
  name_ = name;
  fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw errno_error("open " + device);
  }
  if (::ioctl(fd_, I2C_SLAVE, address) < 0) {
    const auto error = errno_error("select " + name_);
    ::close(fd_);
    throw error;
  }
}

I2cTransport::~I2cTransport()
{
  ::close(fd_);
}

void I2cTransport::transfer(std::span<const std::uint8_t> data)
{
  // u8g2's I2C drivers put the control byte (command or data) in the transfer themselves
  write_all(fd_, data, name_);
}

// --- SPI ---

/// Output lines of one GPIO chip (libgpiod v2).
class SpiTransport::Gpio
{
public:
  Gpio(const std::string & chip_path, const std::vector<unsigned int> & offsets)
  {
    chip_ = gpiod_chip_open(chip_path.c_str());
    if (chip_ == nullptr) {
      throw errno_error("open " + chip_path);
    }
    auto * settings = gpiod_line_settings_new();
    auto * line_config = gpiod_line_config_new();
    auto * request_config = gpiod_request_config_new();
    bool configured = settings != nullptr && line_config != nullptr && request_config != nullptr;
    if (configured) {
      gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
      // High: reset released
      gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_ACTIVE);
      gpiod_request_config_set_consumer(request_config, "kvn_display");
      configured = gpiod_line_config_add_line_settings(
        line_config, offsets.data(), offsets.size(), settings) == 0;
    }
    if (configured) {
      request_ = gpiod_chip_request_lines(chip_, request_config, line_config);
    }
    const auto error = errno_error("request GPIO lines on " + chip_path);
    gpiod_request_config_free(request_config);
    gpiod_line_config_free(line_config);
    gpiod_line_settings_free(settings);
    if (request_ == nullptr) {
      gpiod_chip_close(chip_);
      throw error;
    }
  }

  ~Gpio()
  {
    gpiod_line_request_release(request_);
    gpiod_chip_close(chip_);
  }

  Gpio(const Gpio &) = delete;
  Gpio & operator=(const Gpio &) = delete;

  void set(unsigned int offset, bool high)
  {
    if (gpiod_line_request_set_value(
        request_, offset, high ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) < 0)
    {
      throw errno_error("set GPIO line " + std::to_string(offset));
    }
  }

private:
  gpiod_chip * chip_ = nullptr;
  gpiod_line_request * request_ = nullptr;
};

SpiTransport::SpiTransport(const SpiConfig & config)
: config_(config)
{
  if (config_.dc_line < 0) {
    throw std::system_error(EINVAL, std::generic_category(), "SPI displays need a dc_line");
  }
  std::vector<unsigned int> lines{static_cast<unsigned int>(config_.dc_line)};
  if (config_.reset_line >= 0) {
    lines.push_back(static_cast<unsigned int>(config_.reset_line));
  }
  fd_ = ::open(config_.device.c_str(), O_RDWR | O_CLOEXEC);
  if (fd_ < 0) {
    throw errno_error("open " + config_.device);
  }
  try {
    gpio_ = std::make_unique<Gpio>(config_.gpio_chip, lines);
  } catch (...) {
    ::close(fd_);
    throw;
  }
}

SpiTransport::~SpiTransport()
{
  ::close(fd_);
}

void SpiTransport::init(int spi_mode, std::uint32_t max_clock_hz)
{
  auto mode = static_cast<std::uint8_t>(spi_mode & (SPI_CPOL | SPI_CPHA));
  std::uint8_t bits = 8;
  std::uint32_t speed = config_.speed_hz > 0 ? config_.speed_hz : max_clock_hz;
  if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
    ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
  {
    record(errno_error("configure " + config_.device));
  }
}

void SpiTransport::set_dc(bool data)
{
  // Bytes already collected were sent with the previous D/C level
  flush();
  try {
    gpio_->set(static_cast<unsigned int>(config_.dc_line), data);
  } catch (const std::system_error & error) {
    record(error);
  }
}

void SpiTransport::set_reset(bool level)
{
  if (config_.reset_line < 0) {
    return;
  }
  try {
    gpio_->set(static_cast<unsigned int>(config_.reset_line), level);
  } catch (const std::system_error & error) {
    record(error);
  }
}

void SpiTransport::transfer(std::span<const std::uint8_t> data)
{
  // spidev limits one write to its buffer size (4096 bytes by default)
  constexpr std::size_t kChunk = 4096;
  for (std::size_t offset = 0; offset < data.size(); offset += kChunk) {
    write_all(fd_, data.subspan(offset, std::min(kChunk, data.size() - offset)), config_.device);
  }
}

}  // namespace kvn_display
