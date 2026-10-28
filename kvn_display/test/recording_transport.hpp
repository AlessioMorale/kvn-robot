// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <cstdint>
#include <memory>
#include <system_error>
#include <vector>

#include "kvn_display/transport.hpp"

namespace kvn_display
{

/// What a RecordingTransport saw; shared so it outlives the transport owned by a Display.
struct TransportLog
{
  std::vector<std::vector<std::uint8_t>> transfers;
  std::vector<bool> dc;
  std::vector<bool> reset;
  int spi_mode = -1;
  std::uint32_t max_clock_hz = 0;
  bool fail = false;  ///< Make every transfer fail
};

class RecordingTransport : public Transport
{
public:
  explicit RecordingTransport(std::shared_ptr<TransportLog> log)
  : log_(std::move(log)) {}

  void init(int spi_mode, std::uint32_t max_clock_hz) override
  {
    log_->spi_mode = spi_mode;
    log_->max_clock_hz = max_clock_hz;
  }
  void set_dc(bool data) override
  {
    flush();
    log_->dc.push_back(data);
  }
  void set_reset(bool level) override {log_->reset.push_back(level);}

protected:
  void transfer(std::span<const std::uint8_t> data) override
  {
    if (log_->fail) {
      throw std::system_error(std::make_error_code(std::errc::io_error), "fake bus");
    }
    log_->transfers.emplace_back(data.begin(), data.end());
  }

private:
  std::shared_ptr<TransportLog> log_;
};

}  // namespace kvn_display
