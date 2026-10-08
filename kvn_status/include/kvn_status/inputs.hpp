// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "kvn_status/status_reducer.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kvn_status
{

/// Everything an input needs to configure itself and report updates.
struct InputContext
{
  rclcpp::Node * node{nullptr};
  /// Input name from the `inputs` list; its parameters are under `<name>.`
  std::string name;
  /// Monotonic time source shared by all inputs.
  std::function<TimePoint()> now;
  /// Called after each fresh reading so the node can publish a change at once.
  std::function<void()> on_update;
};

/// Common parameters of every input (`<name>.timeout_s`, `<name>.code`, ...).
struct InputCommonConfig
{
  std::string topic;
  std::string code;
  std::string stale_code;
  Duration timeout{};
};

/**
 * @brief Base class of a status input: one subscription, one InputMonitor.
 *
 * To add an input type, derive from this class, update monitor_ from the
 * subscription callback (then call notify()), and register a factory in
 * input_factories() (inputs.cpp).
 */
class StatusInput
{
public:
  StatusInput(const InputContext & context, const InputCommonConfig & config);
  virtual ~StatusInput() = default;

  StatusInput(const StatusInput &) = delete;
  StatusInput & operator=(const StatusInput &) = delete;

  const std::string & name() const { return name_; }

  /// Condition of this input (stale → FAULT with the stale code).
  Condition evaluate(TimePoint now) const { return monitor_.evaluate(now); }

  /// Operating state contributed by this input, if it has one (e.g. joy enable).
  virtual std::optional<OperatingState> operating_state(TimePoint /*now*/) const
  {
    return std::nullopt;
  }

protected:
  /// Record a fresh reading and notify the node.
  void report(Severity severity, const std::string & code);

  std::string name_;
  InputCommonConfig config_;
  InputMonitor monitor_;
  std::function<TimePoint()> now_;
  std::function<void()> on_update_;
};

/// Reads the common parameters, with @p default_topic for `<name>.topic`.
InputCommonConfig declare_common_parameters(
  const InputContext & context, const std::string & default_topic, double default_timeout_s);

using InputFactory = std::function<std::unique_ptr<StatusInput>(const InputContext &)>;

/// Registered input types by `<name>.type` value.
const std::map<std::string, InputFactory> & input_factories();

/// Create an input of @p type; throws std::invalid_argument for an unknown type.
std::unique_ptr<StatusInput> make_input(const std::string & type, const InputContext & context);

}  // namespace kvn_status
