// Copyright (c) 2026, Alessio Morale
// Licensed under the MIT License.
#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <sensor_msgs/msg/joy.hpp>

#include "kvn_orchestrator/ros_context.hpp"

namespace kvn_orchestrator
{

/// Watches a set of Joy buttons. A button index beyond the message's button array counts as not
/// pressed, so short or empty Joy messages are safe.
class ButtonTracker
{
public:
  ButtonTracker(
    RosContext & ctx, const std::string & topic, std::vector<int> buttons, double timeout_s)
  : buttons_(std::move(buttons)), timeout_s_(timeout_s)
  {
    topic_ = ctx.subscribe<sensor_msgs::msg::Joy>(topic, nav2::qos::StandardTopicQoS());
    listener_ = topic_->listen([this](const sensor_msgs::msg::Joy & msg) {on_joy(msg);});
    if (topic_->latest()) {
      update(*topic_->latest());
    }
  }

  /// A Joy message arrived within the timeout.
  bool fresh() const {return topic_->fresh(timeout_s_);}

  /// Every tracked button is pressed in the latest message.
  bool all_pressed() const {return all_pressed_;}

  /// No tracked button is pressed in the latest message.
  bool none_pressed() const {return none_pressed_;}

  /// Seconds since all tracked buttons became pressed together (0 when they are not).
  double held_s() const
  {
    if (!pressed_since_) {
      return 0.0;
    }
    return std::chrono::duration<double>(SteadyClock::now() - *pressed_since_).count();
  }

  /// True if any message since the last call showed the buttons not all pressed.
  bool take_release()
  {
    const bool released = released_;
    released_ = false;
    return released;
  }

private:
  void on_joy(const sensor_msgs::msg::Joy & msg)
  {
    update(msg);
    if (!all_pressed_) {
      released_ = true;
    }
  }

  void update(const sensor_msgs::msg::Joy & msg)
  {
    const auto pressed = [&msg](int index) {
        return index >= 0 && static_cast<size_t>(index) < msg.buttons.size() &&
               msg.buttons[static_cast<size_t>(index)] != 0;
      };
    all_pressed_ = !buttons_.empty() && std::all_of(buttons_.begin(), buttons_.end(), pressed);
    none_pressed_ = std::none_of(buttons_.begin(), buttons_.end(), pressed);
    if (!all_pressed_) {
      pressed_since_.reset();
    } else if (!pressed_since_) {
      pressed_since_ = SteadyClock::now();
    }
  }

  std::vector<int> buttons_;
  double timeout_s_;
  std::shared_ptr<Topic<sensor_msgs::msg::Joy>> topic_;
  std::shared_ptr<Topic<sensor_msgs::msg::Joy>::Listener> listener_;
  bool all_pressed_{false};
  bool none_pressed_{true};
  bool released_{false};
  std::optional<SteadyClock::time_point> pressed_since_;
};

}  // namespace kvn_orchestrator
