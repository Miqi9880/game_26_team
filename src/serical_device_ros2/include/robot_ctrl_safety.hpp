#ifndef ROBOMASTER_ROBOT_CTRL_SAFETY_H
#define ROBOMASTER_ROBOT_CTRL_SAFETY_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <utility>

#include "protocol_new.hpp"

namespace rm_auto_aim::safety
{

using ClockFn = std::function<std::int64_t()>;

constexpr std::int8_t kTargetLocked = 49;
constexpr std::int8_t kTargetUnlocked = 50;
constexpr std::int8_t kFireNone = 0;
constexpr std::int64_t kTimeoutNs = 100'000'000;

struct Config
{
  bool allow_fire{false};
  std::int8_t burst_command{1};
  std::int8_t single_command{2};
};

struct Output
{
  io::RobotCtrlData control{};
  bool fresh{false};
};

/**
 * Small, ROS-independent state machine for the RobotCtrl timeout policy.
 *
 * A message is accepted only when every field is finite and the protocol
 * enumerations are known.  The clock is injected so the exact timeout
 * boundary can be tested without a ROS clock or a serial device.
 */
class RobotCtrlSafety
{
public:
  explicit RobotCtrlSafety(Config config, ClockFn clock = {})
  : config_(config), clock_(std::move(clock))
  {
    if (!clock_) {
      clock_ = []() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      };
    }
  }

  /** Accept and remember a complete valid command. */
  bool Accept(const io::RobotCtrlData & input)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!IsValid(input, config_)) {
      // An invalid message must not leave a previously valid firing command
      // active for the timeout window. Keep the last finite position as a
      // hold point, but immediately replace motion/lock/fire with safe values.
      const auto held_yaw = std::isfinite(last_valid_.yaw) ? last_valid_.yaw : 0.0f;
      const auto held_pitch = std::isfinite(last_valid_.pitch) ? last_valid_.pitch : 0.0f;
      last_valid_ = io::RobotCtrlData{};
      last_valid_.yaw = held_yaw;
      last_valid_.pitch = held_pitch;
      last_valid_.target_lock = kTargetUnlocked;
      last_valid_.fire_command = kFireNone;
      force_stale_ = true;
      return false;
    }

    last_valid_ = input;
    last_valid_ns_ = clock_();
    have_valid_ = true;
    force_stale_ = false;
    return true;
  }

  /** Generate the command for the current timer tick. */
  Output Tick() const
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    Output output{};
    const std::int64_t now_ns = clock_();

    const bool fresh = have_valid_ && !force_stale_ && now_ns >= last_valid_ns_ &&
      (now_ns - last_valid_ns_) < kTimeoutNs;
    output.fresh = fresh;

    if (fresh) {
      output.control = last_valid_;
      if (!config_.allow_fire || output.control.target_lock != kTargetLocked) {
        output.control.fire_command = kFireNone;
      }
      return output;
    }

    // Fail safe on no input, timeout, or a monotonic-clock rollback.  Keep
    // only the most recent valid yaw/pitch to avoid a discontinuous zero jump.
    if (have_valid_) {
      output.control.yaw = last_valid_.yaw;
      output.control.pitch = last_valid_.pitch;
    }
    output.control.yaw_vel = 0.0f;
    output.control.yaw_acc = 0.0f;
    output.control.pitch_vel = 0.0f;
    output.control.pitch_acc = 0.0f;
    output.control.target_lock = kTargetUnlocked;
    output.control.fire_command = kFireNone;
    return output;
  }

  static bool IsValid(const io::RobotCtrlData & input, const Config & config) noexcept
  {
    return std::isfinite(input.yaw) && std::isfinite(input.yaw_vel) &&
           std::isfinite(input.yaw_acc) && std::isfinite(input.pitch) &&
           std::isfinite(input.pitch_vel) && std::isfinite(input.pitch_acc) &&
           (input.target_lock == kTargetLocked || input.target_lock == kTargetUnlocked) &&
           (input.fire_command == kFireNone ||
            input.fire_command == config.burst_command ||
            input.fire_command == config.single_command) &&
           (input.target_lock == kTargetLocked || input.fire_command == kFireNone);
  }

private:
  mutable std::mutex mutex_;
  Config config_;
  ClockFn clock_;
  io::RobotCtrlData last_valid_{};
  std::int64_t last_valid_ns_{0};
  bool have_valid_{false};
  bool force_stale_{false};
};

}  // namespace rm_auto_aim::safety

#endif  // ROBOMASTER_ROBOT_CTRL_SAFETY_H
