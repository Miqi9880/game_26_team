#ifndef ROBOMASTER_ROBOT_CTRL_SAFETY_H
#define ROBOMASTER_ROBOT_CTRL_SAFETY_H

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <utility>

#include "control_interface_constraints.hpp"
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
  auto_aim_interfaces::control::VehicleProfile vehicle_profile{
    auto_aim_interfaces::control::VehicleProfile::Unselected};
  std::int64_t input_timeout_ns{kTimeoutNs};
};

struct Output
{
  io::RobotCtrlData control{};
  bool fresh{false};
  bool yaw_wrapped{false};
  bool pitch_clamped{false};
};

struct ControlConstraintResult
{
  io::RobotCtrlData control{};
  auto_aim_interfaces::control::PositionConstraint position{};
  bool valid_input{false};

  bool accepted() const noexcept
  {
    return valid_input && position.accepted();
  }
};

// Compatibility wrapper for callers in this package.  The implementation is
// shared with AutoAimNode so both publishers round parameterized Hz values in
// exactly the same way.
inline std::optional<std::chrono::nanoseconds> control_period_from_hz(
  double output_hz) noexcept
{
  return auto_aim_interfaces::control::control_period_from_hz(output_hz);
}

/**
 * Small, ROS-independent state machine for the RobotCtrl timeout policy.
 *
 * A message is accepted only when every field is finite, the protocol
 * enumerations are known, and a vehicle profile is explicitly selected.  The
 * final serial-side safety boundary canonicalizes yaw to [-180, 180], applies
 * the selected pitch pre-limit, and always zeroes motion feedforward.  The
 * clock is injected so the exact timeout boundary can be tested without a
 * ROS clock or a serial device.
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
    const auto constrained = Constrain(input, config_);
    if (!constrained.accepted()) {
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

    last_valid_ = constrained.control;
    last_yaw_wrapped_ = constrained.position.yaw_wrapped();
    last_pitch_clamped_ = constrained.position.pitch_clamped();
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
      (now_ns - last_valid_ns_) < config_.input_timeout_ns;
    output.fresh = fresh;

    if (fresh) {
      output.control = last_valid_;
      output.yaw_wrapped = last_yaw_wrapped_;
      output.pitch_clamped = last_pitch_clamped_;
      // Fire semantics remain unconfirmed; this branch never emits a fire
      // command, including when a direct ROS publisher bypasses AutoAimNode.
      output.control.fire_command = kFireNone;
      zero_motion_fields(&output.control);
      return output;
    }

    // Fail safe on no input, timeout, or a monotonic-clock rollback.  Keep
    // only the most recent valid yaw/pitch to avoid a discontinuous zero jump.
    if (have_valid_) {
      output.control.yaw = last_valid_.yaw;
      output.control.pitch = last_valid_.pitch;
    }
    zero_motion_fields(&output.control);
    output.control.target_lock = kTargetUnlocked;
    output.control.fire_command = kFireNone;
    return output;
  }

  static ControlConstraintResult Constrain(
    const io::RobotCtrlData & input, const Config & config) noexcept
  {
    ControlConstraintResult result{};
    if (!std::isfinite(input.yaw) || !std::isfinite(input.yaw_vel) ||
      !std::isfinite(input.yaw_acc) || !std::isfinite(input.pitch) ||
      !std::isfinite(input.pitch_vel) || !std::isfinite(input.pitch_acc) ||
      config.input_timeout_ns <= 0 ||
      (input.target_lock != kTargetLocked && input.target_lock != kTargetUnlocked) ||
      (input.fire_command != kFireNone && input.fire_command != config.burst_command &&
      input.fire_command != config.single_command) ||
      (input.target_lock != kTargetLocked && input.fire_command != kFireNone))
    {
      result.position.status = auto_aim_interfaces::control::ConstraintStatus::NonFiniteInput;
      return result;
    }

    result.valid_input = true;
    result.position = auto_aim_interfaces::control::constrain_position_degrees(
      input.yaw, input.pitch, config.vehicle_profile);
    if (!result.position.accepted()) {
      return result;
    }

    result.control = input;
    result.control.yaw = result.position.yaw_degree;
    result.control.pitch = result.position.pitch_degree;
    zero_motion_fields(&result.control);
    // Fire is disabled in this control-interface phase regardless of input.
    result.control.fire_command = kFireNone;
    return result;
  }

  static bool IsValid(const io::RobotCtrlData & input, const Config & config) noexcept
  {
    return Constrain(input, config).accepted();
  }

private:
  static void zero_motion_fields(io::RobotCtrlData * control) noexcept
  {
    control->yaw_vel = 0.0F;
    control->yaw_acc = 0.0F;
    control->pitch_vel = 0.0F;
    control->pitch_acc = 0.0F;
  }

  mutable std::mutex mutex_;
  Config config_;
  ClockFn clock_;
  io::RobotCtrlData last_valid_{};
  std::int64_t last_valid_ns_{0};
  bool have_valid_{false};
  bool force_stale_{false};
  bool last_yaw_wrapped_{false};
  bool last_pitch_clamped_{false};
};

}  // namespace rm_auto_aim::safety

#endif  // ROBOMASTER_ROBOT_CTRL_SAFETY_H
