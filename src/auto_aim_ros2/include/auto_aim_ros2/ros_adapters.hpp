#ifndef AUTO_AIM_ROS2__ROS_ADAPTERS_HPP_
#define AUTO_AIM_ROS2__ROS_ADAPTERS_HPP_

#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "control_interface_constraints.hpp"
#include "auto_aim_ros2/angle_units.hpp"
#include "auto_aim_ros2/auto_aim_core.hpp"

#include <cmath>
#include <optional>

namespace rm_auto_aim::ros_adapters
{

inline std::optional<pipeline::VisionState> to_algorithm_vision(
  const auto_aim_interfaces::msg::Vision & message)
{
  // Vision.msg and VisionData carry positions in degree and velocities in
  // degree/s.  Convert only at this ROS/algorithm boundary; do not repeat
  // conversion in the serial bridge.
  // A Vision message is a stamped sensor sample, not a duration.  Reject
  // negative, unset, or non-canonical ROS time values rather than allowing an
  // invalid timestamp to participate in freshness or replay decisions later.
  if (message.header.stamp.sec < 0 ||
    message.header.stamp.nanosec >= 1'000'000'000U ||
    (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0))
  {
    return std::nullopt;
  }
  if (!units::finite(message.yaw) || !units::finite(message.pitch) ||
    !units::finite(message.roll) || !units::finite(message.yaw_vel) ||
    !units::finite(message.pitch_vel) || !units::finite(message.shoot_speed))
  {
    return std::nullopt;
  }

  pipeline::VisionState state{};
  state.stamp_ns = static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
    static_cast<std::int64_t>(message.header.stamp.nanosec);
  state.frame_id = message.header.frame_id;
  state.id = message.id;
  state.mode = message.mode;
  state.yaw_rad = units::degrees_to_radians(message.yaw);
  state.yaw_vel_rad_s = units::degrees_per_second_to_radians_per_second(message.yaw_vel);
  state.pitch_rad = units::degrees_to_radians(message.pitch);
  state.pitch_vel_rad_s =
    units::degrees_per_second_to_radians_per_second(message.pitch_vel);
  state.roll_rad = units::degrees_to_radians(message.roll);
  state.shoot_speed_mps = message.shoot_speed;
  state.bullet_count = message.bullet_count;
  state.game_progress = message.game_progress;
  for (std::size_t index = 0; index < state.quaternion_wxyz.size(); ++index) {
    if (!units::finite(message.quaternion[index])) {
      return std::nullopt;
    }
    state.quaternion_wxyz[index] = message.quaternion[index];
  }
  if (!units::finite(state.yaw_rad) || !units::finite(state.yaw_vel_rad_s) ||
    !units::finite(state.pitch_rad) || !units::finite(state.pitch_vel_rad_s) ||
    !units::finite(state.roll_rad))
  {
    return std::nullopt;
  }
  // Converted Vision velocity is retained only for diagnostic completeness.
  // It must not cross into Tracker, Aimer, RobotCtrl, quaternion handling,
  // or time-alignment control decisions in this phase.
  return state;
}

struct RobotCtrlAdapterResult
{
  auto_aim_interfaces::msg::RobotCtrl message{};
  auto_aim_interfaces::control::PositionConstraint position_constraint{};
  bool valid_command{false};

  bool accepted() const noexcept
  {
    return valid_command && position_constraint.accepted();
  }
};

inline auto to_ros_with_profile(
  const pipeline::AimCommand & command,
  const auto_aim_interfaces::control::VehicleProfile vehicle_profile)
  -> RobotCtrlAdapterResult
{
  RobotCtrlAdapterResult result{};
  auto & message = result.message;
  message.target_lock = pipeline::kTargetUnlocked;
  message.fire_command = pipeline::kFireNone;
  const bool valid_target_lock = command.target_lock == pipeline::kTargetLocked ||
    command.target_lock == pipeline::kTargetUnlocked;
  const bool valid_fire_command = command.fire_command == pipeline::kFireNone ||
    command.fire_command == pipeline::kFireBurst ||
    command.fire_command == pipeline::kFireSingle;
  const bool fire_requires_lock = command.fire_command != pipeline::kFireNone &&
    command.target_lock != pipeline::kTargetLocked;
  const bool finite_angles = units::finite(command.yaw_rad) && units::finite(command.pitch_rad);

  // RobotCtrl is a hardware-facing boundary.  An invalid enum or non-finite
  // angle is never allowed to leak through as a partially valid command:
  // unlock, zero positions, and inhibit firing as one atomic safe result.
  if (!valid_target_lock || !valid_fire_command || fire_requires_lock || !finite_angles) {
    message.target_lock = pipeline::kTargetUnlocked;
    message.fire_command = pipeline::kFireNone;
    return result;
  }

  // AimCommand is internal rad.  RobotCtrl.msg/RobotCtrlData are degree for
  // position angles, so convert exactly once at this output boundary and
  // apply the shared yaw/pitch safety contract in external units.
  const auto yaw_degree = units::radians_to_degrees(command.yaw_rad);
  const auto pitch_degree = units::radians_to_degrees(command.pitch_rad);
  result.position_constraint = auto_aim_interfaces::control::constrain_position_degrees(
    yaw_degree, pitch_degree, vehicle_profile);
  if (!result.position_constraint.accepted()) {
    return result;
  }
  message.yaw = result.position_constraint.yaw_degree;
  message.pitch = result.position_constraint.pitch_degree;
  // The units are confirmed (degree/s and degree/s^2), but MCU feedforward
  // control semantics are not.  Never send internal rad/s or rad/s^2 values
  // over ROS/serial in this phase: all four hardware-facing fields stay zero.
  message.yaw_vel = 0.0F;
  message.yaw_acc = 0.0F;
  message.pitch_vel = 0.0F;
  message.pitch_acc = 0.0F;
  message.target_lock = command.target_lock;
  message.fire_command = command.fire_command;
  result.valid_command = true;
  return result;
}

inline auto to_ros(
  const pipeline::AimCommand & command,
  const auto_aim_interfaces::control::VehicleProfile vehicle_profile =
    auto_aim_interfaces::control::VehicleProfile::Unselected)
  -> auto_aim_interfaces::msg::RobotCtrl
{
  return to_ros_with_profile(command, vehicle_profile).message;
}

inline pipeline::AimCommand force_dry_run_safe(pipeline::AimCommand command) noexcept
{
  command.fire_command = pipeline::kFireNone;
  return command;
}

}  // namespace rm_auto_aim::ros_adapters

#endif  // AUTO_AIM_ROS2__ROS_ADAPTERS_HPP_
