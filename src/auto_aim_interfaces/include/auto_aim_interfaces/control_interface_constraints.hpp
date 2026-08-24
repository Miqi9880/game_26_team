#ifndef AUTO_AIM_INTERFACES__CONTROL_INTERFACE_CONSTRAINTS_HPP_
#define AUTO_AIM_INTERFACES__CONTROL_INTERFACE_CONSTRAINTS_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>

namespace auto_aim_interfaces::control
{

// The vehicle profile is intentionally an explicit configuration choice.  A
// missing profile is not a permissive "generic" range: it must fail closed
// before a RobotCtrl position can become a hardware command.
enum class VehicleProfile : std::uint8_t
{
  Unselected = 0,
  NewTurtle,
  DogLeg,
};

struct PitchLimitsDegrees
{
  float minimum{0.0F};
  float maximum{0.0F};
};

enum class ConstraintStatus : std::uint8_t
{
  Accepted = 0,
  YawWrapped,
  PitchClamped,
  YawWrappedAndPitchClamped,
  ProfileUnselected,
  NonFiniteInput,
};

struct PositionConstraint
{
  float yaw_degree{0.0F};
  float pitch_degree{0.0F};
  ConstraintStatus status{ConstraintStatus::NonFiniteInput};

  bool accepted() const noexcept
  {
    return status == ConstraintStatus::Accepted ||
           status == ConstraintStatus::YawWrapped ||
           status == ConstraintStatus::PitchClamped ||
           status == ConstraintStatus::YawWrappedAndPitchClamped;
  }

  bool yaw_wrapped() const noexcept
  {
    return status == ConstraintStatus::YawWrapped ||
           status == ConstraintStatus::YawWrappedAndPitchClamped;
  }

  bool pitch_clamped() const noexcept
  {
    return status == ConstraintStatus::PitchClamped ||
           status == ConstraintStatus::YawWrappedAndPitchClamped;
  }
};

inline constexpr const char * vehicle_profile_name(VehicleProfile profile) noexcept
{
  switch (profile) {
    case VehicleProfile::Unselected:
      return "unselected";
    case VehicleProfile::NewTurtle:
      return "new_turtle";
    case VehicleProfile::DogLeg:
      return "dog_leg";
  }
  return "unselected";
}

inline std::optional<VehicleProfile> parse_vehicle_profile(
  std::string_view name) noexcept
{
  if (name == "unselected") {
    return VehicleProfile::Unselected;
  }
  if (name == "new_turtle") {
    return VehicleProfile::NewTurtle;
  }
  if (name == "dog_leg") {
    return VehicleProfile::DogLeg;
  }
  return std::nullopt;
}

inline std::optional<PitchLimitsDegrees> pitch_limits_degrees(
  VehicleProfile profile) noexcept
{
  switch (profile) {
    case VehicleProfile::NewTurtle:
      return PitchLimitsDegrees{-20.0F, 19.0F};
    case VehicleProfile::DogLeg:
      return PitchLimitsDegrees{-10.0F, 31.0F};
    case VehicleProfile::Unselected:
      return std::nullopt;
  }
  return std::nullopt;
}

inline constexpr const char * constraint_status_name(ConstraintStatus status) noexcept
{
  switch (status) {
    case ConstraintStatus::Accepted:
      return "accepted";
    case ConstraintStatus::YawWrapped:
      return "yaw_wrapped";
    case ConstraintStatus::PitchClamped:
      return "pitch_clamped";
    case ConstraintStatus::YawWrappedAndPitchClamped:
      return "yaw_wrapped_and_pitch_clamped";
    case ConstraintStatus::ProfileUnselected:
      return "profile_unselected";
    case ConstraintStatus::NonFiniteInput:
      return "non_finite_input";
  }
  return "non_finite_input";
}

// Canonical range is inclusive [-180, 180].  Exact +/-180 values keep their
// sign; +540 becomes +180 and -540 becomes -180.  Returning nullopt keeps
// NaN/Inf from being normalized into an apparently valid command.
inline std::optional<float> wrap_yaw_degrees(float yaw_degree) noexcept
{
  if (!std::isfinite(yaw_degree)) {
    return std::nullopt;
  }

  float wrapped = std::fmod(yaw_degree, 360.0F);
  if (wrapped > 180.0F) {
    wrapped -= 360.0F;
  } else if (wrapped < -180.0F) {
    wrapped += 360.0F;
  }
  return wrapped;
}

// This is a visual-side pre-limit and diagnostic boundary.  The lower-level
// controller remains the final mechanical pitch-limit protection.
inline PositionConstraint constrain_position_degrees(
  float yaw_degree,
  float pitch_degree,
  VehicleProfile profile) noexcept
{
  PositionConstraint result{};
  if (!std::isfinite(yaw_degree) || !std::isfinite(pitch_degree)) {
    result.status = ConstraintStatus::NonFiniteInput;
    return result;
  }

  const auto limits = pitch_limits_degrees(profile);
  if (!limits.has_value()) {
    result.status = ConstraintStatus::ProfileUnselected;
    return result;
  }

  const auto wrapped_yaw = wrap_yaw_degrees(yaw_degree);
  if (!wrapped_yaw.has_value()) {
    result.status = ConstraintStatus::NonFiniteInput;
    return result;
  }

  result.yaw_degree = *wrapped_yaw;
  result.pitch_degree = std::clamp(pitch_degree, limits->minimum, limits->maximum);
  const bool yaw_changed = result.yaw_degree != yaw_degree;
  const bool pitch_changed = result.pitch_degree != pitch_degree;
  if (yaw_changed && pitch_changed) {
    result.status = ConstraintStatus::YawWrappedAndPitchClamped;
  } else if (yaw_changed) {
    result.status = ConstraintStatus::YawWrapped;
  } else if (pitch_changed) {
    result.status = ConstraintStatus::PitchClamped;
  } else {
    result.status = ConstraintStatus::Accepted;
  }
  return result;
}

}  // namespace auto_aim_interfaces::control

#endif  // AUTO_AIM_INTERFACES__CONTROL_INTERFACE_CONSTRAINTS_HPP_
