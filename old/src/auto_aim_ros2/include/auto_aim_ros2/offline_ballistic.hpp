#ifndef AUTO_AIM_ROS2__OFFLINE_BALLISTIC_HPP_
#define AUTO_AIM_ROS2__OFFLINE_BALLISTIC_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "auto_aim_ros2/offline_pipeline.hpp"

namespace rm_auto_aim::offline
{

// These reasons are evidence only.  They intentionally do not map to an
// AimCommand, RobotCtrl message, or a hardware action.
enum class BallisticFailureReason : std::uint8_t
{
  None = 0,
  Disabled,
  NoTarget,
  InvalidTrack,
  NotTracking,
  InvalidObservation,
  NegativeTimestamp,
  TimestampMismatch,
  MissingMuzzleTransform,
  MissingGimbalPose,
  MissingBulletSpeed,
  InvalidBulletSpeed,
  MissingGravity,
  InvalidGravity,
  MissingSystemLatency,
  NegativeSystemLatency,
  InvalidMaximumFlightTime,
  InvalidMaximumPredictionHorizon,
  InvalidMinimumHorizontalDistance,
  NonFiniteTargetPosition,
  TargetBehindMuzzle,
  HorizontalDistanceTooSmall,
  NonFiniteDiscriminant,
  DiscriminantNegative,
  NonFinitePitch,
  NonFiniteFlightTime,
  FlightTimeUnrepresentable,
  FlightTimeOverflow,
  FlightTimeExceedsMaximum,
  HorizonOverflow,
  HorizonExceedsPredictionMaximum,
  NonFiniteResult,
};

const char * ballistic_failure_reason_name(BallisticFailureReason reason) noexcept;

// The core solver only accepts a target that is already expressed in the
// muzzle frame.  TestOnlyGimbalOrigin exists solely for the CLI diagnostic
// adapter and is never a substitute for a reviewed gimbal->muzzle transform.
enum class BallisticOriginAssumption : std::uint8_t
{
  NotEvaluated = 0,
  MuzzleFrame,
  TestOnlyGimbalOrigin,
};

const char * ballistic_origin_assumption_name(BallisticOriginAssumption assumption) noexcept;

// All timing inputs are integer nanoseconds so an offline replay has one
// exact time base.  Optional physical inputs make an omitted CLI argument
// distinguishable from an explicitly supplied zero, which must fail closed.
struct BallisticConfig
{
  bool enabled{false};
  std::optional<double> bullet_speed_mps;
  std::optional<double> gravity_mps2;
  std::optional<std::int64_t> system_latency_ns;
  std::int64_t max_flight_time_ns{500'000'000};
  std::int64_t max_prediction_horizon_ns{500'000'000};
  double min_horizontal_distance_m{1e-6};
  bool allow_test_gimbal_origin_as_muzzle{false};
};

// x is forward, y is left, and z is up.  The point must already be relative
// to the muzzle origin; callers may not silently reinterpret gimbal position
// as muzzle position.
struct BallisticMuzzleInput
{
  std::uint64_t track_id{0};
  std::int64_t source_stamp_ns{-1};
  cv::Vec3d target_muzzle_m{};
  // Require callers to make the frame-origin contract explicit.  A default
  // constructed input must never silently turn camera/gimbal coordinates
  // into muzzle coordinates.
  BallisticOriginAssumption origin_assumption{BallisticOriginAssumption::NotEvaluated};
};

struct BallisticResult
{
  bool enabled{false};
  bool valid{false};
  BallisticFailureReason failure_reason{BallisticFailureReason::Disabled};
  std::string reason;

  std::uint64_t track_id{0};
  std::int64_t source_stamp_ns{-1};
  std::optional<cv::Vec3d> target_muzzle_m;
  std::optional<double> horizontal_distance_m;
  std::optional<double> geometric_yaw_rad;
  std::optional<double> geometric_pitch_rad;
  std::optional<double> ballistic_yaw_rad;
  std::optional<double> ballistic_pitch_rad;
  std::optional<double> gravity_pitch_correction_rad;
  std::optional<double> flight_time_s;
  std::optional<std::int64_t> flight_time_ns;
  std::optional<std::int64_t> system_latency_ns;
  std::optional<std::int64_t> recommended_prediction_horizon_ns;
  std::optional<double> bullet_speed_mps;
  std::optional<double> gravity_mps2;
  BallisticOriginAssumption origin_assumption{BallisticOriginAssumption::NotEvaluated};

  // Immutable safety boundary for every outcome, including a valid analytic
  // solution.  A valid result is diagnostic geometry, never a firing claim.
  bool test_only{true};
  bool production_ready{false};
  bool ballistic_control_applied{false};
};

// Solves the drag-free low-arc trajectory independently of tracking, PnP,
// Selector, Aimer, ROS, and RobotCtrl.  It accepts only muzzle-frame input.
class OfflineBallisticSolver final
{
public:
  explicit OfflineBallisticSolver(BallisticConfig config = {});

  BallisticResult solve(const BallisticMuzzleInput & input) const noexcept;
  const BallisticConfig & config() const noexcept;

private:
  BallisticConfig config_;
};

// This small adapter permits a future caller to supply a verified muzzle-frame
// target.  The current offline CLI has only a test-only gimbal position; it
// may use that position only when the explicit test-only flag is enabled.
class OfflineBallisticDiagnostic final
{
public:
  explicit OfflineBallisticDiagnostic(BallisticConfig config = {});

  BallisticResult diagnose(
    const std::optional<TrackedTarget> & selected,
    std::int64_t frame_stamp_ns,
    const std::optional<cv::Vec3d> & target_muzzle_m = std::nullopt) const noexcept;
  const BallisticConfig & config() const noexcept;

private:
  BallisticConfig config_;
};

}  // namespace rm_auto_aim::offline

#endif  // AUTO_AIM_ROS2__OFFLINE_BALLISTIC_HPP_
