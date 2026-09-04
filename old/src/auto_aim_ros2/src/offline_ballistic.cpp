#include "auto_aim_ros2/offline_ballistic.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace rm_auto_aim::offline
{
namespace
{

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

bool finite_vector(const cv::Vec3d & value) noexcept
{
  return finite(value[0]) && finite(value[1]) && finite(value[2]);
}

BallisticResult make_result(const BallisticConfig & config)
{
  BallisticResult result{};
  result.enabled = config.enabled;
  // Do not allow a direct C++ caller to carry NaN/Inf configuration values
  // into a serializable evidence result.  The later configuration checks
  // still retain their specific fail-closed reason.
  if (config.bullet_speed_mps.has_value() && finite(*config.bullet_speed_mps)) {
    result.bullet_speed_mps = config.bullet_speed_mps;
  }
  if (config.gravity_mps2.has_value() && finite(*config.gravity_mps2)) {
    result.gravity_mps2 = config.gravity_mps2;
  }
  result.system_latency_ns = config.system_latency_ns;
  return result;
}

void fail(
  BallisticResult & result,
  BallisticFailureReason reason,
  const char * message)
{
  result.valid = false;
  result.failure_reason = reason;
  result.reason = message;
}

bool has_valid_common_configuration(const BallisticConfig & config, BallisticResult & result)
{
  if (!config.bullet_speed_mps.has_value()) {
    fail(
      result, BallisticFailureReason::MissingBulletSpeed,
      "ballistic diagnostic requires an explicit bullet speed");
    return false;
  }
  if (!finite(*config.bullet_speed_mps) || *config.bullet_speed_mps <= 0.0) {
    fail(
      result, BallisticFailureReason::InvalidBulletSpeed,
      "bullet speed must be finite and greater than zero m/s");
    return false;
  }
  if (!config.gravity_mps2.has_value()) {
    fail(
      result, BallisticFailureReason::MissingGravity,
      "ballistic diagnostic requires an explicit gravity magnitude");
    return false;
  }
  if (!finite(*config.gravity_mps2) || *config.gravity_mps2 <= 0.0) {
    fail(
      result, BallisticFailureReason::InvalidGravity,
      "gravity magnitude must be finite and greater than zero m/s^2");
    return false;
  }
  if (!config.system_latency_ns.has_value()) {
    fail(
      result, BallisticFailureReason::MissingSystemLatency,
      "ballistic diagnostic requires an explicit system latency");
    return false;
  }
  if (*config.system_latency_ns < 0) {
    fail(
      result, BallisticFailureReason::NegativeSystemLatency,
      "system latency must not be negative nanoseconds");
    return false;
  }
  if (config.max_flight_time_ns <= 0) {
    fail(
      result, BallisticFailureReason::InvalidMaximumFlightTime,
      "maximum flight time must be positive nanoseconds");
    return false;
  }
  if (config.max_prediction_horizon_ns < 0) {
    fail(
      result, BallisticFailureReason::InvalidMaximumPredictionHorizon,
      "maximum prediction horizon must not be negative nanoseconds");
    return false;
  }
  if (!finite(config.min_horizontal_distance_m) || config.min_horizontal_distance_m <= 0.0) {
    fail(
      result, BallisticFailureReason::InvalidMinimumHorizontalDistance,
      "minimum horizontal distance must be finite and greater than zero metres");
    return false;
  }
  return true;
}

}  // namespace

const char * ballistic_failure_reason_name(BallisticFailureReason reason) noexcept
{
  switch (reason) {
    case BallisticFailureReason::None:
      return "none";
    case BallisticFailureReason::Disabled:
      return "disabled";
    case BallisticFailureReason::NoTarget:
      return "no_target";
    case BallisticFailureReason::InvalidTrack:
      return "invalid_track";
    case BallisticFailureReason::NotTracking:
      return "not_tracking";
    case BallisticFailureReason::InvalidObservation:
      return "invalid_observation";
    case BallisticFailureReason::NegativeTimestamp:
      return "negative_timestamp";
    case BallisticFailureReason::TimestampMismatch:
      return "timestamp_mismatch";
    case BallisticFailureReason::MissingMuzzleTransform:
      return "missing_muzzle_transform";
    case BallisticFailureReason::MissingGimbalPose:
      return "missing_gimbal_pose";
    case BallisticFailureReason::MissingBulletSpeed:
      return "missing_bullet_speed";
    case BallisticFailureReason::InvalidBulletSpeed:
      return "invalid_bullet_speed";
    case BallisticFailureReason::MissingGravity:
      return "missing_gravity";
    case BallisticFailureReason::InvalidGravity:
      return "invalid_gravity";
    case BallisticFailureReason::MissingSystemLatency:
      return "missing_system_latency";
    case BallisticFailureReason::NegativeSystemLatency:
      return "negative_system_latency";
    case BallisticFailureReason::InvalidMaximumFlightTime:
      return "invalid_maximum_flight_time";
    case BallisticFailureReason::InvalidMaximumPredictionHorizon:
      return "invalid_maximum_prediction_horizon";
    case BallisticFailureReason::InvalidMinimumHorizontalDistance:
      return "invalid_minimum_horizontal_distance";
    case BallisticFailureReason::NonFiniteTargetPosition:
      return "non_finite_target_position";
    case BallisticFailureReason::TargetBehindMuzzle:
      return "target_behind_muzzle";
    case BallisticFailureReason::HorizontalDistanceTooSmall:
      return "horizontal_distance_too_small";
    case BallisticFailureReason::NonFiniteDiscriminant:
      return "non_finite_discriminant";
    case BallisticFailureReason::DiscriminantNegative:
      return "discriminant_negative";
    case BallisticFailureReason::NonFinitePitch:
      return "non_finite_pitch";
    case BallisticFailureReason::NonFiniteFlightTime:
      return "non_finite_flight_time";
    case BallisticFailureReason::FlightTimeUnrepresentable:
      return "flight_time_unrepresentable";
    case BallisticFailureReason::FlightTimeOverflow:
      return "flight_time_overflow";
    case BallisticFailureReason::FlightTimeExceedsMaximum:
      return "flight_time_exceeds_maximum";
    case BallisticFailureReason::HorizonOverflow:
      return "horizon_overflow";
    case BallisticFailureReason::HorizonExceedsPredictionMaximum:
      return "horizon_exceeds_prediction_maximum";
    case BallisticFailureReason::NonFiniteResult:
      return "non_finite_result";
  }
  return "unknown";
}

const char * ballistic_origin_assumption_name(BallisticOriginAssumption assumption) noexcept
{
  switch (assumption) {
    case BallisticOriginAssumption::NotEvaluated:
      return "not_evaluated";
    case BallisticOriginAssumption::MuzzleFrame:
      return "muzzle_frame";
    case BallisticOriginAssumption::TestOnlyGimbalOrigin:
      return "test_only_gimbal_origin";
  }
  return "unknown";
}

OfflineBallisticSolver::OfflineBallisticSolver(BallisticConfig config)
: config_(std::move(config)) {}

BallisticResult OfflineBallisticSolver::solve(const BallisticMuzzleInput & input) const noexcept
{
  BallisticResult result = make_result(config_);
  result.track_id = input.track_id;
  result.source_stamp_ns = input.source_stamp_ns;
  result.origin_assumption = input.origin_assumption;

  if (!config_.enabled) {
    fail(result, BallisticFailureReason::Disabled, "ballistic diagnostic is disabled");
    return result;
  }
  if (input.track_id == 0) {
    fail(result, BallisticFailureReason::InvalidTrack, "track id must be non-zero");
    return result;
  }
  if (input.source_stamp_ns < 0) {
    fail(
      result, BallisticFailureReason::NegativeTimestamp,
      "source timestamp must not be negative");
    return result;
  }
  if (input.origin_assumption != BallisticOriginAssumption::MuzzleFrame) {
    fail(
      result, BallisticFailureReason::MissingMuzzleTransform,
      "core solver accepts only a target declared in the muzzle frame");
    return result;
  }
  if (!has_valid_common_configuration(config_, result)) {
    return result;
  }
  if (!finite_vector(input.target_muzzle_m)) {
    fail(
      result, BallisticFailureReason::NonFiniteTargetPosition,
      "muzzle-frame target position must be finite");
    return result;
  }

  // Do not retain a non-finite point in a result that may be serialized as
  // evidence.  The finite check above is intentionally before this copy so a
  // fail-closed result can never emit NaN/Inf target coordinates.
  result.target_muzzle_m = input.target_muzzle_m;

  const double x = input.target_muzzle_m[0];
  const double y = input.target_muzzle_m[1];
  const double z = input.target_muzzle_m[2];
  const double horizontal_distance = std::hypot(x, y);
  result.horizontal_distance_m = horizontal_distance;
  if (!finite(horizontal_distance)) {
    fail(
      result, BallisticFailureReason::NonFiniteResult,
      "horizontal distance is not finite");
    return result;
  }
  if (horizontal_distance <= config_.min_horizontal_distance_m) {
    fail(
      result, BallisticFailureReason::HorizontalDistanceTooSmall,
      "horizontal distance is too small for a stable ballistic solution");
    return result;
  }
  if (x <= 0.0) {
    fail(
      result, BallisticFailureReason::TargetBehindMuzzle,
      "target must be in front of the muzzle frame");
    return result;
  }

  const double speed = *config_.bullet_speed_mps;
  const double gravity = *config_.gravity_mps2;
  const double speed_squared = speed * speed;
  const double range_squared = horizontal_distance * horizontal_distance;
  const double discriminant = speed_squared * speed_squared - gravity *
    (gravity * range_squared + 2.0 * z * speed_squared);
  if (!finite(discriminant)) {
    fail(
      result, BallisticFailureReason::NonFiniteDiscriminant,
      "drag-free trajectory discriminant is not finite");
    return result;
  }
  if (discriminant < 0.0) {
    fail(
      result, BallisticFailureReason::DiscriminantNegative,
      "target is unreachable by the drag-free low-arc model");
    return result;
  }

  const double root = std::sqrt(discriminant);
  // Rationalize the low-arc root to avoid catastrophic cancellation when the
  // gravity correction is small (the direct v^2 - sqrt(D) form can round to
  // zero even for a valid, non-zero correction).
  const double tangent_numerator = gravity * range_squared + 2.0 * z * speed_squared;
  const double tangent_denominator = horizontal_distance * (speed_squared + root);
  const double tangent_low_arc = tangent_numerator / tangent_denominator;
  const double geometric_yaw = std::atan2(y, x);
  const double geometric_pitch = std::atan2(z, horizontal_distance);
  const double ballistic_pitch = std::atan(tangent_low_arc);
  const double ballistic_yaw = geometric_yaw;
  const double gravity_correction = ballistic_pitch - geometric_pitch;
  if (!finite(root) || !finite(tangent_numerator) || !finite(tangent_denominator) ||
    tangent_denominator <= 0.0 || !finite(tangent_low_arc) || !finite(geometric_yaw) ||
    !finite(geometric_pitch) || !finite(ballistic_pitch) || !finite(ballistic_yaw) ||
    !finite(gravity_correction))
  {
    fail(
      result, BallisticFailureReason::NonFinitePitch,
      "drag-free geometric or low-arc angle is not finite");
    return result;
  }

  const double horizontal_speed = speed * std::cos(ballistic_pitch);
  const double flight_time_s = horizontal_distance / horizontal_speed;
  if (!finite(horizontal_speed) || horizontal_speed <= 0.0 || !finite(flight_time_s) ||
    flight_time_s <= 0.0)
  {
    fail(
      result, BallisticFailureReason::NonFiniteFlightTime,
      "drag-free flight time is not finite and positive");
    return result;
  }

  result.geometric_yaw_rad = geometric_yaw;
  result.geometric_pitch_rad = geometric_pitch;
  result.ballistic_yaw_rad = ballistic_yaw;
  result.ballistic_pitch_rad = ballistic_pitch;
  result.gravity_pitch_correction_rad = gravity_correction;
  result.flight_time_s = flight_time_s;

  constexpr long double kNanosecondsPerSecond = 1'000'000'000.0L;
  const long double flight_time_nanoseconds =
    static_cast<long double>(flight_time_s) * kNanosecondsPerSecond;
  const long double rounded_flight_time_nanoseconds = std::round(flight_time_nanoseconds);
  // Converting INT64_MAX to long double may round it to 2^63 on platforms
  // whose long double has no spare precision.  Use the exclusive upper bound
  // instead, so a value that would become 2^63 can never be cast to int64.
  constexpr long double kInt64ExclusiveUpperBound = 9223372036854775808.0L;
  if (!std::isfinite(flight_time_nanoseconds) || !std::isfinite(rounded_flight_time_nanoseconds) ||
    rounded_flight_time_nanoseconds < 0.0L ||
    rounded_flight_time_nanoseconds >= kInt64ExclusiveUpperBound)
  {
    fail(
      result, BallisticFailureReason::FlightTimeOverflow,
      "flight time cannot be represented as int64 nanoseconds");
    return result;
  }
  if (rounded_flight_time_nanoseconds <= 0.0L) {
    fail(
      result, BallisticFailureReason::FlightTimeUnrepresentable,
      "flight time is too small to represent as positive nanoseconds");
    return result;
  }
  const auto flight_time_ns = static_cast<std::int64_t>(rounded_flight_time_nanoseconds);
  result.flight_time_ns = flight_time_ns;
  if (flight_time_ns > config_.max_flight_time_ns) {
    fail(
      result, BallisticFailureReason::FlightTimeExceedsMaximum,
      "flight time exceeds the explicit offline maximum");
    return result;
  }

  const auto latency_ns = *config_.system_latency_ns;
  if (latency_ns > std::numeric_limits<std::int64_t>::max() - flight_time_ns) {
    fail(
      result, BallisticFailureReason::HorizonOverflow,
      "system latency plus flight time overflows int64 nanoseconds");
    return result;
  }
  const auto recommended_horizon_ns = latency_ns + flight_time_ns;
  result.recommended_prediction_horizon_ns = recommended_horizon_ns;
  if (recommended_horizon_ns > config_.max_prediction_horizon_ns) {
    fail(
      result, BallisticFailureReason::HorizonExceedsPredictionMaximum,
      "recommended prediction horizon exceeds the configured predictor maximum");
    return result;
  }

  result.valid = true;
  result.failure_reason = BallisticFailureReason::None;
  result.reason = "analytic_drag_free_low_arc";
  return result;
}

const BallisticConfig & OfflineBallisticSolver::config() const noexcept
{
  return config_;
}

OfflineBallisticDiagnostic::OfflineBallisticDiagnostic(BallisticConfig config)
: config_(std::move(config)) {}

BallisticResult OfflineBallisticDiagnostic::diagnose(
  const std::optional<TrackedTarget> & selected,
  std::int64_t frame_stamp_ns,
  const std::optional<cv::Vec3d> & target_muzzle_m) const noexcept
{
  BallisticResult result = make_result(config_);
  if (!config_.enabled) {
    fail(result, BallisticFailureReason::Disabled, "ballistic diagnostic is disabled");
    return result;
  }
  if (frame_stamp_ns < 0) {
    fail(
      result, BallisticFailureReason::NegativeTimestamp,
      "frame timestamp must not be negative");
    return result;
  }
  if (!selected.has_value()) {
    result.source_stamp_ns = frame_stamp_ns;
    fail(result, BallisticFailureReason::NoTarget, "no selected target");
    return result;
  }

  result.track_id = selected->track_id;
  result.source_stamp_ns = frame_stamp_ns;
  if (selected->track_id == 0) {
    fail(result, BallisticFailureReason::InvalidTrack, "track id must be non-zero");
    return result;
  }
  if (selected->state != TrackingState::Tracking) {
    fail(
      result, BallisticFailureReason::NotTracking,
      "ballistic diagnostic requires a Tracking target");
    return result;
  }
  const auto & observation = selected->observation;
  if (observation.stamp_ns < 0) {
    fail(
      result, BallisticFailureReason::NegativeTimestamp,
      "selected observation timestamp must not be negative");
    return result;
  }
  if (observation.stamp_ns != frame_stamp_ns ||
    (selected->last_valid_timestamp_ns >= 0 &&
    selected->last_valid_timestamp_ns != observation.stamp_ns))
  {
    fail(
      result, BallisticFailureReason::TimestampMismatch,
      "selected observation timestamp does not match the current frame");
    return result;
  }
  if (!selected->valid()) {
    fail(
      result, BallisticFailureReason::InvalidObservation,
      "selected target observation failed fail-closed validation");
    return result;
  }

  if (target_muzzle_m.has_value()) {
    OfflineBallisticSolver solver(config_);
    return solver.solve(
      BallisticMuzzleInput{
        selected->track_id,
        frame_stamp_ns,
        *target_muzzle_m,
        BallisticOriginAssumption::MuzzleFrame});
  }
  if (!config_.allow_test_gimbal_origin_as_muzzle) {
    fail(
      result, BallisticFailureReason::MissingMuzzleTransform,
      "no verified gimbal-to-muzzle transform; test gimbal origin was not explicitly allowed");
    return result;
  }
  if (!observation.gimbal_xyz_m.has_value()) {
    fail(
      result, BallisticFailureReason::MissingGimbalPose,
      "test gimbal-origin mode requires a gimbal-frame target position");
    return result;
  }

  OfflineBallisticSolver solver(config_);
  auto test_origin_result = solver.solve(
    BallisticMuzzleInput{
      selected->track_id,
      frame_stamp_ns,
      *observation.gimbal_xyz_m,
      BallisticOriginAssumption::MuzzleFrame});
  // This tag is applied only after the explicit adapter boundary above.  It
  // records the unsafe-as-production origin assumption without broadening the
  // core solver's muzzle-frame input contract.
  test_origin_result.origin_assumption = BallisticOriginAssumption::TestOnlyGimbalOrigin;
  return test_origin_result;
}

const BallisticConfig & OfflineBallisticDiagnostic::config() const noexcept
{
  return config_;
}

}  // namespace rm_auto_aim::offline
