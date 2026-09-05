#include "auto_aim_ros2/offline_ballistic.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::offline::BallisticConfig;
using rm_auto_aim::offline::BallisticFailureReason;
using rm_auto_aim::offline::BallisticMuzzleInput;
using rm_auto_aim::offline::BallisticOriginAssumption;
using rm_auto_aim::offline::OfflineBallisticDiagnostic;
using rm_auto_aim::offline::OfflineBallisticSolver;
using rm_auto_aim::offline::TargetObservation;
using rm_auto_aim::offline::TrackedTarget;
using rm_auto_aim::offline::TrackingState;

BallisticConfig valid_config()
{
  BallisticConfig config{};
  config.enabled = true;
  config.bullet_speed_mps = 20.0;
  config.gravity_mps2 = 9.81;
  config.system_latency_ns = 10'000'000;
  config.max_flight_time_ns = 2'000'000'000;
  config.max_prediction_horizon_ns = 2'000'000'000;
  return config;
}

BallisticMuzzleInput muzzle_input(
  const cv::Vec3d & position = {3.0, 0.0, 0.0},
  std::int64_t stamp_ns = 20'000'000)
{
  BallisticMuzzleInput input{};
  input.track_id = 7;
  input.source_stamp_ns = stamp_ns;
  input.target_muzzle_m = position;
  input.origin_assumption = BallisticOriginAssumption::MuzzleFrame;
  return input;
}

TargetObservation valid_observation(
  std::int64_t stamp_ns,
  std::optional<cv::Vec3d> gimbal_position = cv::Vec3d{3.0, 0.0, 0.0})
{
  TargetObservation result{};
  result.detection_index = 0;
  result.class_id = 1;
  result.armor_size = rm_auto_aim::pnp::ArmorSize::Small;
  result.confidence = 0.9F;
  result.camera_xyz_m = cv::Vec3d{0.1, 0.0, 3.0};
  result.gimbal_xyz_m = gimbal_position;
  result.relative_yaw_rad = 0.1;
  result.relative_pitch_rad = 0.02;
  result.reprojection_error_px = 0.2;
  result.stamp_ns = stamp_ns;
  result.valid = true;
  result.geometry_known = true;
  result.raw_detection.class_id = 1;
  result.raw_detection.color_id = 0;
  result.raw_detection.armor_type =
    rm_auto_aim::detector::RawArmorDetection::ArmorTypeHint::Small;
  result.raw_detection.confidence = 0.9F;
  result.raw_detection.bbox = cv::Rect2f(620.0F, 360.0F, 40.0F, 40.0F);
  result.raw_detection.keypoints = {
    result.raw_detection.bbox.tl(),
    cv::Point2f(
      result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y),
    cv::Point2f(
      result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height),
    cv::Point2f(
      result.raw_detection.bbox.x,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height)};
  return result;
}

TrackedTarget tracking_target(
  std::int64_t stamp_ns,
  std::optional<cv::Vec3d> gimbal_position = cv::Vec3d{3.0, 0.0, 0.0})
{
  TrackedTarget target{};
  target.track_id = 42;
  target.state = TrackingState::Tracking;
  target.observation = valid_observation(stamp_ns, gimbal_position);
  target.first_valid_timestamp_ns = stamp_ns;
  target.last_valid_timestamp_ns = stamp_ns;
  target.consecutive_valid = 2;
  target.yaw_vel_rad_s = 0.0;
  target.pitch_vel_rad_s = 0.0;
  return target;
}

void expect_safe_command(const rm_auto_aim::pipeline::AimCommand & command)
{
  EXPECT_FLOAT_EQ(command.yaw_rad, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_rad, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw_acc_rad_s2, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_acc_rad_s2, 0.0F);
  EXPECT_EQ(command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(command.fire_command, rm_auto_aim::pipeline::kFireNone);
}

}  // namespace

TEST(OfflineBallisticSolver, HorizontalThreeMetreGoldenIsDragFreeLowArc)
{
  const OfflineBallisticSolver solver(valid_config());
  const auto result = solver.solve(muzzle_input());

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.failure_reason, BallisticFailureReason::None);
  ASSERT_TRUE(result.geometric_yaw_rad.has_value());
  ASSERT_TRUE(result.geometric_pitch_rad.has_value());
  ASSERT_TRUE(result.ballistic_yaw_rad.has_value());
  ASSERT_TRUE(result.ballistic_pitch_rad.has_value());
  ASSERT_TRUE(result.gravity_pitch_correction_rad.has_value());
  ASSERT_TRUE(result.flight_time_s.has_value());
  ASSERT_TRUE(result.flight_time_ns.has_value());
  ASSERT_TRUE(result.recommended_prediction_horizon_ns.has_value());
  // Independent hard-coded software golden; it is not a hardware frame or a
  // real-world hit-rate claim.  Values use this test's explicit v=20 m/s and
  // g=9.81 m/s^2 inputs.
  EXPECT_NEAR(*result.geometric_yaw_rad, 0.0, 1e-15);
  EXPECT_NEAR(*result.geometric_pitch_rad, 0.0, 1e-15);
  EXPECT_NEAR(*result.ballistic_pitch_rad, 0.03682077128879508, 1e-14);
  EXPECT_NEAR(*result.flight_time_s, 0.1501017401625011, 1e-14);
  EXPECT_EQ(*result.flight_time_ns, 150'101'740);
  EXPECT_EQ(*result.recommended_prediction_horizon_ns, 160'101'740);
  EXPECT_TRUE(result.test_only);
  EXPECT_FALSE(result.production_ready);
  EXPECT_FALSE(result.ballistic_control_applied);
  EXPECT_EQ(result.origin_assumption, BallisticOriginAssumption::MuzzleFrame);
}

TEST(OfflineBallisticSolver, OriginMustBeDeclaredByCaller)
{
  auto input = muzzle_input();
  input.origin_assumption = BallisticOriginAssumption::NotEvaluated;
  const auto result = OfflineBallisticSolver(valid_config()).solve(input);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, BallisticFailureReason::MissingMuzzleTransform);
  EXPECT_FALSE(result.target_muzzle_m.has_value());
}

TEST(OfflineBallisticSolver, ForwardProjectileEquationReturnsToThreeDimensionalTarget)
{
  const auto config = valid_config();
  const auto result = OfflineBallisticSolver(config).solve(muzzle_input({3.0, 1.0, 0.5}));
  ASSERT_TRUE(result.valid);
  ASSERT_TRUE(result.ballistic_yaw_rad.has_value());
  ASSERT_TRUE(result.ballistic_pitch_rad.has_value());
  ASSERT_TRUE(result.flight_time_s.has_value());

  // This forward equation is deliberately written independently of the
  // inverse solver.  It verifies x-forward/y-left/z-up with gravity down.
  const double speed = *config.bullet_speed_mps;
  const double gravity = *config.gravity_mps2;
  const double yaw = *result.ballistic_yaw_rad;
  const double pitch = *result.ballistic_pitch_rad;
  const double time = *result.flight_time_s;
  const double x = speed * std::cos(pitch) * std::cos(yaw) * time;
  const double y = speed * std::cos(pitch) * std::sin(yaw) * time;
  const double z = speed * std::sin(pitch) * time - 0.5 * gravity * time * time;
  EXPECT_NEAR(x, 3.0, 1e-10);
  EXPECT_NEAR(y, 1.0, 1e-10);
  EXPECT_NEAR(z, 0.5, 1e-10);
}

TEST(OfflineBallisticSolver, SpeedHeightAndYawAreExplicitGeometryDiagnostics)
{
  auto slow_config = valid_config();
  slow_config.bullet_speed_mps = 15.0;
  auto fast_config = valid_config();
  fast_config.bullet_speed_mps = 30.0;
  const auto slow = OfflineBallisticSolver(slow_config).solve(muzzle_input());
  const auto fast = OfflineBallisticSolver(fast_config).solve(muzzle_input());
  ASSERT_TRUE(slow.valid);
  ASSERT_TRUE(fast.valid);
  EXPECT_GT(*slow.flight_time_s, *fast.flight_time_s);
  EXPECT_GT(*slow.gravity_pitch_correction_rad, *fast.gravity_pitch_correction_rad);

  const OfflineBallisticSolver solver(valid_config());
  const auto above = solver.solve(muzzle_input({3.0, 0.0, 0.5}));
  const auto below = solver.solve(muzzle_input({3.0, 0.0, -0.5}, 20'000'001));
  const auto left = solver.solve(muzzle_input({3.0, 1.0, 0.0}, 20'000'002));
  const auto right = solver.solve(muzzle_input({3.0, -1.0, 0.0}, 20'000'003));
  ASSERT_TRUE(above.valid);
  ASSERT_TRUE(below.valid);
  ASSERT_TRUE(left.valid);
  ASSERT_TRUE(right.valid);
  EXPECT_GT(*above.geometric_pitch_rad, 0.0);
  EXPECT_LT(*below.geometric_pitch_rad, 0.0);
  EXPECT_GT(*above.gravity_pitch_correction_rad, 0.0);
  EXPECT_GT(*below.gravity_pitch_correction_rad, 0.0);
  EXPECT_GT(*left.ballistic_yaw_rad, 0.0);
  EXPECT_LT(*right.ballistic_yaw_rad, 0.0);
}

TEST(OfflineBallisticSolver, ReachabilityAndMuzzleGeometryFailClosed)
{
  auto boundary_config = valid_config();
  boundary_config.bullet_speed_mps = 10.0;
  boundary_config.gravity_mps2 = 10.0;
  boundary_config.system_latency_ns = 0;
  boundary_config.max_flight_time_ns = std::numeric_limits<std::int64_t>::max();
  boundary_config.max_prediction_horizon_ns = std::numeric_limits<std::int64_t>::max();
  const OfflineBallisticSolver boundary_solver(boundary_config);
  const auto edge = boundary_solver.solve(muzzle_input({1.0, 0.0, 4.95}));
  ASSERT_TRUE(edge.valid);
  const auto unreachable = boundary_solver.solve(muzzle_input({1.0, 0.0, 4.951}, 21));
  EXPECT_FALSE(unreachable.valid);
  EXPECT_EQ(unreachable.failure_reason, BallisticFailureReason::DiscriminantNegative);

  const OfflineBallisticSolver solver(valid_config());
  const auto behind = solver.solve(muzzle_input({-3.0, 0.0, 0.0}));
  EXPECT_FALSE(behind.valid);
  EXPECT_EQ(behind.failure_reason, BallisticFailureReason::TargetBehindMuzzle);
  const auto lateral = solver.solve(muzzle_input({0.0, 3.0, 0.0}, 22));
  EXPECT_FALSE(lateral.valid);
  EXPECT_EQ(lateral.failure_reason, BallisticFailureReason::TargetBehindMuzzle);
  const auto too_close = solver.solve(muzzle_input({0.0, 0.0, 0.0}, 23));
  EXPECT_FALSE(too_close.valid);
  EXPECT_EQ(too_close.failure_reason, BallisticFailureReason::HorizontalDistanceTooSmall);
  auto non_muzzle = muzzle_input({3.0, 0.0, 0.0}, 24);
  non_muzzle.origin_assumption = BallisticOriginAssumption::TestOnlyGimbalOrigin;
  const auto rejected_non_muzzle = solver.solve(non_muzzle);
  EXPECT_FALSE(rejected_non_muzzle.valid);
  EXPECT_EQ(rejected_non_muzzle.failure_reason, BallisticFailureReason::MissingMuzzleTransform);
}

TEST(OfflineBallisticSolver, InvalidPhysicalInputsAndNonFiniteValuesFailClosed)
{
  const auto input = muzzle_input();
  {
    auto config = valid_config();
    config.bullet_speed_mps.reset();
    EXPECT_EQ(
      OfflineBallisticSolver(config).solve(input).failure_reason,
      BallisticFailureReason::MissingBulletSpeed);
  }
  for (const double speed : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto config = valid_config();
    config.bullet_speed_mps = speed;
    const auto result = OfflineBallisticSolver(config).solve(input);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::InvalidBulletSpeed);
    if (!std::isfinite(speed)) {
      EXPECT_FALSE(result.bullet_speed_mps.has_value());
    }
  }
  {
    auto config = valid_config();
    config.gravity_mps2.reset();
    EXPECT_EQ(
      OfflineBallisticSolver(config).solve(input).failure_reason,
      BallisticFailureReason::MissingGravity);
  }
  for (const double gravity : {0.0, -9.81, std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    auto config = valid_config();
    config.gravity_mps2 = gravity;
    const auto result = OfflineBallisticSolver(config).solve(input);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::InvalidGravity);
    if (!std::isfinite(gravity)) {
      EXPECT_FALSE(result.gravity_mps2.has_value());
    }
  }
  {
    auto config = valid_config();
    config.system_latency_ns.reset();
    EXPECT_EQ(
      OfflineBallisticSolver(config).solve(input).failure_reason,
      BallisticFailureReason::MissingSystemLatency);
    config.system_latency_ns = -1;
    EXPECT_EQ(
      OfflineBallisticSolver(config).solve(input).failure_reason,
      BallisticFailureReason::NegativeSystemLatency);
  }
  for (const cv::Vec3d position : {
    cv::Vec3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    cv::Vec3d{3.0, std::numeric_limits<double>::infinity(), 0.0},
    cv::Vec3d{3.0, 0.0, -std::numeric_limits<double>::infinity()}})
  {
    const auto result = OfflineBallisticSolver(valid_config()).solve(muzzle_input(position));
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::NonFiniteTargetPosition);
    EXPECT_FALSE(result.target_muzzle_m.has_value());
  }
}

TEST(OfflineBallisticSolver, TimeAndHorizonBoundsNeverTruncate)
{
  {
    auto config = valid_config();
    config.max_flight_time_ns = 1;
    const auto result = OfflineBallisticSolver(config).solve(muzzle_input());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::FlightTimeExceedsMaximum);
    EXPECT_TRUE(result.flight_time_ns.has_value());
  }
  {
    auto config = valid_config();
    config.max_flight_time_ns = std::numeric_limits<std::int64_t>::max();
    config.max_prediction_horizon_ns = std::numeric_limits<std::int64_t>::max();
    config.bullet_speed_mps = 1.0e12;
    config.system_latency_ns = 0;
    const auto result = OfflineBallisticSolver(config).solve(muzzle_input({1.0e22, 0.0, 0.0}));
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::FlightTimeOverflow);
  }
  {
    auto config = valid_config();
    config.max_flight_time_ns = std::numeric_limits<std::int64_t>::max();
    config.max_prediction_horizon_ns = std::numeric_limits<std::int64_t>::max();
    config.system_latency_ns = std::numeric_limits<std::int64_t>::max() - 1;
    const auto result = OfflineBallisticSolver(config).solve(muzzle_input());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::HorizonOverflow);
  }
  {
    auto config = valid_config();
    config.system_latency_ns = 0;
    config.max_prediction_horizon_ns = 100'000'000;
    const auto result = OfflineBallisticSolver(config).solve(muzzle_input());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::HorizonExceedsPredictionMaximum);
    ASSERT_TRUE(result.recommended_prediction_horizon_ns.has_value());
    EXPECT_GT(*result.recommended_prediction_horizon_ns, config.max_prediction_horizon_ns);
  }
}

TEST(OfflineBallisticDiagnostic, DisabledAndSelectedTargetBoundariesAreExplicit)
{
  const auto selected = tracking_target(100);
  const auto disabled = OfflineBallisticDiagnostic{}.diagnose(selected, 100);
  EXPECT_FALSE(disabled.valid);
  EXPECT_EQ(disabled.failure_reason, BallisticFailureReason::Disabled);

  const auto no_target = OfflineBallisticDiagnostic(valid_config()).diagnose(std::nullopt, 100);
  EXPECT_FALSE(no_target.valid);
  EXPECT_EQ(no_target.failure_reason, BallisticFailureReason::NoTarget);

  for (const auto state :
    {TrackingState::Detecting, TrackingState::TempLost, TrackingState::Lost})
  {
    auto target = selected;
    target.state = state;
    const auto result = OfflineBallisticDiagnostic(valid_config()).diagnose(target, 100);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, BallisticFailureReason::NotTracking);
  }

  auto invalid = selected;
  invalid.observation.valid = false;
  const auto invalid_result = OfflineBallisticDiagnostic(valid_config()).diagnose(invalid, 100);
  EXPECT_FALSE(invalid_result.valid);
  EXPECT_EQ(invalid_result.failure_reason, BallisticFailureReason::InvalidObservation);

  auto mismatch = selected;
  mismatch.observation.stamp_ns = 99;
  const auto mismatch_result = OfflineBallisticDiagnostic(valid_config()).diagnose(mismatch, 100);
  EXPECT_FALSE(mismatch_result.valid);
  EXPECT_EQ(mismatch_result.failure_reason, BallisticFailureReason::TimestampMismatch);
}

TEST(OfflineBallisticDiagnostic, MuzzleBoundaryRequiresExplicitTestOriginAuthorization)
{
  const auto selected = tracking_target(100, cv::Vec3d{3.0, 0.0, 0.0});
  const auto missing_transform = OfflineBallisticDiagnostic(valid_config()).diagnose(selected, 100);
  EXPECT_FALSE(missing_transform.valid);
  EXPECT_EQ(missing_transform.failure_reason, BallisticFailureReason::MissingMuzzleTransform);

  auto test_origin_config = valid_config();
  test_origin_config.allow_test_gimbal_origin_as_muzzle = true;
  const auto test_origin = OfflineBallisticDiagnostic(test_origin_config).diagnose(selected, 100);
  ASSERT_TRUE(test_origin.valid);
  EXPECT_EQ(test_origin.origin_assumption, BallisticOriginAssumption::TestOnlyGimbalOrigin);
  EXPECT_TRUE(test_origin.test_only);
  EXPECT_FALSE(test_origin.production_ready);

  const auto no_gimbal = tracking_target(100, std::nullopt);
  const auto missing_gimbal =
    OfflineBallisticDiagnostic(test_origin_config).diagnose(no_gimbal, 100);
  EXPECT_FALSE(missing_gimbal.valid);
  EXPECT_EQ(missing_gimbal.failure_reason, BallisticFailureReason::MissingGimbalPose);

  const auto supplied_muzzle = OfflineBallisticDiagnostic(valid_config()).diagnose(
    selected, 100, cv::Vec3d{3.0, 0.0, 0.0});
  ASSERT_TRUE(supplied_muzzle.valid);
  EXPECT_EQ(supplied_muzzle.origin_assumption, BallisticOriginAssumption::MuzzleFrame);
}

TEST(OfflineBallisticDiagnostic, ItIsDeterministicAndCannotAlterSelectionOrSafeCommand)
{
  const auto selected = tracking_target(100, cv::Vec3d{3.0, 0.5, 0.0});
  auto config = valid_config();
  config.allow_test_gimbal_origin_as_muzzle = true;
  const OfflineBallisticDiagnostic diagnostic(config);
  const auto aimed_before = rm_auto_aim::offline::SafeOfflineAimer{}.aim(selected);
  const auto selected_id_before = selected.track_id;
  const auto state_before = selected.state;
  const auto gimbal_before = *selected.observation.gimbal_xyz_m;

  const auto first = diagnostic.diagnose(selected, 100);
  const auto second = diagnostic.diagnose(selected, 100);
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_EQ(first.track_id, second.track_id);
  EXPECT_EQ(first.flight_time_ns, second.flight_time_ns);
  EXPECT_EQ(first.recommended_prediction_horizon_ns, second.recommended_prediction_horizon_ns);
  EXPECT_EQ(selected.track_id, selected_id_before);
  EXPECT_EQ(selected.state, state_before);
  EXPECT_DOUBLE_EQ((*selected.observation.gimbal_xyz_m)[0], gimbal_before[0]);
  EXPECT_DOUBLE_EQ((*selected.observation.gimbal_xyz_m)[1], gimbal_before[1]);
  EXPECT_DOUBLE_EQ((*selected.observation.gimbal_xyz_m)[2], gimbal_before[2]);

  const auto aimed_after = rm_auto_aim::offline::SafeOfflineAimer{}.aim(selected);
  EXPECT_EQ(aimed_before.target_lock, aimed_after.target_lock);
  EXPECT_EQ(aimed_before.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_EQ(aimed_after.fire_command, rm_auto_aim::pipeline::kFireNone);
  expect_safe_command(aimed_before.safe_command());
  expect_safe_command(aimed_after.safe_command());
}
