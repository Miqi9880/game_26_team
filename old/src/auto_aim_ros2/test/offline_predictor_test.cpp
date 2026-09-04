#include "auto_aim_ros2/offline_pipeline.hpp"

#include <cmath>
#include <limits>
#include <optional>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::offline::AimerConfig;
using rm_auto_aim::offline::OfflinePredictor;
using rm_auto_aim::offline::PredictionConfig;
using rm_auto_aim::offline::PredictionFailureReason;
using rm_auto_aim::offline::SyntheticPredictionError;
using rm_auto_aim::offline::TargetObservation;
using rm_auto_aim::offline::TargetSelector;
using rm_auto_aim::offline::TrackedTarget;
using rm_auto_aim::offline::TrackingState;

TargetObservation make_observation(
  std::int64_t stamp_ns, double yaw, double pitch, std::uint64_t detection_index = 0)
{
  TargetObservation observation{};
  observation.detection_index = static_cast<std::size_t>(detection_index);
  observation.class_id = 1;
  observation.armor_size = rm_auto_aim::pnp::ArmorSize::Small;
  observation.confidence = 0.9F;
  observation.camera_xyz_m = cv::Vec3d{0.1, 0.0, 5.0};
  observation.relative_yaw_rad = yaw;
  observation.relative_pitch_rad = pitch;
  observation.reprojection_error_px = 0.25;
  observation.stamp_ns = stamp_ns;
  observation.valid = true;
  observation.geometry_known = true;
  observation.raw_detection.class_id = 1;
  observation.raw_detection.confidence = 0.9F;
  observation.raw_detection.bbox = cv::Rect2f(620.0F, 360.0F, 40.0F, 40.0F);
  observation.raw_detection.keypoints = {
    observation.raw_detection.bbox.tl(),
    cv::Point2f(observation.raw_detection.bbox.x + observation.raw_detection.bbox.width,
      observation.raw_detection.bbox.y),
    cv::Point2f(observation.raw_detection.bbox.x + observation.raw_detection.bbox.width,
      observation.raw_detection.bbox.y + observation.raw_detection.bbox.height),
    cv::Point2f(observation.raw_detection.bbox.x,
      observation.raw_detection.bbox.y + observation.raw_detection.bbox.height)};
  return observation;
}

TrackedTarget make_tracking(
  std::uint64_t track_id, std::int64_t stamp_ns, double yaw, double pitch,
  double yaw_velocity = 1.0, double pitch_velocity = 0.2)
{
  TrackedTarget target{};
  target.track_id = track_id;
  target.state = TrackingState::Tracking;
  target.observation = make_observation(stamp_ns, yaw, pitch, track_id);
  target.last_valid_timestamp_ns = stamp_ns;
  target.first_valid_timestamp_ns = stamp_ns;
  target.consecutive_valid = 2;
  target.yaw_vel_rad_s = yaw_velocity;
  target.pitch_vel_rad_s = pitch_velocity;
  return target;
}

PredictionConfig config(std::int64_t horizon_ns = 50'000'000)
{
  PredictionConfig result{};
  result.enabled = true;
  result.horizon_ns = horizon_ns;
  result.max_horizon_ns = 100'000'000;
  return result;
}

TEST(OfflinePredictor, ConstantVelocityUsesRelativeRadiansAndExactTimestamps)
{
  OfflinePredictor predictor(config());
  const auto target = make_tracking(7, 10'000'000, 0.10, 0.02, 1.0, 0.2);
  const auto result = predictor.predict(target, 10'000'000);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::None);
  EXPECT_EQ(result.track_id, 7U);
  EXPECT_EQ(result.source_stamp_ns, 10'000'000);
  EXPECT_EQ(result.horizon_ns, 50'000'000);
  EXPECT_EQ(result.predicted_stamp_ns, 60'000'000);
  EXPECT_NEAR(result.horizon_s, 0.05, 1e-15);
  ASSERT_TRUE(result.predicted_relative_yaw_rad.has_value());
  ASSERT_TRUE(result.predicted_relative_pitch_rad.has_value());
  EXPECT_NEAR(*result.predicted_relative_yaw_rad, 0.15, 1e-12);
  EXPECT_NEAR(*result.predicted_relative_pitch_rad, 0.03, 1e-12);
  EXPECT_TRUE(result.test_only);
  EXPECT_FALSE(result.production_ready);
}

TEST(OfflinePredictor, ExplicitZeroHorizonReturnsCurrentAngles)
{
  OfflinePredictor predictor(config(0));
  const auto target = make_tracking(1, 0, -0.4, 0.3, 12.0, -8.0);
  const auto result = predictor.predict(target, 0);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.predicted_stamp_ns, 0);
  EXPECT_DOUBLE_EQ(*result.predicted_relative_yaw_rad, -0.4);
  EXPECT_DOUBLE_EQ(*result.predicted_relative_pitch_rad, 0.3);
}

TEST(OfflinePredictor, DisabledByDefaultAndNegativeOrExcessHorizonFailClosed)
{
  OfflinePredictor disabled;
  const auto target = make_tracking(1, 0, 0.0, 0.0);
  auto result = disabled.predict(target, 0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::Disabled);
  EXPECT_TRUE(result.test_only);
  EXPECT_FALSE(result.production_ready);

  OfflinePredictor negative( config(-1) );
  result = negative.predict(target, 0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::NegativeHorizon);

  auto over_limit = config(100'000'001);
  over_limit.max_horizon_ns = 100'000'000;
  OfflinePredictor excessive(over_limit);
  result = excessive.predict(target, 0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::HorizonExceedsMaximum);
}

TEST(OfflinePredictor, MissingAndNonFiniteAnglesOrVelocityNeverProduceOutput)
{
  const auto base = make_tracking(1, 0, 0.1, 0.2);
  {
    OfflinePredictor predictor(config());
    auto target = base;
    target.observation.relative_yaw_rad.reset();
    const auto result = predictor.predict(target, 0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::MissingRelativeAngle);
    EXPECT_FALSE(result.predicted_relative_yaw_rad.has_value());
  }
  {
    OfflinePredictor predictor(config());
    auto target = base;
    target.observation.relative_pitch_rad = std::numeric_limits<double>::quiet_NaN();
    const auto result = predictor.predict(target, 0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NonFiniteAngle);
  }
  {
    OfflinePredictor predictor(config());
    auto target = base;
    target.yaw_vel_rad_s = std::numeric_limits<double>::infinity();
    const auto result = predictor.predict(target, 0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NonFiniteVelocity);
  }
  {
    OfflinePredictor predictor(config());
    auto target = base;
    target.observation.relative_yaw_rad = std::numeric_limits<double>::max();
    target.yaw_vel_rad_s = std::numeric_limits<double>::max();
    const auto result = predictor.predict(target, 0);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NonFiniteResult);
  }
}

TEST(OfflinePredictor, NonTrackingStatesAndMalformedTimestampsAreRejected)
{
  const auto base = make_tracking(1, 10, 0.1, 0.2);
  for (const auto state : {TrackingState::Detecting, TrackingState::TempLost, TrackingState::Lost}) {
    OfflinePredictor predictor(config());
    auto target = base;
    target.state = state;
    const auto result = predictor.predict(target, 10);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NotTracking);
  }

  {
    OfflinePredictor predictor(config());
    const auto result = predictor.predict(base, -1);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NegativeTimestamp);
  }
  {
    OfflinePredictor predictor(config());
    auto mismatch = base;
    mismatch.observation.stamp_ns = 9;
    const auto result = predictor.predict(mismatch, 10);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::TimestampMismatch);
  }
  {
    OfflinePredictor predictor(config());
    ASSERT_TRUE(predictor.predict(base, 10).valid);
    auto rollback = make_tracking(1, 9, 0.2, 0.22);
    auto result = predictor.predict(rollback, 9);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NonMonotonicTimestamp);
    auto repeat = make_tracking(1, 10, 0.2, 0.22);
    result = predictor.predict(repeat, 10);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.failure_reason, PredictionFailureReason::NonMonotonicTimestamp);
    predictor.reset();
    EXPECT_TRUE(predictor.predict(repeat, 10).valid);
  }
}

TEST(OfflinePredictor, TimestampOverflowAndNoTargetAreExplicitFailures)
{
  OfflinePredictor predictor(config());
  const auto target = make_tracking(1, std::numeric_limits<std::int64_t>::max() - 1, 0.0, 0.0);
  auto result = predictor.predict(target, std::numeric_limits<std::int64_t>::max() - 1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::TimestampOverflow);

  predictor.reset();
  result = predictor.predict(std::nullopt, 0);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure_reason, PredictionFailureReason::NoTarget);
}

TEST(OfflinePredictor, TrackIdsAndSelectorAimerSafetyRemainIndependent)
{
  const auto a = make_tracking(11, 0, 0.1, 0.01, 1.0, 0.1);
  const auto b = make_tracking(22, 10, -0.4, -0.02, -2.0, 0.3);
  OfflinePredictor predictor(config(10'000'000));
  const auto predicted_a = predictor.predict(a, 0);
  const auto predicted_b = predictor.predict(b, 10);
  ASSERT_TRUE(predicted_a.valid);
  ASSERT_TRUE(predicted_b.valid);
  EXPECT_EQ(predicted_a.track_id, a.track_id);
  EXPECT_EQ(predicted_b.track_id, b.track_id);
  EXPECT_DOUBLE_EQ(*predicted_a.predicted_relative_yaw_rad, 0.11);
  EXPECT_DOUBLE_EQ(*predicted_b.predicted_relative_yaw_rad, -0.42);

  TargetSelector selector;
  const auto selected_before = selector.select({a, b}, 1280, 800);
  ASSERT_TRUE(selected_before.has_value());
  const auto selected_id = selected_before->track_id;
  const auto aimed_before = rm_auto_aim::offline::SafeOfflineAimer{}.aim(selected_before);
  (void)predicted_a;
  (void)predicted_b;
  const auto selected_after = selector.select({a, b}, 1280, 800);
  ASSERT_TRUE(selected_after.has_value());
  EXPECT_EQ(selected_after->track_id, selected_id);
  const auto aimed_after = rm_auto_aim::offline::SafeOfflineAimer{}.aim(selected_after);
  EXPECT_EQ(aimed_before.target_lock, aimed_after.target_lock);
  EXPECT_EQ(aimed_before.fire_command, 0);
  EXPECT_EQ(aimed_after.fire_command, 0);
  EXPECT_EQ(aimed_after.safe_command().target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(aimed_after.safe_command().fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflinePredictor, InputOrderAndSyntheticLatencyDiagnosticAreDeterministic)
{
  const auto first = make_tracking(1, 0, 0.0, 0.0, 0.5, -0.25);
  const auto second = make_tracking(1, 100'000'000, 0.05, -0.025, 0.5, -0.25);
  OfflinePredictor predictor_a(config(100'000'000));
  OfflinePredictor predictor_b(config(100'000'000));
  const auto a0 = predictor_a.predict(first, 0);
  const auto a1 = predictor_a.predict(second, 100'000'000);
  const auto b1 = predictor_b.predict(first, 0);
  const auto b0 = predictor_b.predict(second, 100'000'000);
  EXPECT_EQ(a0.track_id, b1.track_id);
  EXPECT_EQ(a1.track_id, b0.track_id);
  EXPECT_EQ(a0.predicted_relative_yaw_rad, b1.predicted_relative_yaw_rad);
  EXPECT_EQ(a1.predicted_relative_pitch_rad, b0.predicted_relative_pitch_rad);

  auto future = make_observation(200'000'000, 0.105, -0.0525, 1);
  const SyntheticPredictionError error =
    rm_auto_aim::offline::diagnose_synthetic_prediction_error(a1, future);
  ASSERT_TRUE(error.valid);
  EXPECT_TRUE(error.synthetic);
  EXPECT_NEAR(*error.yaw_error_rad, 0.005, 1e-12);
  EXPECT_NEAR(*error.pitch_error_rad, -0.0025, 1e-12);
  EXPECT_EQ(error.track_id, 1U);
}

}  // namespace
