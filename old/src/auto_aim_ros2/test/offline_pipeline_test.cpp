#include "auto_aim_ros2/offline_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::offline::AimerConfig;
using rm_auto_aim::offline::AimerMode;
using rm_auto_aim::offline::OfflineTracker;
using rm_auto_aim::offline::TargetObservation;
using rm_auto_aim::offline::TargetSelector;
using rm_auto_aim::offline::TargetSelectorConfig;
using rm_auto_aim::offline::TrackedTarget;
using rm_auto_aim::offline::TrackerConfig;
using rm_auto_aim::offline::TrackingState;

TargetObservation observation(
  std::int64_t stamp_ns,
  std::size_t detection_index,
  double x,
  double yaw,
  double pitch,
  float confidence = 0.8F,
  int class_id = 1)
{
  TargetObservation result{};
  result.detection_index = detection_index;
  result.class_id = class_id;
  result.armor_size = rm_auto_aim::pnp::ArmorSize::Small;
  result.confidence = confidence;
  result.camera_xyz_m = cv::Vec3d{x, 0.0, 5.0};
  result.gimbal_xyz_m = cv::Vec3d{5.0, x, 0.2};
  result.relative_yaw_rad = yaw;
  result.relative_pitch_rad = pitch;
  result.reprojection_error_px = 0.5;
  result.stamp_ns = stamp_ns;
  result.valid = true;
  result.geometry_known = true;
  result.raw_detection.class_id = class_id;
  result.raw_detection.confidence = confidence;
  result.raw_detection.bbox = cv::Rect2f(
    static_cast<float>(x * 20.0 + 600.0), 400.0F, 40.0F, 40.0F);
  result.raw_detection.keypoints = {
    cv::Point2f(result.raw_detection.bbox.x, result.raw_detection.bbox.y),
    cv::Point2f(result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y),
    cv::Point2f(result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height),
    cv::Point2f(result.raw_detection.bbox.x,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height)};
  return result;
}

TrackerConfig tracker_config()
{
  TrackerConfig config{};
  config.min_detect_count = 2;
  config.max_temp_lost_ms = 100;
  config.max_position_jump_m = 0.75;
  config.max_angle_jump_rad = 0.5;
  return config;
}

TrackedTarget tracked(
  std::uint64_t track_id,
  float confidence,
  float center_x,
  TrackingState state = TrackingState::Tracking)
{
  TrackedTarget result{};
  result.track_id = track_id;
  result.state = state;
  result.observation = observation(0, static_cast<std::size_t>(track_id),
    static_cast<double>(center_x) / 20.0, 0.1, 0.05, confidence);
  result.observation.raw_detection.bbox.x = center_x;
  result.consecutive_valid = 2;
  return result;
}
}  // namespace

TEST(OfflineTargetObservation, InvalidPoseDoesNotBecomeGeometryKnown)
{
  rm_auto_aim::pnp::PoseObservation pose{};
  pose.valid = false;
  pose.raw_detection.class_id = 2;
  const auto result = rm_auto_aim::offline::make_target_observation(pose, 10, 3);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.geometry_known);
  EXPECT_EQ(result.detection_index, 3U);
  EXPECT_FALSE(result.camera_xyz_m.has_value());
}

TEST(OfflineTracker, FirstAndSecondValidObservationsLock)
{
  OfflineTracker tracker(tracker_config());
  const auto first = tracker.update(std::vector<TargetObservation>{observation(0, 0, 0.0, 0.1, 0.2)}, 0);
  ASSERT_TRUE(first.accepted);
  ASSERT_EQ(first.tracks.size(), 1U);
  EXPECT_EQ(first.tracks[0].state, TrackingState::Detecting);
  EXPECT_FALSE(first.target_lock());

  const auto second = tracker.update(
    std::vector<TargetObservation>{observation(10'000'000, 0, 0.01, 0.11, 0.21)}, 10'000'000);
  ASSERT_TRUE(second.accepted);
  ASSERT_EQ(second.tracks.size(), 1U);
  EXPECT_EQ(second.tracks[0].state, TrackingState::Tracking);
  EXPECT_TRUE(second.target_lock());
  EXPECT_NEAR(second.tracks[0].yaw_vel_rad_s, 1.0, 1e-9);
  EXPECT_NEAR(second.tracks[0].pitch_vel_rad_s, 1.0, 1e-9);
}

TEST(OfflineTracker, InvalidPnPAndBadEvidenceNeverLock)
{
  OfflineTracker tracker(tracker_config());
  auto invalid = observation(0, 0, 0.0, 0.0, 0.0);
  invalid.valid = false;
  const auto update = tracker.update(std::vector<TargetObservation>{invalid}, 0);
  EXPECT_FALSE(update.accepted);
  EXPECT_TRUE(update.rejected);
  EXPECT_FALSE(update.target_lock());
  EXPECT_TRUE(update.tracks.empty());

  auto nan_position = observation(10, 0, 0.0, 0.0, 0.0);
  nan_position.camera_xyz_m->val[0] = std::numeric_limits<double>::quiet_NaN();
  const auto nan_update = tracker.update(std::vector<TargetObservation>{nan_position}, 10);
  EXPECT_TRUE(nan_update.rejected);
  EXPECT_TRUE(nan_update.tracks.empty());

  auto negative_depth = observation(20, 0, 0.0, 0.0, 0.0);
  negative_depth.camera_xyz_m->val[2] = -1.0;
  const auto depth_update = tracker.update(std::vector<TargetObservation>{negative_depth}, 20);
  EXPECT_TRUE(depth_update.rejected);
  EXPECT_TRUE(depth_update.tracks.empty());

  auto unsupported_armor = observation(30, 0, 0.0, 0.0, 0.0);
  unsupported_armor.armor_size = static_cast<rm_auto_aim::pnp::ArmorSize>(99);
  const auto armor_update = tracker.update(
    std::vector<TargetObservation>{unsupported_armor}, 30);
  EXPECT_TRUE(armor_update.rejected);
  EXPECT_TRUE(armor_update.tracks.empty());

  auto conflicting_armor = observation(40, 0, 0.0, 0.0, 0.0);
  conflicting_armor.raw_detection.armor_type =
    rm_auto_aim::detector::RawArmorDetection::ArmorTypeHint::Large;
  const auto conflicting_update = tracker.update(
    std::vector<TargetObservation>{conflicting_armor}, 40);
  EXPECT_TRUE(conflicting_update.rejected);
  EXPECT_TRUE(conflicting_update.tracks.empty());
}

TEST(OfflineTracker, TemporaryAndLongLossUnlock)
{
  OfflineTracker tracker(tracker_config());
  ASSERT_TRUE(tracker.update(std::vector<TargetObservation>{observation(0, 0, 0.0, 0.0, 0.0)}, 0).accepted);
  ASSERT_TRUE(tracker.update(
    std::vector<TargetObservation>{observation(10, 0, 0.0, 0.0, 0.0)}, 10).accepted);

  const auto temporary = tracker.update(std::vector<TargetObservation>{}, 50);
  ASSERT_EQ(temporary.tracks.size(), 1U);
  EXPECT_EQ(temporary.tracks[0].state, TrackingState::TempLost);
  EXPECT_FALSE(temporary.target_lock());

  const auto lost = tracker.update(std::vector<TargetObservation>{}, 100'000'010);
  ASSERT_EQ(lost.tracks.size(), 1U);
  EXPECT_EQ(lost.tracks[0].state, TrackingState::Lost);
  EXPECT_FALSE(lost.target_lock());
}

TEST(OfflineTracker, TimestampRollbackAndDuplicateAreRejected)
{
  OfflineTracker tracker(tracker_config());
  ASSERT_TRUE(tracker.update(std::vector<TargetObservation>{observation(100, 0, 0.0, 0.0, 0.0)}, 100).accepted);
  ASSERT_TRUE(tracker.update(
    std::vector<TargetObservation>{observation(110, 0, 0.0, 0.0, 0.0)}, 110).accepted);
  const auto rollback = tracker.update(
    std::vector<TargetObservation>{observation(90, 0, 0.0, 0.0, 0.0)}, 90);
  EXPECT_TRUE(rollback.rejected);
  EXPECT_FALSE(rollback.target_lock());
  ASSERT_EQ(rollback.tracks.size(), 1U);
  EXPECT_EQ(rollback.tracks[0].state, TrackingState::TempLost);
  ASSERT_TRUE(rollback.primary_track.has_value());
  EXPECT_EQ(rollback.state, TrackingState::TempLost);
  EXPECT_EQ(rollback.primary_track->state, TrackingState::TempLost);
  const auto duplicate = tracker.update(
    std::vector<TargetObservation>{observation(110, 0, 0.0, 0.0, 0.0)}, 110);
  EXPECT_TRUE(duplicate.rejected);
  ASSERT_EQ(duplicate.tracks.size(), 1U);
  EXPECT_EQ(duplicate.tracks[0].last_valid_timestamp_ns, 110);
  ASSERT_TRUE(duplicate.primary_track.has_value());
  EXPECT_EQ(duplicate.state, TrackingState::TempLost);
  EXPECT_EQ(duplicate.primary_track->state, TrackingState::TempLost);
}

TEST(OfflineTracker, NegativeTimestampReportsSafeDiagnosticState)
{
  OfflineTracker tracker(tracker_config());
  ASSERT_TRUE(tracker.update(
    std::vector<TargetObservation>{observation(100, 0, 0.0, 0.0, 0.0)}, 100).accepted);
  ASSERT_TRUE(tracker.update(
    std::vector<TargetObservation>{observation(110, 0, 0.0, 0.0, 0.0)}, 110).accepted);

  const auto invalid = tracker.update(
    std::vector<TargetObservation>{observation(0, 0, 0.0, 0.0, 0.0)}, -1);
  EXPECT_TRUE(invalid.rejected);
  EXPECT_FALSE(invalid.lock_allowed);
  EXPECT_FALSE(invalid.target_lock());
  ASSERT_TRUE(invalid.primary_track.has_value());
  EXPECT_EQ(invalid.state, TrackingState::TempLost);
  EXPECT_EQ(invalid.primary_track->state, TrackingState::TempLost);
  // The safety state is persistent as well as diagnostic: callers that read
  // the tracker directly cannot keep a lock after a corrupt frame clock.
  EXPECT_EQ(tracker.state(), TrackingState::TempLost);
  ASSERT_EQ(tracker.tracks().size(), 1U);
  EXPECT_FALSE(tracker.tracks().front().target_lock());
}

TEST(OfflineTracker, PositionAndAngleJumpsAreRejectedWithoutReplacingEvidence)
{
  OfflineTracker tracker(tracker_config());
  ASSERT_TRUE(tracker.update(std::vector<TargetObservation>{observation(0, 0, 0.0, 0.0, 0.0)}, 0).accepted);

  const auto position_jump = tracker.update(
    std::vector<TargetObservation>{observation(10, 0, 2.0, 0.0, 0.0)}, 10);
  EXPECT_TRUE(position_jump.rejected);
  ASSERT_EQ(position_jump.tracks.size(), 1U);
  EXPECT_NEAR((*position_jump.tracks[0].observation.camera_xyz_m)[0], 0.0, 1e-12);

  const auto angle_jump = tracker.update(
    std::vector<TargetObservation>{observation(20, 0, 0.0, 2.0, 0.0)}, 20);
  EXPECT_TRUE(angle_jump.rejected);
  ASSERT_EQ(angle_jump.tracks.size(), 1U);
  EXPECT_NEAR(*angle_jump.tracks[0].observation.relative_yaw_rad, 0.0, 1e-12);
}

TEST(OfflineTracker, DifferentIdentityCreatesSeparateTrack)
{
  OfflineTracker tracker(tracker_config());
  const auto update = tracker.update(
    std::vector<TargetObservation>{observation(0, 0, 0.0, 0.0, 0.0, 0.8F, 1),
      observation(0, 1, 0.1, 0.0, 0.0, 0.7F, 2)}, 0);
  ASSERT_EQ(update.tracks.size(), 2U);
  EXPECT_NE(update.tracks[0].track_id, update.tracks[1].track_id);
}

TEST(OfflineTracker, TrackIdsRemainStableWhenDetectionOrderChanges)
{
  OfflineTracker tracker(tracker_config());
  const auto first = tracker.update(
    std::vector<TargetObservation>{observation(0, 0, -0.4, 0.0, 0.0, 0.8F),
      observation(0, 1, 0.4, 0.0, 0.0, 0.9F)}, 0);
  ASSERT_EQ(first.tracks.size(), 2U);
  const auto left_id = first.tracks[0].observation.camera_xyz_m->val[0] < 0.0 ?
    first.tracks[0].track_id : first.tracks[1].track_id;
  const auto right_id = first.tracks[0].observation.camera_xyz_m->val[0] > 0.0 ?
    first.tracks[0].track_id : first.tracks[1].track_id;

  const auto second = tracker.update(
    std::vector<TargetObservation>{observation(10, 1, 0.4, 0.0, 0.0, 0.9F),
      observation(10, 0, -0.4, 0.0, 0.0, 0.8F)}, 10);
  ASSERT_EQ(second.tracks.size(), 2U);
  for (const auto & track : second.tracks) {
    if (track.observation.camera_xyz_m->val[0] < 0.0) {
      EXPECT_EQ(track.track_id, left_id);
    } else {
      EXPECT_EQ(track.track_id, right_id);
    }
  }
}

TEST(TargetSelector, ConfidenceIsPrimaryAndSelectionIsDeterministic)
{
  TargetSelector selector;
  const auto high = tracked(2, 0.9F, 700.0F);
  const auto low = tracked(1, 0.8F, 640.0F);
  const auto selected = selector.select({low, high}, 1280, 800);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, 2U);

  const auto repeated = selector.select({high, low}, 1280, 800);
  ASSERT_TRUE(repeated.has_value());
  EXPECT_EQ(repeated->track_id, 2U);
}

TEST(TargetSelector, ConfidenceEpsilonUsesGlobalMaximumAcrossPermutations)
{
  TargetSelectorConfig config{};
  config.confidence_tie_epsilon = 1e-6F;
  // The first two confidence gaps are within epsilon, while the lowest
  // candidate is outside the global epsilon band around the maximum.  All
  // boxes share the same center so the ID tie-break is the only secondary
  // ordering; a pairwise epsilon chain must not let ID 1 or 3 win.
  const std::array<TrackedTarget, 3> fixtures{
    tracked(1, 0.8000000F, 640.0F),
    tracked(2, 0.8000009F, 640.0F),
    tracked(3, 0.8000018F, 640.0F),
  };
  const std::array<std::array<std::size_t, 3>, 6> permutations{{
    {{0U, 1U, 2U}},
    {{0U, 2U, 1U}},
    {{1U, 0U, 2U}},
    {{1U, 2U, 0U}},
    {{2U, 0U, 1U}},
    {{2U, 1U, 0U}},
  }};

  for (const auto & permutation : permutations) {
    std::vector<TrackedTarget> ordered;
    ordered.reserve(permutation.size());
    for (const auto index : permutation) {
      ordered.push_back(fixtures[index]);
    }
    TargetSelector selector(config);
    const auto selected = selector.select(ordered, 1280, 800);
    SCOPED_TRACE(
      "permutation=" + std::to_string(permutation[0]) + "," +
      std::to_string(permutation[1]) + "," + std::to_string(permutation[2]));
    ASSERT_TRUE(selected.has_value());
    EXPECT_EQ(selected->track_id, 2U);
    EXPECT_EQ(selector.diagnostics().candidate_track_id, std::optional<std::uint64_t>(2U));
  }
}

TEST(TargetSelector, PreviousTrackBreaksConfidenceTieThenCenterAndIdBreakTies)
{
  TargetSelector selector;
  const auto first = selector.select(
    {tracked(4, 0.8F, 500.0F), tracked(9, 0.8F, 700.0F)}, 1280, 800);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->track_id, 9U);

  const auto previous_wins = selector.select(
    {tracked(9, 0.8F, 700.0F), tracked(4, 0.8F, 500.0F)}, 1280, 800);
  ASSERT_TRUE(previous_wins.has_value());
  EXPECT_EQ(previous_wins->track_id, 9U);

  selector.reset();
  const auto center_wins = selector.select(
    {tracked(9, 0.8F, 700.0F), tracked(4, 0.8F, 500.0F)}, 1280, 800);
  ASSERT_TRUE(center_wins.has_value());
  EXPECT_EQ(center_wins->track_id, 9U);
}

TEST(TargetSelector, SelectsAvailableReplacementWhenPreviousTargetDisappears)
{
  TargetSelector selector;
  const auto first = selector.select(
    {tracked(3, 0.9F, 640.0F), tracked(8, 0.75F, 700.0F)}, 1280, 800);
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->track_id, 3U);

  // A target switch is observable and deterministic, but does not imply that
  // a real two-robot scenario or firing policy has been validated.
  const auto replacement = selector.select({tracked(8, 0.75F, 700.0F)}, 1280, 800);
  ASSERT_TRUE(replacement.has_value());
  EXPECT_EQ(replacement->track_id, 8U);
  EXPECT_EQ(selector.previous_track_id(), std::optional<std::uint64_t>(8U));
}

TEST(TargetSelector, NonTrackingAndInvalidTracksAreFiltered)
{
  TargetSelector selector;
  auto detecting = tracked(1, 0.99F, 640.0F, TrackingState::Detecting);
  auto invalid = tracked(2, 0.9F, 640.0F);
  invalid.observation.confidence = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(invalid.target_lock());
  EXPECT_FALSE(selector.select({detecting, invalid}, 1280, 800).has_value());
}

TEST(TargetSelector, InvalidImageDimensionsDoNotSelect)
{
  TargetSelector selector;
  EXPECT_FALSE(selector.select({tracked(1, 0.9F, 640.0F)}, 0, 800).has_value());
  EXPECT_FALSE(selector.previous_track_id().has_value());
}

TEST(OfflineAimer, RelativeDebugNeverProducesAbsoluteCommandOrFire)
{
  rm_auto_aim::offline::SafeOfflineAimer aimer(AimerConfig{});
  const auto selected = tracked(1, 0.8F, 640.0F);
  const auto output = aimer.aim(selected);
  EXPECT_TRUE(output.test_only);
  EXPECT_EQ(output.target_lock, rm_auto_aim::pipeline::kTargetLocked);
  EXPECT_TRUE(output.relative_yaw_rad.has_value());
  EXPECT_TRUE(output.relative_pitch_rad.has_value());
  EXPECT_FALSE(output.absolute_command_valid);
  EXPECT_FALSE(output.command_yaw_rad.has_value());
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_EQ(output.safe_command().target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
}

TEST(OfflineAimer, TrackingWithoutRelativeAnglesCannotCreateAbsoluteCommand)
{
  AimerConfig config{};
  config.mode = AimerMode::TestAbsoluteZero;
  config.absolute_zero_configured = true;
  config.yaw_zero_rad = 0.2;
  config.pitch_zero_rad = -0.1;
  rm_auto_aim::offline::SafeOfflineAimer aimer(config);
  auto selected = tracked(1, 0.8F, 640.0F);
  selected.observation.relative_yaw_rad.reset();
  selected.observation.relative_pitch_rad.reset();
  const auto output = aimer.aim(selected);
  EXPECT_EQ(output.target_lock, rm_auto_aim::pipeline::kTargetLocked);
  EXPECT_FALSE(output.absolute_command_valid);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflineAimer, TestAbsoluteZeroProducesMarkedRadAndDegreeCandidate)
{
  AimerConfig config{};
  config.mode = AimerMode::TestAbsoluteZero;
  config.absolute_zero_configured = true;
  config.yaw_zero_rad = 0.1;
  config.pitch_zero_rad = -0.2;
  rm_auto_aim::offline::SafeOfflineAimer aimer(config);
  const auto output = aimer.aim(tracked(1, 0.8F, 640.0F));
  ASSERT_TRUE(output.absolute_command_valid);
  ASSERT_TRUE(output.command_yaw_rad.has_value());
  ASSERT_TRUE(output.command_pitch_rad.has_value());
  EXPECT_NEAR(*output.command_yaw_rad, 0.2, 1e-12);
  EXPECT_NEAR(*output.command_pitch_rad, -0.15, 1e-12);
  EXPECT_NEAR(*output.command_yaw_degree, 0.2 * rm_auto_aim::units::kDegreesPerRadian, 1e-5);
  EXPECT_NEAR(*output.command_pitch_degree, -0.15 * rm_auto_aim::units::kDegreesPerRadian, 1e-5);
  EXPECT_TRUE(output.test_only);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflineAimer, LostTargetIsAlwaysUnlockedAndNeverFires)
{
  rm_auto_aim::offline::SafeOfflineAimer aimer;
  auto lost = tracked(1, 0.8F, 640.0F, TrackingState::TempLost);
  const auto output = aimer.aim(lost, 28.0);
  EXPECT_EQ(output.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_DOUBLE_EQ(output.shoot_speed_mps, 28.0);
}

TEST(OfflineAimer, ShootSpeedIsDiagnosticOnlyAndInvalidValuesAreSafe)
{
  rm_auto_aim::offline::SafeOfflineAimer aimer;
  const auto output = aimer.aim(
    tracked(1, 0.8F, 640.0F), std::numeric_limits<double>::quiet_NaN());
  EXPECT_DOUBLE_EQ(output.shoot_speed_mps, 0.0);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflineAimer, NonFiniteTrackedMotionCannotBecomeLockableOutput)
{
  rm_auto_aim::offline::SafeOfflineAimer aimer;
  auto selected = tracked(1, 0.8F, 640.0F);
  selected.yaw_vel_rad_s = std::numeric_limits<double>::quiet_NaN();
  const auto output = aimer.aim(selected);
  EXPECT_EQ(output.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflineAimer, MalformedTrackingEvidenceCannotBecomeLockableOutput)
{
  rm_auto_aim::offline::SafeOfflineAimer aimer;
  auto selected = tracked(1, 0.8F, 640.0F);
  selected.observation.raw_detection.armor_type =
    rm_auto_aim::detector::RawArmorDetection::ArmorTypeHint::Large;
  EXPECT_FALSE(selected.target_lock());
  const auto output = aimer.aim(selected);
  EXPECT_EQ(output.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(output.fire_command, rm_auto_aim::pipeline::kFireNone);
}

TEST(OfflineGeometry, RelativeAngleSignsFollowGimbalConvention)
{
  const auto left_up = rm_auto_aim::pnp::PnpStage::relative_angles_from_gimbal_translation(
    cv::Vec3d{1.0, 1.0, 1.0});
  ASSERT_TRUE(left_up.has_value());
  EXPECT_GT(left_up->relative_yaw_rad, 0.0);
  EXPECT_GT(left_up->relative_pitch_rad, 0.0);
  const auto right_down = rm_auto_aim::pnp::PnpStage::relative_angles_from_gimbal_translation(
    cv::Vec3d{1.0, -1.0, -1.0});
  ASSERT_TRUE(right_down.has_value());
  EXPECT_LT(right_down->relative_yaw_rad, 0.0);
  EXPECT_LT(right_down->relative_pitch_rad, 0.0);
}

TEST(OfflineConfiguration, InvalidConfigurationsAreRejected)
{
  TrackerConfig tracker_bad = tracker_config();
  tracker_bad.max_velocity_rad_s = -1.0;
  EXPECT_TRUE(tracker_bad.validate().has_value());

  rm_auto_aim::offline::TargetSelectorConfig selector_bad{};
  selector_bad.confidence_tie_epsilon = -1.0F;
  EXPECT_TRUE(selector_bad.validate().has_value());

  AimerConfig aimer_bad{};
  aimer_bad.test_only = false;
  EXPECT_TRUE(aimer_bad.validate().has_value());
}
