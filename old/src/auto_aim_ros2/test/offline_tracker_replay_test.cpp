#include "auto_aim_ros2/offline_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::offline::AssociationResult;
using rm_auto_aim::offline::OfflineTracker;
using rm_auto_aim::offline::TargetObservation;
using rm_auto_aim::offline::TargetSelector;
using rm_auto_aim::offline::TargetSelectorConfig;
using rm_auto_aim::offline::TrackerConfig;
using rm_auto_aim::offline::TrackerUpdate;
using rm_auto_aim::offline::TrackedTarget;
using rm_auto_aim::offline::TrackingState;

constexpr int kImageWidth = 1280;
constexpr int kImageHeight = 800;
constexpr std::int64_t kFramePeriodNs = 10'000'000;

TrackerUpdate update_frame(
  OfflineTracker & tracker, std::vector<TargetObservation> observations,
  std::int64_t stamp_ns)
{
  return tracker.update(observations, stamp_ns);
}

TargetObservation make_observation(
  std::int64_t stamp_ns,
  std::size_t detection_index,
  double x,
  double yaw = 0.0,
  double pitch = 0.0,
  float confidence = 0.8F,
  int class_id = 1,
  rm_auto_aim::pnp::ArmorSize armor_size = rm_auto_aim::pnp::ArmorSize::Small)
{
  TargetObservation result{};
  result.detection_index = detection_index;
  result.class_id = class_id;
  result.armor_size = armor_size;
  result.confidence = confidence;
  result.camera_xyz_m = cv::Vec3d{x, 0.0, 5.0};
  result.relative_yaw_rad = yaw;
  result.relative_pitch_rad = pitch;
  result.reprojection_error_px = 0.5;
  result.stamp_ns = stamp_ns;
  result.valid = true;
  result.geometry_known = true;
  result.raw_detection.class_id = class_id;
  result.raw_detection.confidence = confidence;
  result.raw_detection.bbox = cv::Rect2f(
    static_cast<float>(640.0 + x * 120.0), 380.0F, 40.0F, 40.0F);
  result.raw_detection.keypoints = {
    result.raw_detection.bbox.tl(),
    cv::Point2f(result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y),
    cv::Point2f(result.raw_detection.bbox.x + result.raw_detection.bbox.width,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height),
    cv::Point2f(result.raw_detection.bbox.x,
      result.raw_detection.bbox.y + result.raw_detection.bbox.height)};
  return result;
}

TrackerConfig replay_config()
{
  TrackerConfig config{};
  config.min_detect_count = 2;
  config.max_temp_lost_ms = 100;
  config.max_position_jump_m = 0.75;
  config.max_angle_jump_rad = 0.5;
  config.max_velocity_rad_s = 0.0;
  return config;
}

const rm_auto_aim::offline::TrackedTarget * find_by_x(
  const std::vector<rm_auto_aim::offline::TrackedTarget> & tracks, double x)
{
  for (const auto & track : tracks) {
    if (track.observation.camera_xyz_m.has_value() &&
      std::abs((*track.observation.camera_xyz_m)[0] - x) < 1e-9) {
      return &track;
    }
  }
  return nullptr;
}

std::vector<std::uint64_t> ids_by_x(
  const std::vector<rm_auto_aim::offline::TrackedTarget> & tracks)
{
  std::vector<std::pair<double, std::uint64_t>> values;
  for (const auto & track : tracks) {
    if (track.observation.camera_xyz_m.has_value()) {
      values.emplace_back((*track.observation.camera_xyz_m)[0], track.track_id);
    }
  }
  std::sort(values.begin(), values.end());
  std::vector<std::uint64_t> result;
  for (const auto & value : values) result.push_back(value.second);
  return result;
}

const TrackedTarget * find_by_id(
  const std::vector<TrackedTarget> & tracks, std::uint64_t track_id)
{
  for (const auto & track : tracks) {
    if (track.track_id == track_id) {
      return &track;
    }
  }
  return nullptr;
}

TrackedTarget tracking_candidate(
  std::uint64_t track_id, double x, float confidence)
{
  TrackedTarget result{};
  result.track_id = track_id;
  result.state = TrackingState::Tracking;
  result.observation = make_observation(0, static_cast<std::size_t>(track_id), x,
    0.0, 0.0, confidence);
  result.consecutive_valid = 2;
  result.first_valid_timestamp_ns = 0;
  result.last_valid_timestamp_ns = 0;
  return result;
}

}  // namespace

TEST(OfflineTrackerReplay, SingleTargetContinuousMotionHasStableIdAndFiniteVelocity)
{
  OfflineTracker tracker(replay_config());
  std::uint64_t track_id = 0;
  for (int frame = 0; frame < 8; ++frame) {
    SCOPED_TRACE(frame);
    const auto stamp = static_cast<std::int64_t>(frame) * kFramePeriodNs;
    const auto update = update_frame(tracker,
      {make_observation(stamp, 0, 0.01 * frame, 0.01 * frame, 0.002 * frame)}, stamp);
    ASSERT_TRUE(update.accepted);
    ASSERT_EQ(update.tracks.size(), 1U);
    if (frame == 0) {
      track_id = update.tracks.front().track_id;
      EXPECT_EQ(update.tracks.front().state, TrackingState::Detecting);
    } else {
      EXPECT_EQ(update.tracks.front().track_id, track_id);
      EXPECT_EQ(update.tracks.front().state, TrackingState::Tracking);
      EXPECT_TRUE(std::isfinite(update.tracks.front().yaw_vel_rad_s));
      EXPECT_TRUE(std::isfinite(update.tracks.front().pitch_vel_rad_s));
    }
  }
}

TEST(OfflineTrackerReplay, MultiTargetAndPermutedInputKeepPhysicalTrackIds)
{
  OfflineTracker tracker(replay_config());
  const auto first = update_frame(tracker,
    {make_observation(0, 0, -0.5, 0.0, 0.0, 0.8F),
      make_observation(0, 1, 0.5, 0.0, 0.0, 0.9F)}, 0);
  ASSERT_EQ(first.tracks.size(), 2U);
  const auto expected = ids_by_x(first.tracks);
  ASSERT_EQ(expected.size(), 2U);

  const auto second = update_frame(tracker,
    {make_observation(kFramePeriodNs, 91, 0.51, 0.0, 0.0, 0.9F),
      make_observation(kFramePeriodNs, 17, -0.49, 0.0, 0.0, 0.8F)}, kFramePeriodNs);
  ASSERT_EQ(second.tracks.size(), 2U);
  EXPECT_EQ(ids_by_x(second.tracks), expected);
}

TEST(OfflineTrackerReplay, AssociationMaximizesCardinalityAndIsDeterministic)
{
  auto config = replay_config();
  // Construct an adversarial 2x2 graph from camera-frame position gates:
  // track A (x=0) can use both observations, while track B (x=0.45) can
  // only use the observation at x=0.10.  A shortest-edge greedy pass consumes
  // that observation for A and leaves B unmatched; augmenting paths must move
  // A to x=-0.40 so both tracks survive.
  config.max_position_jump_m = 0.5;
  OfflineTracker tracker(config);
  const auto seed = update_frame(tracker,
    {make_observation(0, 0, 0.0), make_observation(0, 1, 0.45)}, 0);
  ASSERT_EQ(seed.tracks.size(), 2U);
  const auto * track_a = find_by_x(seed.tracks, 0.0);
  const auto * track_b = find_by_x(seed.tracks, 0.45);
  ASSERT_NE(track_a, nullptr);
  ASSERT_NE(track_b, nullptr);
  const auto id_a = track_a->track_id;
  const auto id_b = track_b->track_id;

  const auto update = update_frame(tracker,
    {make_observation(kFramePeriodNs, 10, 0.10),
      make_observation(kFramePeriodNs, 11, -0.40)}, kFramePeriodNs);
  ASSERT_TRUE(update.accepted);
  EXPECT_FALSE(update.rejected);
  EXPECT_EQ(update.valid_observation_count, 2U);
  EXPECT_EQ(update.accepted_count, 2U);
  EXPECT_EQ(update.matched_count, 2U);
  EXPECT_EQ(update.rejected_observation_count, 0U);
  const auto * reassigned_a = find_by_x(update.tracks, -0.40);
  const auto * matched_b = find_by_x(update.tracks, 0.10);
  ASSERT_NE(reassigned_a, nullptr);
  ASSERT_NE(matched_b, nullptr);
  EXPECT_EQ(reassigned_a->track_id, id_a);
  EXPECT_EQ(matched_b->track_id, id_b);
  EXPECT_EQ(reassigned_a->association_result, AssociationResult::Matched);
  EXPECT_EQ(matched_b->association_result, AssociationResult::Matched);

  // Detector vector order is not part of the tie-break.  A fresh tracker with
  // the same physical replay must produce the same IDs and assignment.
  OfflineTracker replay(config);
  const auto replay_seed = update_frame(replay,
    {make_observation(0, 101, 0.45), make_observation(0, 100, 0.0)}, 0);
  ASSERT_EQ(ids_by_x(replay_seed.tracks), ids_by_x(seed.tracks));
  const auto replay_update = update_frame(replay,
    {make_observation(kFramePeriodNs, 111, -0.40),
      make_observation(kFramePeriodNs, 110, 0.10)}, kFramePeriodNs);
  ASSERT_EQ(ids_by_x(replay_update.tracks), ids_by_x(update.tracks));
  EXPECT_EQ(replay_update.matched_count, update.matched_count);
  EXPECT_EQ(replay_update.association_result, update.association_result);
}

TEST(OfflineTrackerReplay, AssociationConflictIsDistinctFromInvalidEvidence)
{
  OfflineTracker tracker(replay_config());
  ASSERT_TRUE(update_frame(tracker, {make_observation(0, 0, 0.0)}, 0).accepted);

  // Two structurally valid observations compete for one active identity.
  // One is matched; the other is a deterministic association conflict, not
  // malformed input and not a new lockable track.
  const auto update = update_frame(tracker,
    {make_observation(kFramePeriodNs, 1, 0.01), make_observation(kFramePeriodNs, 2, 0.02)},
    kFramePeriodNs);
  EXPECT_TRUE(update.accepted);
  EXPECT_TRUE(update.rejected);
  EXPECT_EQ(update.association_result, AssociationResult::RejectedAssociationConflict);
  EXPECT_EQ(update.accepted_count, 1U);
  EXPECT_EQ(update.matched_count, 1U);
  EXPECT_EQ(update.rejected_observation_count, 1U);
  EXPECT_EQ(update.statistics.invalid_count, 0U);
  EXPECT_EQ(update.statistics.rejected_count, 1U);
  ASSERT_EQ(update.tracks.size(), 1U);
  EXPECT_EQ(update.tracks.front().state, TrackingState::Tracking);
}

TEST(OfflineTrackerReplay, CloseCrossingTargetsRemainDistinctWhenWithinGate)
{
  OfflineTracker tracker(replay_config());
  const auto first = update_frame(tracker,
    {make_observation(0, 0, -0.30), make_observation(0, 1, 0.30)}, 0);
  ASSERT_EQ(first.tracks.size(), 2U);
  const auto * left = find_by_x(first.tracks, -0.30);
  const auto * right = find_by_x(first.tracks, 0.30);
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);
  const auto left_id = left->track_id;
  const auto right_id = right->track_id;

  const auto crossing = update_frame(tracker,
    {make_observation(kFramePeriodNs, 2, -0.02), make_observation(kFramePeriodNs, 3, 0.02)},
    kFramePeriodNs);
  ASSERT_EQ(crossing.tracks.size(), 2U);
  const auto * crossed_left = find_by_x(crossing.tracks, -0.02);
  const auto * crossed_right = find_by_x(crossing.tracks, 0.02);
  ASSERT_NE(crossed_left, nullptr);
  ASSERT_NE(crossed_right, nullptr);
  EXPECT_EQ(crossed_left->track_id, left_id);
  EXPECT_EQ(crossed_right->track_id, right_id);
}

TEST(OfflineTrackerReplay, TemporaryOcclusionReacquiresAndLongLossExpires)
{
  OfflineTracker tracker(replay_config());
  ASSERT_TRUE(update_frame(tracker, {make_observation(0, 0, 0.0)}, 0).accepted);
  const auto locked = update_frame(tracker, {make_observation(kFramePeriodNs, 1, 0.01)}, kFramePeriodNs);
  ASSERT_TRUE(locked.target_lock());
  const auto id = locked.tracks.front().track_id;

  const auto temp_lost = update_frame(tracker, {}, 50'000'000);
  ASSERT_EQ(temp_lost.tracks.size(), 1U);
  EXPECT_EQ(temp_lost.tracks.front().state, TrackingState::TempLost);
  EXPECT_FALSE(temp_lost.target_lock());

  const auto reacquired = update_frame(tracker, {make_observation(60'000'000, 2, 0.02)}, 60'000'000);
  ASSERT_EQ(reacquired.tracks.size(), 1U);
  EXPECT_EQ(reacquired.tracks.front().track_id, id);
  EXPECT_FALSE(reacquired.target_lock());

  const auto relocked = update_frame(tracker, {make_observation(70'000'000, 3, 0.03)}, 70'000'000);
  EXPECT_EQ(relocked.tracks.front().track_id, id);
  EXPECT_TRUE(relocked.target_lock());

  const auto lost = update_frame(tracker, {}, 180'000'000);
  ASSERT_EQ(lost.tracks.size(), 1U);
  EXPECT_EQ(lost.tracks.front().state, TrackingState::Lost);
  EXPECT_FALSE(lost.target_lock());
  EXPECT_EQ(lost.tracks.front().track_id, id);
}

TEST(OfflineTrackerReplay, DiagnosticsReportAssociationAndCumulativeStatistics)
{
  OfflineTracker tracker(replay_config());

  const auto first = update_frame(tracker, {make_observation(0, 0, 0.0)}, 0);
  ASSERT_TRUE(first.accepted);
  EXPECT_EQ(first.association_result, AssociationResult::NewTrack);
  EXPECT_EQ(first.new_count, 1U);
  EXPECT_EQ(first.matched_count, 0U);
  EXPECT_EQ(first.reacquired_count, 0U);
  EXPECT_FALSE(first.association_reason.empty());
  ASSERT_EQ(first.tracks.size(), 1U);
  EXPECT_EQ(first.tracks.front().association_result, AssociationResult::NewTrack);
  EXPECT_EQ(first.statistics.frame_count, 1U);
  EXPECT_EQ(first.statistics.observation_count, 1U);
  EXPECT_EQ(first.statistics.valid_count, 1U);
  EXPECT_EQ(first.statistics.accepted_count, 1U);
  EXPECT_EQ(first.statistics.new_count, 1U);

  const auto matched = update_frame(
    tracker, {make_observation(kFramePeriodNs, 1, 0.01)}, kFramePeriodNs);
  ASSERT_TRUE(matched.accepted);
  EXPECT_EQ(matched.association_result, AssociationResult::Matched);
  EXPECT_EQ(matched.matched_count, 1U);
  ASSERT_EQ(matched.tracks.size(), 1U);
  EXPECT_EQ(matched.tracks.front().association_result, AssociationResult::Matched);
  EXPECT_EQ(matched.statistics.frame_count, 2U);
  EXPECT_EQ(matched.statistics.observation_count, 2U);
  EXPECT_EQ(matched.statistics.valid_count, 2U);
  EXPECT_EQ(matched.statistics.accepted_count, 2U);
  EXPECT_EQ(matched.statistics.matched_count, 1U);
  EXPECT_EQ(matched.statistics.new_count, 1U);

  const auto missed = update_frame(tracker, {}, 50'000'000);
  EXPECT_FALSE(missed.accepted);
  EXPECT_FALSE(missed.rejected);
  EXPECT_EQ(missed.association_result, AssociationResult::Missed);
  EXPECT_EQ(missed.missed_count, 1U);
  ASSERT_EQ(missed.tracks.size(), 1U);
  EXPECT_EQ(missed.tracks.front().state, TrackingState::TempLost);
  EXPECT_EQ(missed.tracks.front().association_result, AssociationResult::Missed);
  EXPECT_FALSE(missed.target_lock());
  EXPECT_EQ(missed.statistics.frame_count, 3U);
  EXPECT_EQ(missed.statistics.missed_count, 1U);

  const auto reacquired = update_frame(
    tracker, {make_observation(60'000'000, 2, 0.02)}, 60'000'000);
  ASSERT_TRUE(reacquired.accepted);
  EXPECT_EQ(reacquired.association_result, AssociationResult::Reacquired);
  EXPECT_EQ(reacquired.reacquired_count, 1U);
  ASSERT_EQ(reacquired.tracks.size(), 1U);
  EXPECT_EQ(reacquired.tracks.front().state, TrackingState::Detecting);
  EXPECT_EQ(reacquired.tracks.front().association_result, AssociationResult::Reacquired);
  EXPECT_FALSE(reacquired.target_lock());
  EXPECT_EQ(reacquired.statistics.frame_count, 4U);
  EXPECT_EQ(reacquired.statistics.observation_count, 3U);
  EXPECT_EQ(reacquired.statistics.valid_count, 3U);
  EXPECT_EQ(reacquired.statistics.accepted_count, 3U);
  EXPECT_EQ(reacquired.statistics.matched_count, 2U);
  EXPECT_EQ(reacquired.statistics.new_count, 1U);
  EXPECT_EQ(reacquired.statistics.reacquisition_count, 1U);
  EXPECT_EQ(reacquired.statistics.missed_count, 1U);
}

TEST(OfflineTrackerReplay, TimeoutBoundaryExpiresOldTrackAndCreatesNewId)
{
  OfflineTracker tracker(replay_config());
  ASSERT_TRUE(update_frame(tracker, {make_observation(0, 0, 0.0)}, 0).accepted);
  const auto locked = update_frame(
    tracker, {make_observation(kFramePeriodNs, 1, 0.01)}, kFramePeriodNs);
  ASSERT_TRUE(locked.target_lock());
  const auto old_id = locked.tracks.front().track_id;

  const auto boundary_stamp = kFramePeriodNs + 100'000'000;
  const auto boundary = update_frame(
    tracker, {make_observation(boundary_stamp, 2, 0.02)}, boundary_stamp);
  ASSERT_TRUE(boundary.accepted);
  ASSERT_EQ(boundary.tracks.size(), 2U);
  EXPECT_EQ(boundary.association_result, AssociationResult::NewTrack);
  EXPECT_EQ(boundary.new_count, 1U);
  EXPECT_EQ(boundary.expired_count, 1U);
  EXPECT_FALSE(boundary.target_lock());

  const auto * old_track = find_by_id(boundary.tracks, old_id);
  ASSERT_NE(old_track, nullptr);
  EXPECT_EQ(old_track->state, TrackingState::Lost);
  EXPECT_EQ(old_track->association_result, AssociationResult::Expired);

  const auto * new_track = find_by_x(boundary.tracks, 0.02);
  ASSERT_NE(new_track, nullptr);
  EXPECT_NE(new_track->track_id, old_id);
  EXPECT_GT(new_track->track_id, old_id);
  EXPECT_EQ(new_track->state, TrackingState::Detecting);
  EXPECT_EQ(new_track->association_result, AssociationResult::NewTrack);
  EXPECT_EQ(new_track->first_valid_timestamp_ns, boundary_stamp);
  EXPECT_EQ(boundary.statistics.expired_count, 1U);
  EXPECT_EQ(boundary.statistics.new_count, 2U);
}

TEST(OfflineTargetSelectorReplay, SwitchesReplacementAndNeverReturnsTempLost)
{
  TargetSelectorConfig config{};
  config.confidence_tie_epsilon = 1e-3F;
  TargetSelector selector(config);
  OfflineTracker tracker(replay_config());
  const auto first = update_frame(tracker,
    {make_observation(0, 0, -0.25, 0.0, 0.0, 0.80F), make_observation(0, 1, 0.25, 0.0, 0.0, 0.79F)}, 0);
  const auto second = update_frame(tracker,
    {make_observation(kFramePeriodNs, 2, -0.24, 0.0, 0.0, 0.80F),
      make_observation(kFramePeriodNs, 3, 0.24, 0.0, 0.0, 0.79F)}, kFramePeriodNs);
  auto selected = selector.select(second.tracks, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  const auto original_id = selected->track_id;

  const auto third = update_frame(tracker,
    {make_observation(20'000'000, 4, -0.23, 0.0, 0.0, 0.70F),
      make_observation(20'000'000, 5, 0.23, 0.0, 0.0, 0.99F)}, 20'000'000);
  selected = selector.select(third.tracks, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_NE(selected->track_id, original_id);
  const auto fourth = update_frame(tracker,
    {make_observation(30'000'000, 6, -0.22, 0.0, 0.0, 0.70F),
      make_observation(30'000'000, 7, 0.22, 0.0, 0.0, 0.99F)}, 30'000'000);
  selected = selector.select(fourth.tracks, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_NE(selected->track_id, original_id);

  // A temp-lost track must be filtered even when it had the highest confidence.
  const auto no_observation = update_frame(tracker, {}, 50'000'000);
  selected = selector.select(no_observation.tracks, kImageWidth, kImageHeight);
  EXPECT_FALSE(selected.has_value());
}

TEST(OfflineTargetSelectorReplay, DebounceHoldsThenConfirmsStableReplacement)
{
  TargetSelectorConfig config{};
  config.switch_debounce_frames = 3;
  TargetSelector selector(config);
  const auto previous = tracking_candidate(10, -0.2, 0.80F);
  const auto replacement = tracking_candidate(20, 0.2, 0.95F);

  auto selected = selector.select({previous}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, previous.track_id);
  EXPECT_EQ(selector.diagnostics().switch_reason, "initial_selection");
  EXPECT_EQ(selector.diagnostics().candidate_track_id,
    std::optional<std::uint64_t>(previous.track_id));
  EXPECT_EQ(selector.diagnostics().selected_track_id,
    std::optional<std::uint64_t>(previous.track_id));

  selected = selector.select({replacement, previous}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, previous.track_id);
  EXPECT_FALSE(selector.diagnostics().switched);
  EXPECT_EQ(selector.diagnostics().switch_reason, "debounce_hold");
  EXPECT_EQ(selector.diagnostics().candidate_track_id,
    std::optional<std::uint64_t>(replacement.track_id));
  EXPECT_EQ(selector.diagnostics().selected_track_id,
    std::optional<std::uint64_t>(previous.track_id));
  EXPECT_EQ(selector.diagnostics().debounce_hold_count, 1U);

  selected = selector.select({previous, replacement}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, previous.track_id);
  EXPECT_EQ(selector.diagnostics().switch_reason, "debounce_hold");
  EXPECT_EQ(selector.diagnostics().debounce_hold_count, 2U);

  selected = selector.select({replacement, previous}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, replacement.track_id);
  EXPECT_TRUE(selector.diagnostics().switched);
  EXPECT_EQ(selector.diagnostics().switch_reason, "debounce_confirmed_replacement");
  EXPECT_EQ(selector.diagnostics().switch_count, 1U);
  EXPECT_EQ(selector.diagnostics().selection_count, 4U);
  EXPECT_EQ(selector.previous_track_id(),
    std::optional<std::uint64_t>(replacement.track_id));
}

TEST(OfflineTargetSelectorReplay, UnavailablePreviousIsReplacedWithoutDebounce)
{
  TargetSelectorConfig config{};
  config.switch_debounce_frames = 4;
  TargetSelector selector(config);
  const auto previous = tracking_candidate(10, -0.2, 0.80F);
  const auto replacement = tracking_candidate(20, 0.2, 0.95F);

  auto selected = selector.select({previous}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, previous.track_id);

  selected = selector.select({replacement}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, replacement.track_id);
  EXPECT_TRUE(selector.diagnostics().switched);
  EXPECT_EQ(selector.diagnostics().switch_reason, "previous_target_unavailable");
  EXPECT_EQ(selector.diagnostics().switch_count, 1U);
  EXPECT_EQ(selector.diagnostics().debounce_hold_count, 0U);
  EXPECT_EQ(selector.diagnostics().candidate_count, 1U);
  EXPECT_EQ(selector.diagnostics().selected_track_id,
    std::optional<std::uint64_t>(replacement.track_id));
}

TEST(OfflineTargetSelectorReplay, ConfidenceTieUsesPreviousThenCenterThenId)
{
  TargetSelector selector;
  auto left = make_observation(0, 0, -0.5, 0.0, 0.0, 0.8F);
  auto right = make_observation(0, 1, 0.5, 0.0, 0.0, 0.8F);
  rm_auto_aim::offline::TrackedTarget left_track{};
  left_track.track_id = 7;
  left_track.state = TrackingState::Tracking;
  left_track.observation = left;
  rm_auto_aim::offline::TrackedTarget right_track{};
  right_track.track_id = 3;
  right_track.state = TrackingState::Tracking;
  right_track.observation = right;
  auto selected = selector.select({left_track, right_track}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, 7U);  // left bbox is one pixel closer to center
  selected = selector.select({right_track, left_track}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, 7U);  // previous id wins despite order

  TargetSelector id_selector;
  auto exact_left = make_observation(0, 2, -0.5, 0.0, 0.0, 0.8F);
  auto exact_right = make_observation(0, 3, 0.5, 0.0, 0.0, 0.8F);
  exact_left.raw_detection.bbox.x = 580.0F;
  exact_right.raw_detection.bbox.x = 620.0F;
  left_track.track_id = 11;
  right_track.track_id = 4;
  left_track.observation = exact_left;
  right_track.observation = exact_right;
  selected = id_selector.select({left_track, right_track}, kImageWidth, kImageHeight);
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(selected->track_id, 4U);  // exact confidence + center tie: lower id
}

TEST(OfflineTargetSelectorReplay, ConfidenceEpsilonUsesGlobalMaximumAcrossPermutations)
{
  TargetSelectorConfig config{};
  config.confidence_tie_epsilon = 1e-6F;
  // Every candidate has the same image center.  Only IDs 2 and 3 are in the
  // global epsilon band around the maximum confidence, so ID 2 must win all
  // six input permutations.  A pairwise epsilon comparator incorrectly lets
  // the out-of-band ID 1 participate through a non-transitive chain.
  const std::array<TrackedTarget, 3> fixtures{
    tracking_candidate(1, 0.0, 0.8000000F),
    tracking_candidate(2, 0.0, 0.8000009F),
    tracking_candidate(3, 0.0, 0.8000018F),
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
    const auto selected = selector.select(ordered, kImageWidth, kImageHeight);
    SCOPED_TRACE(
      "permutation=" + std::to_string(permutation[0]) + "," +
      std::to_string(permutation[1]) + "," + std::to_string(permutation[2]));
    ASSERT_TRUE(selected.has_value());
    EXPECT_EQ(selected->track_id, 2U);
    EXPECT_EQ(selector.diagnostics().candidate_track_id,
      std::optional<std::uint64_t>(2U));
  }
}

TEST(OfflineTrackerReplay, InvalidInputsFailClosedAndPreserveSafeLock)
{
  OfflineTracker tracker(replay_config());
  ASSERT_TRUE(update_frame(tracker, {make_observation(100, 0, 0.0)}, 100).accepted);
  ASSERT_TRUE(update_frame(tracker, {make_observation(110, 1, 0.01)}, 110).target_lock());

  const std::vector<std::pair<std::string, TargetObservation>> invalid = {
    {"nan_position", [&] { auto value = make_observation(120, 0, 0.02); value.camera_xyz_m->val[0] = std::numeric_limits<double>::quiet_NaN(); return value; }()},
    {"negative_depth", [&] { auto value = make_observation(130, 0, 0.02); value.camera_xyz_m->val[2] = 0.0; return value; }()},
    {"nan_angle", [&] { auto value = make_observation(140, 0, 0.02); value.relative_yaw_rad = std::numeric_limits<double>::infinity(); return value; }()},
    {"unknown_identity", [&] { auto value = make_observation(150, 0, 0.02); value.class_id = -1; value.raw_detection.class_id = -1; return value; }()},
  };
  for (const auto & entry : invalid) {
    const auto update = update_frame(tracker, {entry.second}, entry.second.stamp_ns);
    EXPECT_TRUE(update.rejected) << entry.first;
    EXPECT_FALSE(update.target_lock()) << entry.first;
  }

  const auto rollback = update_frame(tracker, {make_observation(149, 0, 0.02)}, 149);
  EXPECT_TRUE(rollback.rejected);
  EXPECT_FALSE(rollback.lock_allowed);
  EXPECT_FALSE(rollback.target_lock());
  const auto duplicate = update_frame(tracker, {make_observation(150, 0, 0.02)}, 150);
  EXPECT_TRUE(duplicate.rejected);
  EXPECT_FALSE(duplicate.target_lock());
  const auto negative = update_frame(tracker, {make_observation(-1, 0, 0.02)}, -1);
  EXPECT_TRUE(negative.rejected);
  EXPECT_FALSE(negative.target_lock());
}

TEST(OfflineTrackerReplay, PositionAngleAndVelocityGatesRetainPriorEvidence)
{
  auto config = replay_config();
  config.max_velocity_rad_s = 0.5;
  OfflineTracker tracker(config);
  ASSERT_TRUE(update_frame(tracker, {make_observation(0, 0, 0.0, 0.0)}, 0).accepted);

  const auto position_jump = update_frame(tracker, {make_observation(kFramePeriodNs, 1, 2.0, 0.01)}, kFramePeriodNs);
  EXPECT_TRUE(position_jump.rejected);
  ASSERT_FALSE(position_jump.tracks.empty());
  ASSERT_TRUE(position_jump.tracks.front().observation.camera_xyz_m.has_value());
  EXPECT_NEAR((*position_jump.tracks.front().observation.camera_xyz_m)[0], 0.0, 1e-12);

  const auto angle_jump = update_frame(tracker, {make_observation(20'000'000, 2, 0.01, 2.0)}, 20'000'000);
  EXPECT_TRUE(angle_jump.rejected);
  ASSERT_FALSE(angle_jump.tracks.empty());
  ASSERT_TRUE(angle_jump.tracks.front().observation.relative_yaw_rad.has_value());
  EXPECT_NEAR(*angle_jump.tracks.front().observation.relative_yaw_rad, 0.0, 1e-12);

  const auto velocity_jump = update_frame(tracker, {make_observation(30'000'000, 3, 0.02, 0.02)}, 30'000'000);
  EXPECT_TRUE(velocity_jump.rejected);
}

TEST(OfflineTrackerReplay, ReplayOutputsAreDeterministicAcrossRuns)
{
  const std::vector<std::vector<TargetObservation>> frames = {
    {make_observation(0, 0, -0.4, 0.0, 0.0, 0.8F), make_observation(0, 1, 0.4, 0.0, 0.0, 0.9F)},
    {make_observation(10'000'000, 9, 0.41, 0.01, 0.0, 0.9F), make_observation(10'000'000, 8, -0.39, -0.01, 0.0, 0.8F)},
    {},
    {make_observation(80'000'000, 3, 0.43, 0.03, 0.0, 0.9F), make_observation(80'000'000, 2, -0.37, -0.03, 0.0, 0.8F)},
  };
  auto run = [&] {
      OfflineTracker tracker(replay_config());
      TargetSelector selector;
      std::vector<std::tuple<std::int64_t, TrackingState, std::uint64_t, bool>> result;
      for (const auto & frame : frames) {
        const auto stamp = frame.empty() ? (result.size() == 2 ? 20'000'000 : 0) : frame.front().stamp_ns;
        const auto update = update_frame(tracker, frame, stamp);
        const auto selected = selector.select(update.tracks, kImageWidth, kImageHeight);
        result.emplace_back(stamp, update.state, selected.has_value() ? selected->track_id : 0,
          update.target_lock());
      }
      return result;
    };
  EXPECT_EQ(run(), run());
}
