#include "auto_aim_ros2/offline_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace rm_auto_aim::offline
{
namespace
{
constexpr std::int64_t kNanosecondsPerMillisecond = 1'000'000;
constexpr double kFiniteEpsilon = 1e-12;
constexpr double kPositiveDepthEpsilon = 1e-9;

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

bool finite_vec(const cv::Vec3d & value) noexcept
{
  return finite(value[0]) && finite(value[1]) && finite(value[2]);
}

bool finite_point(const cv::Point2f & value) noexcept
{
  return finite(value.x) && finite(value.y);
}

double point_distance(const cv::Vec3d & first, const cv::Vec3d & second) noexcept
{
  return cv::norm(first - second);
}

const cv::Vec3d * camera_position_for(const TargetObservation & observation) noexcept
{
  if (!observation.camera_xyz_m.has_value()) {
    return nullptr;
  }
  return &observation.camera_xyz_m.value();
}

bool valid_armor_size(pnp::ArmorSize size) noexcept
{
  return size == pnp::ArmorSize::Small || size == pnp::ArmorSize::Large;
}

bool valid_armor_type_hint(detector::RawArmorDetection::ArmorTypeHint hint) noexcept
{
  using ArmorTypeHint = detector::RawArmorDetection::ArmorTypeHint;
  return hint == ArmorTypeHint::Unknown || hint == ArmorTypeHint::Small ||
         hint == ArmorTypeHint::Large;
}

bool armor_hint_agrees_with_size(const TargetObservation & observation) noexcept
{
  using ArmorTypeHint = detector::RawArmorDetection::ArmorTypeHint;
  switch (observation.raw_detection.armor_type) {
    case ArmorTypeHint::Unknown:
      // PnP may have selected its geometry through the reviewed class map.
      return true;
    case ArmorTypeHint::Small:
      return observation.armor_size == pnp::ArmorSize::Small;
    case ArmorTypeHint::Large:
      return observation.armor_size == pnp::ArmorSize::Large;
  }
  return false;
}

const char * target_observation_validation_reason(const TargetObservation & observation) noexcept
{
  if (!observation.valid || !observation.geometry_known) {
    return "PnP observation is not valid geometry";
  }
  if (observation.stamp_ns < 0) {
    return "observation timestamp must not be negative";
  }
  if (observation.class_id < 0 || observation.raw_detection.class_id < 0 ||
    observation.raw_detection.class_id != observation.class_id)
  {
    return "observation class identity is invalid or inconsistent";
  }
  if (observation.raw_detection.color_id < -1) {
    return "raw detection color identity is invalid";
  }
  if (!valid_armor_size(observation.armor_size) ||
    !valid_armor_type_hint(observation.raw_detection.armor_type) ||
    !armor_hint_agrees_with_size(observation))
  {
    return "observation armor identity is invalid or inconsistent";
  }
  if (!finite(observation.confidence) || observation.confidence < 0.0F ||
    observation.confidence > 1.0F || !finite(observation.reprojection_error_px) ||
    observation.reprojection_error_px < 0.0)
  {
    return "observation confidence or reprojection evidence is invalid";
  }
  if (!finite(observation.raw_detection.confidence) ||
    observation.raw_detection.confidence < 0.0F ||
    observation.raw_detection.confidence > 1.0F ||
    !finite(observation.raw_detection.bbox.x) || !finite(observation.raw_detection.bbox.y) ||
    !finite(observation.raw_detection.bbox.width) || !finite(observation.raw_detection.bbox.height) ||
    observation.raw_detection.bbox.width <= 0.0F || observation.raw_detection.bbox.height <= 0.0F)
  {
    return "raw detection evidence is invalid";
  }
  for (const auto & keypoint : observation.raw_detection.keypoints) {
    if (!finite_point(keypoint)) {
      return "raw detection keypoint is not finite";
    }
  }
  const auto * camera_position = camera_position_for(observation);
  if (camera_position == nullptr || !finite_vec(*camera_position) ||
    (*camera_position)[2] <= kPositiveDepthEpsilon)
  {
    return "camera position is invalid or has non-positive depth";
  }
  if (observation.gimbal_xyz_m.has_value() && !finite_vec(*observation.gimbal_xyz_m)) {
    return "gimbal position is not finite";
  }
  if (observation.relative_yaw_rad.has_value() && !finite(*observation.relative_yaw_rad)) {
    return "relative yaw is not finite";
  }
  if (observation.relative_pitch_rad.has_value() && !finite(*observation.relative_pitch_rad)) {
    return "relative pitch is not finite";
  }
  return nullptr;
}

bool same_target_identity(
  const TargetObservation & first,
  const TargetObservation & second) noexcept
{
  return first.class_id == second.class_id && first.armor_size == second.armor_size;
}

bool observation_physical_less(
  const TargetObservation & lhs,
  const TargetObservation & rhs) noexcept
{
  if (lhs.class_id != rhs.class_id) {
    return lhs.class_id < rhs.class_id;
  }
  if (lhs.armor_size != rhs.armor_size) {
    return static_cast<int>(lhs.armor_size) < static_cast<int>(rhs.armor_size);
  }
  const auto & lp = *lhs.camera_xyz_m;
  const auto & rp = *rhs.camera_xyz_m;
  for (int index = 0; index < 3; ++index) {
    if (lp[index] != rp[index]) {
      return lp[index] < rp[index];
    }
  }
  if (lhs.confidence != rhs.confidence) {
    return lhs.confidence > rhs.confidence;
  }
  const auto & lb = lhs.raw_detection.bbox;
  const auto & rb = rhs.raw_detection.bbox;
  if (lb.x != rb.x) return lb.x < rb.x;
  if (lb.y != rb.y) return lb.y < rb.y;
  if (lb.width != rb.width) return lb.width < rb.width;
  if (lb.height != rb.height) return lb.height < rb.height;
  return lhs.detection_index < rhs.detection_index;
}

struct AssociationEdge
{
  std::size_t track_index{0};
  std::size_t observation_index{0};
  double distance_m{0.0};
  double yaw_velocity_rad_s{0.0};
  double pitch_velocity_rad_s{0.0};
};

struct AssociationGateStatus
{
  bool same_identity_active{false};
  bool has_valid_edge{false};
  AssociationResult rejection{AssociationResult::None};
  std::string reason;
};

struct GateEvaluation
{
  AssociationResult result{AssociationResult::None};
  std::string reason;
  double distance_m{0.0};
  double yaw_velocity_rad_s{0.0};
  double pitch_velocity_rad_s{0.0};

  bool accepted() const noexcept
  {
    return result == AssociationResult::Matched;
  }
};

GateEvaluation evaluate_gate(
  const TrackedTarget & track,
  const TargetObservation & observation,
  std::int64_t timestamp_ns,
  const TrackerConfig & config)
{
  GateEvaluation result{};
  const auto elapsed_ns = timestamp_ns - track.last_valid_timestamp_ns;
  if (elapsed_ns <= 0) {
    result.result = AssociationResult::RejectedTimestamp;
    result.reason = "track timestamp is not increasing";
    return result;
  }
  const auto * previous_position = camera_position_for(track.observation);
  const auto * current_position = camera_position_for(observation);
  if (previous_position == nullptr || current_position == nullptr) {
    result.result = AssociationResult::RejectedInvalid;
    result.reason = "camera association evidence is unavailable";
    return result;
  }
  result.distance_m = point_distance(*previous_position, *current_position);
  if (!finite(result.distance_m) || result.distance_m > config.max_position_jump_m) {
    result.result = AssociationResult::RejectedPositionJump;
    result.reason = "camera position jump exceeds tracker limit";
    return result;
  }
  if (track.observation.relative_yaw_rad.has_value() && observation.relative_yaw_rad.has_value() &&
    std::abs(*observation.relative_yaw_rad - *track.observation.relative_yaw_rad) >
    config.max_angle_jump_rad)
  {
    result.result = AssociationResult::RejectedAngleJump;
    result.reason = "relative yaw jump exceeds tracker limit";
    return result;
  }
  if (track.observation.relative_pitch_rad.has_value() && observation.relative_pitch_rad.has_value() &&
    std::abs(*observation.relative_pitch_rad - *track.observation.relative_pitch_rad) >
    config.max_angle_jump_rad)
  {
    result.result = AssociationResult::RejectedAngleJump;
    result.reason = "relative pitch jump exceeds tracker limit";
    return result;
  }
  const double dt_s = static_cast<double>(elapsed_ns) / 1'000'000'000.0;
  if (track.observation.relative_yaw_rad.has_value() && observation.relative_yaw_rad.has_value()) {
    result.yaw_velocity_rad_s =
      (*observation.relative_yaw_rad - *track.observation.relative_yaw_rad) / dt_s;
  }
  if (track.observation.relative_pitch_rad.has_value() && observation.relative_pitch_rad.has_value()) {
    result.pitch_velocity_rad_s =
      (*observation.relative_pitch_rad - *track.observation.relative_pitch_rad) / dt_s;
  }
  if (!finite(result.yaw_velocity_rad_s) || !finite(result.pitch_velocity_rad_s) ||
    (config.max_velocity_rad_s > 0.0 &&
    (std::abs(result.yaw_velocity_rad_s) > config.max_velocity_rad_s ||
    std::abs(result.pitch_velocity_rad_s) > config.max_velocity_rad_s)))
  {
    result.result = AssociationResult::RejectedVelocity;
    result.reason = !finite(result.yaw_velocity_rad_s) || !finite(result.pitch_velocity_rad_s) ?
      "computed angular velocity is not finite" : "computed angular velocity exceeds tracker limit";
    return result;
  }
  result.result = AssociationResult::Matched;
  result.reason = "camera identity and motion gates matched";
  return result;
}

int rejection_priority(AssociationResult result) noexcept
{
  switch (result) {
    case AssociationResult::RejectedPositionJump:
      return 1;
    case AssociationResult::RejectedAngleJump:
      return 2;
    case AssociationResult::RejectedVelocity:
      return 3;
    case AssociationResult::RejectedTimestamp:
      return 4;
    case AssociationResult::RejectedInvalid:
      return 5;
    default:
      return 100;
  }
}

bool better_primary_track(const TrackedTarget & candidate, const TrackedTarget & current) noexcept;

void set_primary_track(TrackerUpdate & update)
{
  update.primary_track.reset();
  for (const auto & track : update.tracks) {
    if (!update.primary_track.has_value() || better_primary_track(track, *update.primary_track)) {
      update.primary_track = track;
    }
  }
  update.state = update.primary_track.has_value() ?
    update.primary_track->state : TrackingState::Lost;
}

cv::Point2f detection_center(const TargetObservation & observation) noexcept
{
  return observation.raw_detection.bbox.tl() + cv::Point2f(
    observation.raw_detection.bbox.width * 0.5F,
    observation.raw_detection.bbox.height * 0.5F);
}

std::optional<double> center_distance(
  const TargetObservation & observation,
  int image_width,
  int image_height) noexcept
{
  if (image_width <= 0 || image_height <= 0) {
    return std::nullopt;
  }
  const auto center = detection_center(observation);
  const auto image_center = cv::Point2f(
    static_cast<float>(image_width) * 0.5F,
    static_cast<float>(image_height) * 0.5F);
  const double distance = cv::norm(center - image_center);
  return finite(distance) ? std::optional<double>(distance) : std::nullopt;
}

bool better_primary_track(const TrackedTarget & candidate, const TrackedTarget & current) noexcept
{
  const auto state_rank = [](TrackingState state) {
      switch (state) {
        case TrackingState::Tracking:
          return 3;
        case TrackingState::Detecting:
          return 2;
        case TrackingState::TempLost:
          return 1;
        case TrackingState::Lost:
          return 0;
      }
      return -1;
    };
  if (state_rank(candidate.state) != state_rank(current.state)) {
    return state_rank(candidate.state) > state_rank(current.state);
  }
  if (candidate.target_lock() != current.target_lock()) {
    return candidate.target_lock();
  }
  if (candidate.observation.confidence != current.observation.confidence) {
    return candidate.observation.confidence > current.observation.confidence;
  }
  return candidate.track_id < current.track_id;
}

}  // namespace

const char * tracking_state_name(TrackingState state) noexcept
{
  switch (state) {
    case TrackingState::Lost:
      return "lost";
    case TrackingState::Detecting:
      return "detecting";
    case TrackingState::Tracking:
      return "tracking";
    case TrackingState::TempLost:
      return "temp_lost";
  }
  return "unknown";
}

const char * association_result_name(AssociationResult result) noexcept
{
  switch (result) {
    case AssociationResult::None:
      return "none";
    case AssociationResult::NewTrack:
      return "new_track";
    case AssociationResult::Matched:
      return "matched";
    case AssociationResult::Reacquired:
      return "reacquired";
    case AssociationResult::RejectedInvalid:
      return "rejected_invalid";
    case AssociationResult::RejectedAssociationConflict:
      return "rejected_association_conflict";
    case AssociationResult::RejectedTimestamp:
      return "rejected_timestamp";
    case AssociationResult::RejectedPositionJump:
      return "rejected_position_jump";
    case AssociationResult::RejectedAngleJump:
      return "rejected_angle_jump";
    case AssociationResult::RejectedVelocity:
      return "rejected_velocity";
    case AssociationResult::Missed:
      return "missed";
    case AssociationResult::Expired:
      return "expired";
  }
  return "unknown";
}

TargetObservation make_target_observation(
  const pnp::PoseObservation & pose,
  std::int64_t stamp_ns,
  std::size_t detection_index) noexcept
{
  TargetObservation result{};
  result.raw_detection = pose.raw_detection;
  result.detection_index = detection_index;
  result.class_id = pose.raw_detection.class_id;
  result.armor_size = pose.armor_size;
  result.confidence = pose.raw_detection.confidence;
  result.stamp_ns = stamp_ns;
  result.valid = pose.valid;
  result.geometry_known = pose.valid;
  if (pose.valid) {
    result.camera_xyz_m = pose.translation_in_camera_m;
    result.gimbal_xyz_m = pose.translation_in_gimbal_m;
    if (pose.relative_angles_in_gimbal.has_value()) {
      result.relative_yaw_rad = pose.relative_angles_in_gimbal->relative_yaw_rad;
      result.relative_pitch_rad = pose.relative_angles_in_gimbal->relative_pitch_rad;
    }
    result.reprojection_error_px = pose.reprojection_error_px;
  }
  return result;
}

bool is_valid_target_observation(const TargetObservation & observation) noexcept
{
  return target_observation_validation_reason(observation) == nullptr;
}

std::optional<std::string> TrackerConfig::validate() const
{
  if (min_detect_count <= 0) {
    return "min_detect_count must be positive";
  }
  if (max_temp_lost_ms < 0) {
    return "max_temp_lost_ms must not be negative";
  }
  if (!finite(max_position_jump_m) || max_position_jump_m <= 0.0) {
    return "max_position_jump_m must be positive and finite";
  }
  if (!finite(max_angle_jump_rad) || max_angle_jump_rad <= 0.0) {
    return "max_angle_jump_rad must be positive and finite";
  }
  if (!finite(max_velocity_rad_s) || max_velocity_rad_s < 0.0) {
    return "max_velocity_rad_s must be finite and non-negative";
  }
  return std::nullopt;
}

OfflineTracker::OfflineTracker(TrackerConfig config) : config_(std::move(config))
{
  if (const auto error = config_.validate(); error.has_value()) {
    throw std::invalid_argument("invalid tracker configuration: " + *error);
  }
}

TrackerUpdate OfflineTracker::make_update(
  std::int64_t timestamp_ns,
  bool accepted,
  bool rejected,
  bool had_observation,
  std::string reason,
  std::size_t valid_count,
  std::size_t accepted_count,
  std::size_t rejected_count,
  std::size_t matched_count,
  std::size_t new_count,
  std::size_t reacquired_count,
  std::size_t missed_count,
  std::size_t expired_count,
  AssociationResult association_result) const
{
  TrackerUpdate result{};
  result.accepted = accepted;
  result.rejected = rejected;
  result.had_observation = had_observation;
  result.association_result = association_result;
  result.association_reason = std::move(reason);
  if (rejected) {
    result.rejection_reason = result.association_reason;
  }
  result.timestamp_ns = timestamp_ns;
  result.valid_observation_count = valid_count;
  result.accepted_count = accepted_count;
  result.accepted_observation_count = accepted_count;
  result.rejected_observation_count = rejected_count;
  result.matched_count = matched_count;
  result.new_count = new_count;
  result.reacquired_count = reacquired_count;
  result.missed_count = missed_count;
  result.expired_count = expired_count;
  result.statistics = statistics_;
  result.tracks = tracks_;
  set_primary_track(result);
  return result;
}

TrackerUpdate OfflineTracker::update(
  const std::optional<TargetObservation> & observation,
  std::int64_t timestamp_ns)
{
  if (!observation.has_value()) {
    return update(std::vector<TargetObservation>{}, timestamp_ns);
  }
  return update(std::vector<TargetObservation>{*observation}, timestamp_ns);
}

TrackerUpdate OfflineTracker::update(
  const std::vector<TargetObservation> & observations,
  std::int64_t timestamp_ns)
{
  ++statistics_.frame_count;
  statistics_.observation_count += observations.size();

  const auto rejected_timestamp_update = [&](const char * reason) {
      ++statistics_.rejected_timestamp_count;
      statistics_.rejected_count += observations.size();
      // A bad frame clock is itself a safety event.  Latch every active
      // persistent track out of tracking so callers that inspect
      // `tracks()`/`state()` instead of this returned snapshot cannot keep a
      // stale lock alive.  The last valid sample and frame clock remain
      // unchanged; a later strictly newer valid frame must reacquire through
      // the normal confirmation threshold.
      for (auto & track : tracks_) {
        if (track.state == TrackingState::Lost) {
          continue;
        }
        track.state = TrackingState::TempLost;
        track.consecutive_valid = 0;
        track.yaw_vel_rad_s = 0.0;
        track.pitch_vel_rad_s = 0.0;
        track.updated_this_frame = false;
        track.association_result = AssociationResult::RejectedTimestamp;
        track.association_reason = reason;
      }
      auto result = make_update(timestamp_ns, false, true, !observations.empty(), reason, 0, 0,
          observations.size(), 0, 0, 0, 0, 0, AssociationResult::RejectedTimestamp);
      result.lock_allowed = false;
      return result;
    };

  if (timestamp_ns < 0) {
    return rejected_timestamp_update("timestamp must not be negative");
  }
  if (last_update_timestamp_ns_ >= 0 && timestamp_ns <= last_update_timestamp_ns_) {
    return rejected_timestamp_update("timestamp is not strictly increasing");
  }

  last_update_timestamp_ns_ = timestamp_ns;

  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(observations.size());
  std::size_t rejected_count = 0;
  std::size_t accepted_count = 0;
  std::size_t matched_count = 0;
  std::size_t new_count = 0;
  std::size_t reacquired_count = 0;
  std::size_t missed_count = 0;
  std::size_t expired_count = 0;
  AssociationResult first_rejection = AssociationResult::None;
  std::string first_rejection_reason;
  const auto note_rejection = [&](AssociationResult result, const std::string & reason,
      bool invalid_evidence = false) {
      ++rejected_count;
      ++statistics_.rejected_count;
      switch (result) {
        case AssociationResult::RejectedInvalid:
          if (invalid_evidence) {
            ++statistics_.invalid_count;
          }
          break;
        case AssociationResult::RejectedTimestamp:
          ++statistics_.rejected_timestamp_count;
          break;
        case AssociationResult::RejectedPositionJump:
          ++statistics_.position_jump_count;
          break;
        case AssociationResult::RejectedAngleJump:
          ++statistics_.angle_jump_count;
          break;
        case AssociationResult::RejectedVelocity:
          ++statistics_.velocity_count;
          break;
        default:
          break;
      }
      if (first_rejection == AssociationResult::None) {
        first_rejection = result;
        first_rejection_reason = reason;
      }
    };
  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto & observation = observations[index];
    if (observation.stamp_ns != timestamp_ns) {
      note_rejection(AssociationResult::RejectedTimestamp,
        "observation timestamp does not match frame timestamp");
    } else if (const auto * error = target_observation_validation_reason(observation); error != nullptr) {
      note_rejection(AssociationResult::RejectedInvalid, error, true);
    } else {
      valid_indices.push_back(index);
      ++statistics_.valid_count;
    }
  }

  // Sort by physical evidence, never by vector order. New IDs and all-edge
  // matching therefore stay stable when detector output order changes.
  std::sort(valid_indices.begin(), valid_indices.end(), [&observations](std::size_t first, std::size_t second) {
    return observation_physical_less(observations[first], observations[second]);
  });

  const auto timeout_ns = static_cast<std::int64_t>(config_.max_temp_lost_ms) *
    kNanosecondsPerMillisecond;
  const std::size_t existing_track_count = tracks_.size();
  std::vector<bool> preexpired(existing_track_count, false);
  std::vector<bool> track_updated(existing_track_count, false);
  std::vector<AssociationResult> track_failure(existing_track_count, AssociationResult::None);
  std::vector<std::string> track_failure_reason(existing_track_count);

  // Expire before building edges. A detection at the timeout boundary is a
  // new capture, never a stale-ID revival.
  for (std::size_t index = 0; index < existing_track_count; ++index) {
    auto & track = tracks_[index];
    track.updated_this_frame = false;
    if (track.state == TrackingState::Lost) {
      continue;
    }
    track.association_result = AssociationResult::None;
    track.association_reason.clear();
    if (track.last_valid_timestamp_ns < 0) {
      continue;
    }
    const auto elapsed_ns = timestamp_ns - track.last_valid_timestamp_ns;
    if (elapsed_ns >= timeout_ns) {
      preexpired[index] = true;
      track.state = TrackingState::Lost;
      track.consecutive_valid = 0;
      ++track.consecutive_missed;
      ++track.missed_count;
      ++track.expired_count;
      track.yaw_vel_rad_s = 0.0;
      track.pitch_vel_rad_s = 0.0;
      track.association_result = AssociationResult::Expired;
      track.association_reason = "temporary-loss timeout expired before association";
      ++missed_count;
      ++expired_count;
      ++statistics_.missed_count;
      ++statistics_.expired_count;
    }
  }

  std::vector<AssociationGateStatus> observation_status(observations.size());
  std::vector<AssociationEdge> edges;
  for (const auto observation_index : valid_indices) {
    const auto & observation = observations[observation_index];
    auto & status = observation_status[observation_index];
    for (std::size_t track_index = 0; track_index < existing_track_count; ++track_index) {
      const auto & track = tracks_[track_index];
      if (preexpired[track_index] || track.state == TrackingState::Lost ||
        track.last_valid_timestamp_ns < 0 || !is_valid_target_observation(track.observation) ||
        !same_target_identity(track.observation, observation))
      {
        continue;
      }
      status.same_identity_active = true;
      const auto gate = evaluate_gate(track, observation, timestamp_ns, config_);
      if (gate.accepted()) {
        status.has_valid_edge = true;
        edges.push_back(AssociationEdge{track_index, observation_index, gate.distance_m,
          gate.yaw_velocity_rad_s, gate.pitch_velocity_rad_s});
        continue;
      }
      if (status.rejection == AssociationResult::None ||
        rejection_priority(gate.result) < rejection_priority(status.rejection))
      {
        status.rejection = gate.result;
        status.reason = gate.reason;
      }
      if (track_failure[track_index] == AssociationResult::None ||
        rejection_priority(gate.result) < rejection_priority(track_failure[track_index]))
      {
        track_failure[track_index] = gate.result;
        track_failure_reason[track_index] = gate.reason;
      }
    }
  }

  std::sort(edges.begin(), edges.end(), [&](const AssociationEdge & lhs, const AssociationEdge & rhs) {
    if (lhs.distance_m != rhs.distance_m) return lhs.distance_m < rhs.distance_m;
    const auto lhs_id = tracks_[lhs.track_index].track_id;
    const auto rhs_id = tracks_[rhs.track_index].track_id;
    if (lhs_id != rhs_id) return lhs_id < rhs_id;
    const auto & lhs_observation = observations[lhs.observation_index];
    const auto & rhs_observation = observations[rhs.observation_index];
    if (observation_physical_less(lhs_observation, rhs_observation)) return true;
    if (observation_physical_less(rhs_observation, lhs_observation)) return false;
    return lhs.observation_index < rhs.observation_index;
  });

  // Find a maximum-cardinality matching instead of consuming edges greedily.
  // A globally shortest edge can otherwise strand a track that only has that
  // observation, even though the other track has a feasible alternative.  The
  // edge list is already sorted by distance, track ID, and stable observation
  // evidence above; preserving that order in each adjacency list makes the
  // augmenting-path result deterministic while still maximizing matches.
  std::vector<std::vector<std::size_t>> adjacency(existing_track_count);
  for (std::size_t edge_index = 0; edge_index < edges.size(); ++edge_index) {
    adjacency[edges[edge_index].track_index].push_back(edge_index);
  }
  std::vector<std::size_t> track_order(existing_track_count);
  for (std::size_t track_index = 0; track_index < existing_track_count; ++track_index) {
    track_order[track_index] = track_index;
  }
  std::sort(track_order.begin(), track_order.end(), [&](std::size_t lhs, std::size_t rhs) {
    const auto lhs_id = tracks_[lhs].track_id;
    const auto rhs_id = tracks_[rhs].track_id;
    return lhs_id == rhs_id ? lhs < rhs : lhs_id < rhs_id;
  });

  std::vector<std::optional<std::size_t>> matched_track_for_observation(observations.size());
  std::vector<std::optional<std::size_t>> matched_edge_for_track(existing_track_count);
  std::function<bool(std::size_t, std::vector<bool> &)> augment =
    [&](std::size_t track_index, std::vector<bool> & seen_observations) {
      for (const auto edge_index : adjacency[track_index]) {
        const auto observation_index = edges[edge_index].observation_index;
        if (seen_observations[observation_index]) {
          continue;
        }
        seen_observations[observation_index] = true;
        const auto incumbent = matched_track_for_observation[observation_index];
        if (!incumbent.has_value() || augment(*incumbent, seen_observations)) {
          matched_track_for_observation[observation_index] = track_index;
          matched_edge_for_track[track_index] = edge_index;
          return true;
        }
      }
      return false;
    };

  for (const auto track_index : track_order) {
    std::vector<bool> seen_observations(observations.size(), false);
    (void)augment(track_index, seen_observations);
  }

  std::vector<bool> observation_assigned(observations.size(), false);
  std::vector<AssociationEdge> assignments;
  assignments.reserve(edges.size());
  for (const auto track_index : track_order) {
    if (!matched_edge_for_track[track_index].has_value()) {
      continue;
    }
    const auto & edge = edges[*matched_edge_for_track[track_index]];
    track_updated[track_index] = true;
    observation_assigned[edge.observation_index] = true;
    assignments.push_back(edge);
  }

  for (const auto & edge : assignments) {
    auto & track = tracks_[edge.track_index];
    const bool recovering = track.state == TrackingState::TempLost;
    track.observation = observations[edge.observation_index];
    track.updated_this_frame = true;
    track.consecutive_missed = 0;
    track.consecutive_valid = recovering ? 1 : track.consecutive_valid + 1;
    if (recovering) {
      track.first_valid_timestamp_ns = timestamp_ns;
    }
    track.last_valid_timestamp_ns = timestamp_ns;
    track.yaw_vel_rad_s = edge.yaw_velocity_rad_s;
    track.pitch_vel_rad_s = edge.pitch_velocity_rad_s;
    track.state = track.consecutive_valid >= config_.min_detect_count ?
      TrackingState::Tracking : TrackingState::Detecting;
    track.association_result = recovering ? AssociationResult::Reacquired : AssociationResult::Matched;
    track.association_reason = recovering ?
      "temporary-loss track reacquired; confirmation sequence restarted" :
      "camera identity and motion gates matched";
    ++track.accepted_count;
    ++track.matched_count;
    ++accepted_count;
    ++matched_count;
    ++statistics_.accepted_count;
    ++statistics_.matched_count;
    if (recovering) {
      ++track.reacquisition_count;
      ++reacquired_count;
      ++statistics_.reacquisition_count;
    }
  }

  for (const auto observation_index : valid_indices) {
    if (observation_assigned[observation_index]) {
      continue;
    }
    const auto & observation = observations[observation_index];
    const auto & status = observation_status[observation_index];
    if (status.same_identity_active) {
      const AssociationResult result = status.has_valid_edge ?
        AssociationResult::RejectedAssociationConflict :
        (status.rejection == AssociationResult::None ? AssociationResult::RejectedInvalid : status.rejection);
      const std::string reason = status.has_valid_edge ?
        "all valid association edges were consumed by deterministic matching" :
        (status.reason.empty() ? "association has no valid edge" : status.reason);
      note_rejection(result, reason);
      for (std::size_t track_index = 0; track_index < existing_track_count; ++track_index) {
        auto & track = tracks_[track_index];
        if (!track_updated[track_index] && !preexpired[track_index] &&
          track.state != TrackingState::Lost && same_target_identity(track.observation, observation) &&
          track_failure[track_index] == AssociationResult::None)
        {
          track_failure[track_index] = result;
          track_failure_reason[track_index] = reason;
        }
      }
      continue;
    }

    TrackedTarget track{};
    track.track_id = next_track_id_++;
    track.state = config_.min_detect_count <= 1 ? TrackingState::Tracking : TrackingState::Detecting;
    track.observation = observation;
    track.consecutive_valid = 1;
    track.first_valid_timestamp_ns = timestamp_ns;
    track.last_valid_timestamp_ns = timestamp_ns;
    track.updated_this_frame = true;
    track.association_result = AssociationResult::NewTrack;
    track.association_reason = "new camera-frame identity track created";
    track.accepted_count = 1;
    tracks_.push_back(std::move(track));
    ++accepted_count;
    ++new_count;
    ++statistics_.accepted_count;
    ++statistics_.new_count;
  }

  for (std::size_t track_index = 0; track_index < existing_track_count; ++track_index) {
    auto & track = tracks_[track_index];
    if (track_updated[track_index] || preexpired[track_index] || track.state == TrackingState::Lost ||
      track.last_valid_timestamp_ns < 0)
    {
      continue;
    }
    ++track.consecutive_missed;
    ++track.missed_count;
    ++missed_count;
    ++statistics_.missed_count;
    track.yaw_vel_rad_s = 0.0;
    track.pitch_vel_rad_s = 0.0;
    const auto elapsed_ns = timestamp_ns - track.last_valid_timestamp_ns;
    if (elapsed_ns >= timeout_ns) {
      track.state = TrackingState::Lost;
      track.consecutive_valid = 0;
      ++track.expired_count;
      ++expired_count;
      ++statistics_.expired_count;
      track.association_result = AssociationResult::Expired;
      track.association_reason = "temporary-loss timeout expired";
      continue;
    }
    track.state = TrackingState::TempLost;
    if (track_failure[track_index] != AssociationResult::None) {
      track.association_result = track_failure[track_index];
      track.association_reason = track_failure_reason[track_index];
      ++track.rejected_count;
    } else {
      track.association_result = AssociationResult::Missed;
      track.association_reason = "no accepted observation for active track";
    }
  }

  AssociationResult association_result = AssociationResult::None;
  std::string association_reason;
  if (first_rejection != AssociationResult::None) {
    association_result = first_rejection;
    association_reason = first_rejection_reason;
  } else if (reacquired_count > 0) {
    association_result = AssociationResult::Reacquired;
    association_reason = "temporary-loss track reacquired";
  } else if (matched_count > 0) {
    association_result = AssociationResult::Matched;
    association_reason = "one or more camera-frame tracks matched";
  } else if (new_count > 0) {
    association_result = AssociationResult::NewTrack;
    association_reason = "one or more new camera-frame tracks created";
  } else if (expired_count > 0) {
    association_result = AssociationResult::Expired;
    association_reason = "one or more tracks expired";
  } else if (missed_count > 0) {
    association_result = AssociationResult::Missed;
    association_reason = "one or more active tracks were missed";
  }
  return make_update(timestamp_ns, accepted_count > 0, rejected_count > 0, !observations.empty(),
    association_reason, valid_indices.size(), accepted_count, rejected_count, matched_count, new_count,
    reacquired_count, missed_count, expired_count, association_result);
}

void OfflineTracker::reset() noexcept
{
  tracks_.clear();
  statistics_ = {};
  next_track_id_ = 1;
  last_update_timestamp_ns_ = -1;
}

const TrackerConfig & OfflineTracker::config() const noexcept
{
  return config_;
}

TrackingState OfflineTracker::state() const noexcept
{
  const TrackedTarget * primary = nullptr;
  for (const auto & track : tracks_) {
    if (primary == nullptr || better_primary_track(track, *primary)) {
      primary = &track;
    }
  }
  return primary == nullptr ? TrackingState::Lost : primary->state;
}

const std::vector<TrackedTarget> & OfflineTracker::tracks() const noexcept
{
  return tracks_;
}

const TrackerStatistics & OfflineTracker::statistics() const noexcept
{
  return statistics_;
}

std::optional<std::string> TargetSelectorConfig::validate() const
{
  if (!finite(confidence_tie_epsilon) || confidence_tie_epsilon < 0.0F) {
    return "confidence_tie_epsilon must be finite and non-negative";
  }
  if (switch_debounce_frames <= 0) {
    return "switch_debounce_frames must be positive";
  }
  return std::nullopt;
}

TargetSelector::TargetSelector(TargetSelectorConfig config) : config_(std::move(config))
{
  if (const auto error = config_.validate(); error.has_value()) {
    throw std::invalid_argument("invalid target selector configuration: " + *error);
  }
}

std::optional<TrackedTarget> TargetSelector::select(
  const std::vector<TrackedTarget> & tracks,
  int image_width,
  int image_height)
{
  // Count selector invocations, including safe no-candidate returns.  The
  // API has no frame token, so debounce is intentionally defined over
  // consecutive select calls supplied by the offline caller.
  ++diagnostics_.selection_count;
  diagnostics_.candidate_count = 0;
  diagnostics_.switched = false;
  diagnostics_.candidate_track_id.reset();
  diagnostics_.selected_track_id.reset();
  diagnostics_.switch_reason = "none";
  if (image_width <= 0 || image_height <= 0) {
    previous_track_id_.reset();
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
    ++diagnostics_.no_candidate_count;
    diagnostics_.switch_reason = "invalid_image_dimensions";
    return std::nullopt;
  }

  struct Candidate
  {
    const TrackedTarget * track{nullptr};
    double center_distance_px{0.0};
  };
  std::vector<Candidate> candidates;
  candidates.reserve(tracks.size());
  for (const auto & candidate : tracks) {
    if (!candidate.target_lock() || !is_valid_target_observation(candidate.observation)) {
      continue;
    }
    const auto candidate_center_distance = center_distance(
      candidate.observation, image_width, image_height);
    if (!candidate_center_distance.has_value()) {
      continue;
    }
    candidates.push_back(Candidate{&candidate, *candidate_center_distance});
  }

  diagnostics_.candidate_count = candidates.size();
  if (candidates.empty()) {
    previous_track_id_.reset();
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
    ++diagnostics_.no_candidate_count;
    diagnostics_.switch_reason = "no_valid_tracking_candidates";
    return std::nullopt;
  }

  // Do not fold confidence with pairwise epsilon comparisons: that relation
  // is non-transitive (A~B, B~C, but A !~ C) and can make the result depend on
  // detector vector order.  First establish one global confidence anchor,
  // then apply the remaining deterministic tie-breaks to that fixed set.
  const float maximum_confidence = std::max_element(
    candidates.begin(), candidates.end(), [](const Candidate & lhs, const Candidate & rhs) {
      return lhs.track->observation.confidence < rhs.track->observation.confidence;
    })->track->observation.confidence;
  std::vector<const Candidate *> confidence_candidates;
  confidence_candidates.reserve(candidates.size());
  for (const auto & candidate : candidates) {
    if (maximum_confidence - candidate.track->observation.confidence <=
      config_.confidence_tie_epsilon)
    {
      confidence_candidates.push_back(&candidate);
    }
  }

  const auto previous_it = std::find_if(
    confidence_candidates.begin(), confidence_candidates.end(), [&](const Candidate * candidate) {
      return previous_track_id_.has_value() && candidate->track->track_id == *previous_track_id_;
    });
  const Candidate * ranked = previous_it != confidence_candidates.end() ? *previous_it : nullptr;
  if (ranked == nullptr) {
    // The confidence set is fixed globally.  Use exact center distance and
    // then track ID as a total, order-independent tie-break.
    ranked = *std::min_element(
      confidence_candidates.begin(), confidence_candidates.end(), [](const Candidate * lhs,
        const Candidate * rhs) {
        if (lhs->center_distance_px != rhs->center_distance_px) {
          return lhs->center_distance_px < rhs->center_distance_px;
        }
        return lhs->track->track_id < rhs->track->track_id;
      });
  }
  const Candidate * previous = nullptr;
  for (const auto & candidate : candidates) {
    if (previous_track_id_.has_value() && candidate.track->track_id == *previous_track_id_) {
      previous = &candidate;
    }
  }
  diagnostics_.candidate_track_id = ranked->track->track_id;

  const Candidate * selected = nullptr;
  if (!previous_track_id_.has_value()) {
    selected = ranked;
    diagnostics_.switch_reason = "initial_selection";
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
  } else if (previous == nullptr) {
    selected = ranked;
    diagnostics_.switch_reason = "previous_target_unavailable";
    diagnostics_.switched = selected->track->track_id != *previous_track_id_;
    if (diagnostics_.switched) {
      ++diagnostics_.switch_count;
    }
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
  } else if (ranked->track->track_id == previous->track->track_id) {
    selected = previous;
    diagnostics_.switch_reason = "kept_previous";
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
  } else if (config_.switch_debounce_frames == 1) {
    selected = ranked;
    diagnostics_.switched = true;
    ++diagnostics_.switch_count;
    diagnostics_.switch_reason = "ranked_replacement";
    pending_track_id_.reset();
    pending_switch_frames_ = 0;
  } else {
    if (pending_track_id_.has_value() && *pending_track_id_ == ranked->track->track_id) {
      ++pending_switch_frames_;
    } else {
      pending_track_id_ = ranked->track->track_id;
      pending_switch_frames_ = 1;
    }
    if (pending_switch_frames_ >= config_.switch_debounce_frames) {
      selected = ranked;
      diagnostics_.switched = true;
      ++diagnostics_.switch_count;
      diagnostics_.switch_reason = "debounce_confirmed_replacement";
      pending_track_id_.reset();
      pending_switch_frames_ = 0;
    } else {
      // Holding is safe only because previous was found in this frame's valid
      // Tracking candidates; a stale or TempLost target is never returned.
      selected = previous;
      ++diagnostics_.debounce_hold_count;
      diagnostics_.switch_reason = "debounce_hold";
    }
  }

  previous_track_id_ = selected->track->track_id;
  diagnostics_.selected_track_id = selected->track->track_id;
  return *selected->track;
}

void TargetSelector::reset()
{
  previous_track_id_.reset();
  pending_track_id_.reset();
  pending_switch_frames_ = 0;
  diagnostics_ = {};
  diagnostics_.switch_reason = "reset";
}

std::optional<std::uint64_t> TargetSelector::previous_track_id() const noexcept
{
  return previous_track_id_;
}

const TargetSelectorConfig & TargetSelector::config() const noexcept
{
  return config_;
}

const TargetSelectorDiagnostics & TargetSelector::diagnostics() const noexcept
{
  return diagnostics_;
}

const char * aimer_mode_name(AimerMode mode) noexcept
{
  switch (mode) {
    case AimerMode::RelativeDebug:
      return "relative_debug";
    case AimerMode::TestAbsoluteZero:
      return "test_absolute_zero";
  }
  return "unknown";
}

std::optional<std::string> AimerConfig::validate() const
{
  if (!test_only) {
    return "offline aimer requires test_only=true";
  }
  if (!finite(yaw_zero_rad) || !finite(pitch_zero_rad)) {
    return "test zero angles must be finite";
  }
  return std::nullopt;
}

pipeline::AimCommand AimerOutput::safe_command() const noexcept
{
  pipeline::AimCommand command{};
  command.target_lock = absolute_command_valid ? target_lock : pipeline::kTargetUnlocked;
  command.fire_command = pipeline::kFireNone;
  if (absolute_command_valid && command_yaw_rad.has_value() && command_pitch_rad.has_value()) {
    command.yaw_rad = static_cast<float>(*command_yaw_rad);
    command.pitch_rad = static_cast<float>(*command_pitch_rad);
  }
  return command;
}

SafeOfflineAimer::SafeOfflineAimer(AimerConfig config) : config_(std::move(config))
{
  if (const auto error = config_.validate(); error.has_value()) {
    throw std::invalid_argument("invalid aimer configuration: " + *error);
  }
}

AimerOutput SafeOfflineAimer::aim(
  const std::optional<TrackedTarget> & selected,
  double shoot_speed_mps) const noexcept
{
  AimerOutput result{};
  result.mode = config_.mode;
  result.test_only = config_.test_only;
  result.fire_command = pipeline::kFireNone;
  result.shoot_speed_mps = finite(shoot_speed_mps) && shoot_speed_mps >= 0.0 ? shoot_speed_mps : 0.0;
  if (!selected.has_value() || !selected->target_lock()) {
    return result;
  }

  // Tracker-generated motion diagnostics are normally finite, but keep the
  // aimer boundary fail-closed for synthetic or future callers that construct
  // a TrackedTarget directly.  Do not expose NaN/Inf in diagnostic CSV or let
  // such evidence appear lockable.
  if (!finite(selected->yaw_vel_rad_s) || !finite(selected->pitch_vel_rad_s)) {
    return result;
  }

  result.target_lock = pipeline::kTargetLocked;
  const auto & observation = selected->observation;
  result.relative_yaw_rad = observation.relative_yaw_rad;
  result.relative_pitch_rad = observation.relative_pitch_rad;
  result.yaw_vel_rad_s = selected->yaw_vel_rad_s;
  result.pitch_vel_rad_s = selected->pitch_vel_rad_s;
  if (!observation.relative_yaw_rad.has_value() || !observation.relative_pitch_rad.has_value()) {
    return result;
  }

  if (config_.mode != AimerMode::TestAbsoluteZero || !config_.absolute_zero_configured) {
    return result;
  }
  const double yaw = config_.yaw_zero_rad + *observation.relative_yaw_rad;
  const double pitch = config_.pitch_zero_rad + *observation.relative_pitch_rad;
  if (!finite(yaw) || !finite(pitch)) {
    return result;
  }
  result.absolute_command_valid = true;
  result.command_yaw_rad = yaw;
  result.command_pitch_rad = pitch;
  result.command_yaw_degree = units::radians_to_degrees(static_cast<float>(yaw));
  result.command_pitch_degree = units::radians_to_degrees(static_cast<float>(pitch));
  return result;
}

const AimerConfig & SafeOfflineAimer::config() const noexcept
{
  return config_;
}

const char * prediction_failure_reason_name(PredictionFailureReason reason) noexcept
{
  switch (reason) {
    case PredictionFailureReason::None:
      return "none";
    case PredictionFailureReason::Disabled:
      return "disabled";
    case PredictionFailureReason::NoTarget:
      return "no_target";
    case PredictionFailureReason::InvalidTrack:
      return "invalid_track";
    case PredictionFailureReason::NotTracking:
      return "not_tracking";
    case PredictionFailureReason::InvalidObservation:
      return "invalid_observation";
    case PredictionFailureReason::NegativeTimestamp:
      return "negative_timestamp";
    case PredictionFailureReason::TimestampMismatch:
      return "timestamp_mismatch";
    case PredictionFailureReason::NonMonotonicTimestamp:
      return "non_monotonic_timestamp";
    case PredictionFailureReason::NegativeHorizon:
      return "negative_horizon";
    case PredictionFailureReason::HorizonExceedsMaximum:
      return "horizon_exceeds_maximum";
    case PredictionFailureReason::MissingRelativeAngle:
      return "missing_relative_angle";
    case PredictionFailureReason::NonFiniteAngle:
      return "non_finite_angle";
    case PredictionFailureReason::NonFiniteVelocity:
      return "non_finite_velocity";
    case PredictionFailureReason::TimestampOverflow:
      return "timestamp_overflow";
    case PredictionFailureReason::NonFiniteResult:
      return "non_finite_result";
  }
  return "unknown";
}

std::optional<std::string> PredictionConfig::validate() const
{
  // A negative horizon is intentionally allowed through construction so the
  // predictor can report the explicit NegativeHorizon failure reason.  The
  // maximum, however, is a configuration invariant.
  if (max_horizon_ns < 0) {
    return "max_horizon_ns must not be negative";
  }
  return std::nullopt;
}

OfflinePredictor::OfflinePredictor(PredictionConfig config) : config_(std::move(config))
{
  if (const auto error = config_.validate(); error.has_value()) {
    throw std::invalid_argument("invalid prediction configuration: " + *error);
  }
}

PredictionResult OfflinePredictor::predict(
  const std::optional<TrackedTarget> & selected,
  std::int64_t frame_stamp_ns)
{
  PredictionResult result{};
  result.horizon_ns = config_.horizon_ns;
  result.test_only = true;
  result.production_ready = false;
  if (config_.horizon_ns >= 0) {
    result.horizon_s = static_cast<double>(config_.horizon_ns) / 1'000'000'000.0;
  } else {
    result.horizon_s = static_cast<double>(config_.horizon_ns) / 1'000'000'000.0;
  }

  const auto fail = [&result](PredictionFailureReason reason, const char * message) {
      result.valid = false;
      result.failure_reason = reason;
      result.reason = message;
      result.predicted_relative_yaw_rad.reset();
      result.predicted_relative_pitch_rad.reset();
    };

  // Disabled is a deliberate, safe default.  In particular, a caller that
  // never supplied --prediction-horizon-ms must not accidentally get a
  // zero-horizon record merely because this object exists.
  if (!config_.enabled) {
    fail(PredictionFailureReason::Disabled, "prediction is disabled");
    return result;
  }

  if (frame_stamp_ns < 0) {
    fail(PredictionFailureReason::NegativeTimestamp, "frame timestamp must not be negative");
    return result;
  }
  if (last_source_stamp_ns_.has_value() && frame_stamp_ns <= *last_source_stamp_ns_) {
    fail(PredictionFailureReason::NonMonotonicTimestamp,
      "frame timestamp must be strictly increasing");
    return result;
  }
  // A frame with no selected target still advances the replay clock.  This
  // prevents a later call from silently reusing the same frame timestamp.
  last_source_stamp_ns_ = frame_stamp_ns;
  result.source_stamp_ns = frame_stamp_ns;

  if (!selected.has_value()) {
    fail(PredictionFailureReason::NoTarget, "no selected target");
    return result;
  }
  result.track_id = selected->track_id;
  if (selected->track_id == 0) {
    fail(PredictionFailureReason::InvalidTrack, "track id must be non-zero");
    return result;
  }

  if (selected->state != TrackingState::Tracking) {
    fail(PredictionFailureReason::NotTracking,
      "prediction requires a Tracking target");
    return result;
  }

  const auto & observation = selected->observation;
  if (observation.stamp_ns < 0) {
    fail(PredictionFailureReason::NegativeTimestamp,
      "observation timestamp must not be negative");
    return result;
  }
  if (observation.stamp_ns != frame_stamp_ns ||
    (selected->last_valid_timestamp_ns >= 0 &&
    selected->last_valid_timestamp_ns != observation.stamp_ns))
  {
    fail(PredictionFailureReason::TimestampMismatch,
      "selected observation timestamp does not match frame/last-valid timestamp");
    return result;
  }
  if (!observation.relative_yaw_rad.has_value() ||
    !observation.relative_pitch_rad.has_value())
  {
    fail(PredictionFailureReason::MissingRelativeAngle,
      "relative yaw and pitch are both required");
    return result;
  }
  if (!finite(*observation.relative_yaw_rad) || !finite(*observation.relative_pitch_rad)) {
    fail(PredictionFailureReason::NonFiniteAngle,
      "relative yaw or pitch is not finite");
    return result;
  }
  if (!selected->valid()) {
    fail(PredictionFailureReason::InvalidObservation,
      "selected target observation failed fail-closed validation");
    return result;
  }
  if (config_.horizon_ns < 0) {
    fail(PredictionFailureReason::NegativeHorizon, "prediction horizon must not be negative");
    return result;
  }
  if (config_.horizon_ns > config_.max_horizon_ns) {
    fail(PredictionFailureReason::HorizonExceedsMaximum,
      "prediction horizon exceeds configured maximum");
    return result;
  }
  if (!finite(selected->yaw_vel_rad_s) || !finite(selected->pitch_vel_rad_s)) {
    fail(PredictionFailureReason::NonFiniteVelocity,
      "tracker angular velocity is not finite");
    return result;
  }
  if (frame_stamp_ns > std::numeric_limits<std::int64_t>::max() - config_.horizon_ns) {
    fail(PredictionFailureReason::TimestampOverflow,
      "predicted timestamp overflows int64 nanoseconds");
    return result;
  }

  const double yaw = *observation.relative_yaw_rad +
    selected->yaw_vel_rad_s * result.horizon_s;
  const double pitch = *observation.relative_pitch_rad +
    selected->pitch_vel_rad_s * result.horizon_s;
  if (!finite(yaw) || !finite(pitch)) {
    fail(PredictionFailureReason::NonFiniteResult,
      "constant-velocity prediction is not finite");
    return result;
  }

  result.predicted_stamp_ns = frame_stamp_ns + config_.horizon_ns;
  result.predicted_relative_yaw_rad = yaw;
  result.predicted_relative_pitch_rad = pitch;
  result.valid = true;
  result.failure_reason = PredictionFailureReason::None;
  result.reason = "constant_velocity_relative_angle_prediction";
  return result;
}

void OfflinePredictor::reset() noexcept
{
  last_source_stamp_ns_.reset();
}

const PredictionConfig & OfflinePredictor::config() const noexcept
{
  return config_;
}

SyntheticPredictionError diagnose_synthetic_prediction_error(
  const PredictionResult & prediction,
  const TargetObservation & measured_future) noexcept
{
  SyntheticPredictionError result{};
  result.synthetic = true;
  result.track_id = prediction.track_id;
  result.predicted_stamp_ns = prediction.predicted_stamp_ns;
  if (!prediction.valid || !prediction.predicted_relative_yaw_rad.has_value() ||
    !prediction.predicted_relative_pitch_rad.has_value())
  {
    result.reason = "prediction is not valid";
    return result;
  }
  if (measured_future.stamp_ns != prediction.predicted_stamp_ns) {
    result.reason = "measured future timestamp does not match prediction timestamp";
    return result;
  }
  if (!is_valid_target_observation(measured_future) ||
    !measured_future.relative_yaw_rad.has_value() ||
    !measured_future.relative_pitch_rad.has_value())
  {
    result.reason = "measured future observation is not valid relative-angle evidence";
    return result;
  }
  const double yaw_error = *measured_future.relative_yaw_rad -
    *prediction.predicted_relative_yaw_rad;
  const double pitch_error = *measured_future.relative_pitch_rad -
    *prediction.predicted_relative_pitch_rad;
  if (!finite(yaw_error) || !finite(pitch_error)) {
    result.reason = "synthetic prediction error is not finite";
    return result;
  }
  result.valid = true;
  result.yaw_error_rad = yaw_error;
  result.pitch_error_rad = pitch_error;
  result.reason = "synthetic_future_angle_error";
  return result;
}

cv::Mat annotate_offline_frame(
  const cv::Mat & bgr_image,
  const std::vector<pnp::PoseObservation> & observations,
  const TrackerUpdate & tracked,
  const std::optional<TrackedTarget> & selected,
  const AimerOutput & aimed,
  const PredictionResult * prediction)
{
  auto result = pnp::PnpStage::annotate(bgr_image, observations);
  if (result.empty()) {
    return result;
  }

  if (selected.has_value()) {
    const auto & bbox = selected->observation.raw_detection.bbox;
    cv::rectangle(result, bbox, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);
    const auto label = "selected track=" + std::to_string(selected->track_id);
    cv::putText(result, label, bbox.tl() + cv::Point2f(0.0F, -6.0F),
      cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 0, 255), 2, cv::LINE_AA);
  }

  const cv::Scalar status_color = aimed.target_lock == pipeline::kTargetLocked ?
    cv::Scalar(0, 220, 0) : cv::Scalar(0, 180, 255);
  std::string status = "state=" + std::string(tracking_state_name(tracked.state)) +
    " tracks=" + std::to_string(tracked.tracks.size()) +
    " lock=" + std::to_string(static_cast<int>(aimed.target_lock)) +
    " mode=" + aimer_mode_name(aimed.mode) + " fire=0 test_only=true";
  cv::putText(result, status, cv::Point(12, 28), cv::FONT_HERSHEY_SIMPLEX, 0.58,
    status_color, 2, cv::LINE_AA);
  if (prediction != nullptr) {
    std::string prediction_status = "prediction=" +
      std::string(prediction_failure_reason_name(prediction->failure_reason));
    if (prediction->valid && prediction->predicted_relative_yaw_rad.has_value() &&
      prediction->predicted_relative_pitch_rad.has_value())
    {
      prediction_status += " yaw=" + std::to_string(*prediction->predicted_relative_yaw_rad) +
        " pitch=" + std::to_string(*prediction->predicted_relative_pitch_rad);
    }
    cv::putText(result, prediction_status, cv::Point(12, 78),
      cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 180, 0), 2, cv::LINE_AA);
  }
  if (aimed.relative_yaw_rad.has_value() && aimed.relative_pitch_rad.has_value()) {
    const auto text = "rel_rad=" + std::to_string(*aimed.relative_yaw_rad) + "," +
      std::to_string(*aimed.relative_pitch_rad);
    cv::putText(result, text, cv::Point(12, 53), cv::FONT_HERSHEY_SIMPLEX, 0.52,
      status_color, 1, cv::LINE_AA);
  }
  if (aimed.absolute_command_valid && aimed.command_yaw_degree.has_value() &&
    aimed.command_pitch_degree.has_value())
  {
    const auto text = "test_abs_degree=" + std::to_string(*aimed.command_yaw_degree) + "," +
      std::to_string(*aimed.command_pitch_degree);
    cv::putText(result, text, cv::Point(12, 103), cv::FONT_HERSHEY_SIMPLEX, 0.52,
      cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
  }
  return result;
}

}  // namespace rm_auto_aim::offline
