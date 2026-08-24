#include "auto_aim_ros2/offline_pipeline.hpp"

#include <algorithm>
#include <cmath>
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

bool valid_observation(const TargetObservation & observation) noexcept
{
  if (!observation.valid || !observation.geometry_known || observation.class_id < 0 ||
    !finite(observation.confidence) || observation.confidence < 0.0F ||
    observation.confidence > 1.0F || !finite(observation.reprojection_error_px) ||
    observation.reprojection_error_px < 0.0 || camera_position_for(observation) == nullptr)
  {
    return false;
  }
  if (observation.raw_detection.class_id != observation.class_id ||
    !finite(observation.raw_detection.confidence) ||
    observation.raw_detection.confidence < 0.0F || observation.raw_detection.confidence > 1.0F ||
    !finite(observation.raw_detection.bbox.x) || !finite(observation.raw_detection.bbox.y) ||
    !finite(observation.raw_detection.bbox.width) || !finite(observation.raw_detection.bbox.height) ||
    observation.raw_detection.bbox.width <= 0.0F || observation.raw_detection.bbox.height <= 0.0F)
  {
    return false;
  }
  for (const auto & keypoint : observation.raw_detection.keypoints) {
    if (!finite_point(keypoint)) {
      return false;
    }
  }

  const auto & camera_position = *observation.camera_xyz_m;
  if (!finite_vec(camera_position) || camera_position[2] <= kPositiveDepthEpsilon) {
    return false;
  }
  if (observation.gimbal_xyz_m.has_value() && !finite_vec(*observation.gimbal_xyz_m)) {
    return false;
  }
  if (observation.relative_yaw_rad.has_value() && !finite(*observation.relative_yaw_rad)) {
    return false;
  }
  if (observation.relative_pitch_rad.has_value() && !finite(*observation.relative_pitch_rad)) {
    return false;
  }
  return true;
}

bool same_target_identity(
  const TargetObservation & first,
  const TargetObservation & second) noexcept
{
  return first.class_id == second.class_id && first.armor_size == second.armor_size;
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
  std::size_t rejected_count) const
{
  TrackerUpdate result{};
  result.accepted = accepted;
  result.rejected = rejected;
  result.had_observation = had_observation;
  result.rejection_reason = std::move(reason);
  result.timestamp_ns = timestamp_ns;
  result.valid_observation_count = valid_count;
  result.rejected_observation_count = rejected_count;
  result.tracks = tracks_;

  for (const auto & track : result.tracks) {
    if (!result.primary_track.has_value() || better_primary_track(track, *result.primary_track)) {
      result.primary_track = track;
    }
  }
  if (result.primary_track.has_value()) {
    result.state = result.primary_track->state;
  } else {
    result.state = TrackingState::Lost;
  }
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
  if (timestamp_ns < 0) {
    auto result = make_update(timestamp_ns, false, true, !observations.empty(),
      "timestamp must not be negative", 0, observations.size());
    result.lock_allowed = false;
    for (auto & track : result.tracks) {
      if (track.state == TrackingState::Tracking) {
        track.state = TrackingState::TempLost;
      }
    }
    return result;
  }
  if (last_update_timestamp_ns_ >= 0 && timestamp_ns <= last_update_timestamp_ns_) {
    auto result = make_update(timestamp_ns, false, true, !observations.empty(),
      "timestamp is not strictly increasing", 0, observations.size());
    result.lock_allowed = false;
    for (auto & track : result.tracks) {
      if (track.state == TrackingState::Tracking) {
        track.state = TrackingState::TempLost;
      }
    }
    return result;
  }

  last_update_timestamp_ns_ = timestamp_ns;

  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(observations.size());
  std::size_t rejected_count = 0;
  std::string first_rejection_reason;
  for (std::size_t index = 0; index < observations.size(); ++index) {
    const auto & observation = observations[index];
    if (observation.stamp_ns != timestamp_ns) {
      ++rejected_count;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "observation timestamp does not match frame timestamp";
      }
    } else if (!valid_observation(observation)) {
      ++rejected_count;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "observation is invalid or incomplete";
      }
    } else {
      valid_indices.push_back(index);
    }
  }

  // Sort by physical evidence, not input order. This makes creation and
  // association deterministic when detector output order changes.
  std::sort(valid_indices.begin(), valid_indices.end(), [&observations](std::size_t first, std::size_t second) {
    const auto & lhs = observations[first];
    const auto & rhs = observations[second];
    if (lhs.class_id != rhs.class_id) {
      return lhs.class_id < rhs.class_id;
    }
    if (lhs.armor_size != rhs.armor_size) {
      return static_cast<int>(lhs.armor_size) < static_cast<int>(rhs.armor_size);
    }
    const auto & lp = *lhs.camera_xyz_m;
    const auto & rp = *rhs.camera_xyz_m;
    if (lp[0] != rp[0]) {
      return lp[0] < rp[0];
    }
    if (lp[1] != rp[1]) {
      return lp[1] < rp[1];
    }
    if (lp[2] != rp[2]) {
      return lp[2] < rp[2];
    }
    if (lhs.confidence != rhs.confidence) {
      return lhs.confidence > rhs.confidence;
    }
    return lhs.detection_index < rhs.detection_index;
  });

  std::vector<bool> track_matched(tracks_.size(), false);
  std::vector<bool> observation_consumed(observations.size(), false);
  std::size_t accepted_count = 0;

  for (const auto observation_index : valid_indices) {
    const auto & observation = observations[observation_index];
    std::optional<std::size_t> best_track_index;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
      const auto & track = tracks_[track_index];
      if (track_matched[track_index] || track.state == TrackingState::Lost ||
        track.last_valid_timestamp_ns == timestamp_ns || !track.valid() ||
        !same_target_identity(track.observation, observation))
      {
        continue;
      }
      const auto * previous_position = camera_position_for(track.observation);
      const auto * current_position = camera_position_for(observation);
      if (previous_position == nullptr || current_position == nullptr) {
        continue;
      }
      const double distance = point_distance(*previous_position, *current_position);
      if (distance > config_.max_position_jump_m) {
        continue;
      }
      if (!best_track_index.has_value() || distance < best_distance - kFiniteEpsilon ||
        (std::abs(distance - best_distance) <= kFiniteEpsilon &&
        track.track_id < tracks_[*best_track_index].track_id))
      {
        best_track_index = track_index;
        best_distance = distance;
      }
    }

    if (!best_track_index.has_value()) {
      // If an active track of the same identity exists but is outside the
      // position gate, this is a jump rejection, not a new target. This
      // prevents a single outlier from creating a second lockable track.
      bool same_identity_active = false;
      for (const auto & track : tracks_) {
        if (track.last_valid_timestamp_ns < timestamp_ns &&
          track.state != TrackingState::Lost && track.valid() &&
          same_target_identity(track.observation, observation))
        {
          same_identity_active = true;
          break;
        }
      }
      if (same_identity_active) {
        ++rejected_count;
        if (first_rejection_reason.empty()) {
          first_rejection_reason = "position jump exceeds tracker limit";
        }
        observation_consumed[observation_index] = true;
        continue;
      }
      TrackedTarget track{};
      track.track_id = next_track_id_++;
      track.state = config_.min_detect_count <= 1 ? TrackingState::Tracking :
        TrackingState::Detecting;
      track.observation = observation;
      track.consecutive_valid = 1;
      track.first_valid_timestamp_ns = timestamp_ns;
      track.last_valid_timestamp_ns = timestamp_ns;
      tracks_.push_back(track);
      track_matched.push_back(true);
      observation_consumed[observation_index] = true;
      ++accepted_count;
      continue;
    }

    const auto track_index = *best_track_index;
    auto & track = tracks_[track_index];
    const auto elapsed_ns = timestamp_ns - track.last_valid_timestamp_ns;
    if (elapsed_ns <= 0) {
      ++rejected_count;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "track timestamp is not increasing";
      }
      track_matched[track_index] = true;
      observation_consumed[observation_index] = true;
      continue;
    }

    bool angle_jump = false;
    if (track.observation.relative_yaw_rad.has_value() && observation.relative_yaw_rad.has_value() &&
      std::abs(*observation.relative_yaw_rad - *track.observation.relative_yaw_rad) >
      config_.max_angle_jump_rad)
    {
      angle_jump = true;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "yaw jump exceeds tracker limit";
      }
    }
    if (track.observation.relative_pitch_rad.has_value() && observation.relative_pitch_rad.has_value() &&
      std::abs(*observation.relative_pitch_rad - *track.observation.relative_pitch_rad) >
      config_.max_angle_jump_rad)
    {
      angle_jump = true;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "pitch jump exceeds tracker limit";
      }
    }
    const double dt_s = static_cast<double>(elapsed_ns) / 1'000'000'000.0;
    double yaw_velocity = 0.0;
    double pitch_velocity = 0.0;
    if (track.observation.relative_yaw_rad.has_value() && observation.relative_yaw_rad.has_value()) {
      yaw_velocity = (*observation.relative_yaw_rad - *track.observation.relative_yaw_rad) / dt_s;
    }
    if (track.observation.relative_pitch_rad.has_value() && observation.relative_pitch_rad.has_value()) {
      pitch_velocity = (*observation.relative_pitch_rad - *track.observation.relative_pitch_rad) / dt_s;
    }
    if (!finite(yaw_velocity) || !finite(pitch_velocity)) {
      angle_jump = true;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "computed motion is not finite";
      }
    }
    if (config_.max_velocity_rad_s > 0.0 &&
      (std::abs(yaw_velocity) > config_.max_velocity_rad_s ||
      std::abs(pitch_velocity) > config_.max_velocity_rad_s))
    {
      angle_jump = true;
      if (first_rejection_reason.empty()) {
        first_rejection_reason = "computed motion exceeds tracker limit";
      }
    }

    track_matched[track_index] = true;
    observation_consumed[observation_index] = true;
    if (angle_jump) {
      ++rejected_count;
      continue;
    }

    const bool recovering = track.state == TrackingState::TempLost;
    track.observation = observation;
    track.consecutive_missed = 0;
    track.consecutive_valid = recovering ? 1 : track.consecutive_valid + 1;
    track.state = track.consecutive_valid >= config_.min_detect_count ?
      TrackingState::Tracking : TrackingState::Detecting;
    track.last_valid_timestamp_ns = timestamp_ns;
    track.yaw_vel_rad_s = yaw_velocity;
    track.pitch_vel_rad_s = pitch_velocity;
    ++accepted_count;
  }

  // A matched flag only means an observation was considered for that track;
  // angle-jump rejection must still count as a miss for state transitions.
  for (std::size_t track_index = 0; track_index < tracks_.size(); ++track_index) {
    auto & track = tracks_[track_index];
    const bool actually_updated = std::any_of(
      valid_indices.begin(), valid_indices.end(), [&](std::size_t observation_index) {
        return observation_consumed[observation_index] &&
          same_target_identity(track.observation, observations[observation_index]) &&
          track.last_valid_timestamp_ns == timestamp_ns &&
          track.observation.detection_index == observations[observation_index].detection_index;
      });
    if (actually_updated) {
      continue;
    }
    if (track.last_valid_timestamp_ns < 0) {
      continue;
    }
    ++track.consecutive_missed;
    const auto elapsed_ns = timestamp_ns - track.last_valid_timestamp_ns;
    if (elapsed_ns < static_cast<std::int64_t>(config_.max_temp_lost_ms) *
      kNanosecondsPerMillisecond)
    {
      track.state = TrackingState::TempLost;
    } else {
      track.state = TrackingState::Lost;
      track.consecutive_valid = 0;
    }
    track.yaw_vel_rad_s = 0.0;
    track.pitch_vel_rad_s = 0.0;
  }

  const bool had_observation = !observations.empty();
  const bool rejected = rejected_count > 0;
  const bool accepted = accepted_count > 0;
  return make_update(
    timestamp_ns, accepted, rejected, had_observation,
    first_rejection_reason, accepted_count, rejected_count);
}

void OfflineTracker::reset() noexcept
{
  tracks_.clear();
  next_track_id_ = 1;
  last_update_timestamp_ns_ = -1;
}

const TrackerConfig & OfflineTracker::config() const noexcept
{
  return config_;
}

TrackingState OfflineTracker::state() const noexcept
{
  const auto update = make_update(0, false, false, false, {}, 0, 0);
  return update.state;
}

const std::vector<TrackedTarget> & OfflineTracker::tracks() const noexcept
{
  return tracks_;
}

std::optional<std::string> TargetSelectorConfig::validate() const
{
  if (!finite(confidence_tie_epsilon) || confidence_tie_epsilon < 0.0F) {
    return "confidence_tie_epsilon must be finite and non-negative";
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
  if (image_width <= 0 || image_height <= 0) {
    previous_track_id_.reset();
    return std::nullopt;
  }

  const TrackedTarget * best = nullptr;
  std::optional<double> best_center_distance;
  for (const auto & candidate : tracks) {
    if (!candidate.target_lock() || !valid_observation(candidate.observation)) {
      continue;
    }
    const auto candidate_center_distance = center_distance(
      candidate.observation, image_width, image_height);
    if (!candidate_center_distance.has_value()) {
      continue;
    }
    if (best == nullptr) {
      best = &candidate;
      best_center_distance = candidate_center_distance;
      continue;
    }

    const bool higher_confidence = candidate.observation.confidence >
      best->observation.confidence + config_.confidence_tie_epsilon;
    const bool tied_confidence = std::abs(
      candidate.observation.confidence - best->observation.confidence) <=
      config_.confidence_tie_epsilon;
    if (higher_confidence) {
      best = &candidate;
      best_center_distance = candidate_center_distance;
      continue;
    }
    if (!tied_confidence) {
      continue;
    }

    const bool candidate_is_previous = previous_track_id_.has_value() &&
      candidate.track_id == *previous_track_id_;
    const bool best_is_previous = previous_track_id_.has_value() &&
      best->track_id == *previous_track_id_;
    if (candidate_is_previous != best_is_previous) {
      if (candidate_is_previous) {
        best = &candidate;
        best_center_distance = candidate_center_distance;
      }
      continue;
    }
    if (*candidate_center_distance < *best_center_distance - kFiniteEpsilon ||
      (std::abs(*candidate_center_distance - *best_center_distance) <= kFiniteEpsilon &&
      candidate.track_id < best->track_id))
    {
      best = &candidate;
      best_center_distance = candidate_center_distance;
    }
  }

  if (best == nullptr) {
    previous_track_id_.reset();
    return std::nullopt;
  }
  previous_track_id_ = best->track_id;
  return *best;
}

void TargetSelector::reset() noexcept
{
  previous_track_id_.reset();
}

std::optional<std::uint64_t> TargetSelector::previous_track_id() const noexcept
{
  return previous_track_id_;
}

const TargetSelectorConfig & TargetSelector::config() const noexcept
{
  return config_;
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

cv::Mat annotate_offline_frame(
  const cv::Mat & bgr_image,
  const std::vector<pnp::PoseObservation> & observations,
  const TrackerUpdate & tracked,
  const std::optional<TrackedTarget> & selected,
  const AimerOutput & aimed)
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
    cv::putText(result, text, cv::Point(12, 78), cv::FONT_HERSHEY_SIMPLEX, 0.52,
      cv::Scalar(255, 200, 0), 1, cv::LINE_AA);
  }
  return result;
}

}  // namespace rm_auto_aim::offline
