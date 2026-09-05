#ifndef AUTO_AIM_ROS2__OFFLINE_PIPELINE_HPP_
#define AUTO_AIM_ROS2__OFFLINE_PIPELINE_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "auto_aim_ros2/angle_units.hpp"
#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/pnp_stage.hpp"

namespace rm_auto_aim::offline
{

struct BallisticResult;

enum class TrackingState : std::uint8_t
{
  Lost = 0,
  Detecting,
  Tracking,
  TempLost,
};

const char * tracking_state_name(TrackingState state) noexcept;

// The outcome of one frame's association work.  These values are diagnostic
// only; they never imply a control or fire decision.
enum class AssociationResult : std::uint8_t
{
  None = 0,
  NewTrack,
  Matched,
  Reacquired,
  RejectedInvalid,
  RejectedAssociationConflict,
  RejectedTimestamp,
  RejectedPositionJump,
  RejectedAngleJump,
  RejectedVelocity,
  Missed,
  Expired,
};

const char * association_result_name(AssociationResult result) noexcept;

// A target observation is measured evidence after PnP. It deliberately keeps
// camera-frame position as the association coordinate. A gimbal pose is
// optional and is never substituted for camera pose when comparing samples.
struct TargetObservation
{
  detector::RawArmorDetection raw_detection{};
  std::size_t detection_index{0};
  int class_id{-1};
  pnp::ArmorSize armor_size{pnp::ArmorSize::Small};
  float confidence{0.0F};
  std::optional<cv::Vec3d> camera_xyz_m;
  std::optional<cv::Vec3d> gimbal_xyz_m;
  std::optional<double> relative_yaw_rad;
  std::optional<double> relative_pitch_rad;
  double reprojection_error_px{0.0};
  std::int64_t stamp_ns{0};
  bool valid{false};
  bool geometry_known{false};
};

TargetObservation make_target_observation(
  const pnp::PoseObservation & pose,
  std::int64_t stamp_ns,
  std::size_t detection_index = 0) noexcept;

// Full offline-boundary validation shared by Tracker and TargetSelector.
// Unknown armor hints remain valid when PnP supplied the explicit armor_size;
// unsupported enum values and conflicting known hints fail closed.
bool is_valid_target_observation(const TargetObservation & observation) noexcept;

// Explainable association only: no EKF, ballistic prediction, yaw wrapping,
// or world-coordinate transform is performed here.
struct TrackerConfig
{
  int min_detect_count{2};
  int max_temp_lost_ms{100};
  double max_position_jump_m{0.75};
  double max_angle_jump_rad{0.75};
  // Zero disables this finite-difference diagnostic guard.
  double max_velocity_rad_s{0.0};

  std::optional<std::string> validate() const;
};

// Cumulative, reproducible tracker evidence.  Counts are deliberately kept
// separate: a structurally valid observation is not necessarily accepted by
// the association gates.
struct TrackerStatistics
{
  std::uint64_t frame_count{0};
  std::uint64_t observation_count{0};
  std::uint64_t valid_count{0};
  std::uint64_t accepted_count{0};
  std::uint64_t matched_count{0};
  std::uint64_t new_count{0};
  std::uint64_t reacquisition_count{0};
  std::uint64_t rejected_count{0};
  std::uint64_t invalid_count{0};
  std::uint64_t position_jump_count{0};
  std::uint64_t angle_jump_count{0};
  std::uint64_t velocity_count{0};
  std::uint64_t missed_count{0};
  std::uint64_t expired_count{0};
  std::uint64_t rejected_timestamp_count{0};
};

struct TrackedTarget
{
  std::uint64_t track_id{0};
  TrackingState state{TrackingState::Lost};
  TargetObservation observation{};
  int consecutive_valid{0};
  int consecutive_missed{0};
  std::int64_t first_valid_timestamp_ns{-1};
  std::int64_t last_valid_timestamp_ns{-1};
  double yaw_vel_rad_s{0.0};
  double pitch_vel_rad_s{0.0};
  bool updated_this_frame{false};
  AssociationResult association_result{AssociationResult::None};
  std::string association_reason;
  std::uint64_t accepted_count{0};
  std::uint64_t matched_count{0};
  std::uint64_t reacquisition_count{0};
  std::uint64_t rejected_count{0};
  std::uint64_t missed_count{0};
  std::uint64_t expired_count{0};

  bool valid() const noexcept
  {
    // Keep direct/synthetic TrackedTarget callers on the same fail-closed
    // boundary as OfflineTracker and TargetSelector.  A state value alone
    // must not turn malformed observation evidence into a lock.
    return is_valid_target_observation(observation);
  }

  bool target_lock() const noexcept
  {
    return state == TrackingState::Tracking && valid();
  }
};

struct TrackerUpdate
{
  bool accepted{false};
  bool rejected{false};
  bool had_observation{false};
  std::string rejection_reason;
  TrackingState state{TrackingState::Lost};
  // All tracks are returned for diagnostics. TargetSelector filters this list
  // to valid Tracking targets, so TempLost/Lost can never lock.
  std::vector<TrackedTarget> tracks;
  std::size_t valid_observation_count{0};
  std::size_t accepted_count{0};
  std::size_t accepted_observation_count{0};
  std::size_t rejected_observation_count{0};
  std::size_t matched_count{0};
  std::size_t new_count{0};
  std::size_t reacquired_count{0};
  std::size_t missed_count{0};
  std::size_t expired_count{0};
  AssociationResult association_result{AssociationResult::None};
  std::string association_reason;
  TrackerStatistics statistics{};
  std::int64_t timestamp_ns{0};
  bool lock_allowed{true};
  std::optional<TrackedTarget> primary_track;

  bool target_lock() const noexcept
  {
    return lock_allowed && primary_track.has_value() && primary_track->target_lock();
  }
};

class OfflineTracker final
{
public:
  explicit OfflineTracker(TrackerConfig config = {});

  TrackerUpdate update(
    const std::vector<TargetObservation> & observations,
    std::int64_t timestamp_ns);

  // Convenience overload for single-target tests and small callers.
  TrackerUpdate update(
    const std::optional<TargetObservation> & observation,
    std::int64_t timestamp_ns);

  void reset() noexcept;
  const TrackerConfig & config() const noexcept;
  TrackingState state() const noexcept;
  const std::vector<TrackedTarget> & tracks() const noexcept;
  const TrackerStatistics & statistics() const noexcept;

private:
  TrackerConfig config_;
  std::vector<TrackedTarget> tracks_;
  TrackerStatistics statistics_{};
  std::uint64_t next_track_id_{1};
  std::int64_t last_update_timestamp_ns_{-1};

  TrackerUpdate make_update(
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
    AssociationResult association_result) const;
};

struct TargetSelectorConfig
{
  float confidence_tie_epsilon{1e-6F};
  // One means switch immediately. Larger values require the same replacement
  // candidate to win this many consecutive selections while the old target
  // remains a current valid Tracking candidate.
  int switch_debounce_frames{1};

  std::optional<std::string> validate() const;
};

struct TargetSelectorDiagnostics
{
  std::uint64_t selection_count{0};
  std::uint64_t switch_count{0};
  std::uint64_t no_candidate_count{0};
  std::uint64_t debounce_hold_count{0};
  std::size_t candidate_count{0};
  bool switched{false};
  std::optional<std::uint64_t> candidate_track_id;
  std::optional<std::uint64_t> selected_track_id;
  std::string switch_reason{"none"};
};

// Stateful deterministic selector. Confidence is primary; previous track ID,
// image-center distance, and track ID are deterministic tie-breaks.
class TargetSelector final
{
public:
  explicit TargetSelector(TargetSelectorConfig config = {});

  std::optional<TrackedTarget> select(
    const std::vector<TrackedTarget> & tracks,
    int image_width,
    int image_height);

  void reset();
  std::optional<std::uint64_t> previous_track_id() const noexcept;
  const TargetSelectorConfig & config() const noexcept;
  const TargetSelectorDiagnostics & diagnostics() const noexcept;

private:
  TargetSelectorConfig config_;
  std::optional<std::uint64_t> previous_track_id_;
  std::optional<std::uint64_t> pending_track_id_;
  int pending_switch_frames_{0};
  TargetSelectorDiagnostics diagnostics_{};
};

enum class AimerMode : std::uint8_t
{
  RelativeDebug = 0,
  TestAbsoluteZero,
};

const char * aimer_mode_name(AimerMode mode) noexcept;

struct AimerConfig
{
  AimerMode mode{AimerMode::RelativeDebug};
  // Offline output is always diagnostic/test-only.
  bool test_only{true};
  bool absolute_zero_configured{false};
  double yaw_zero_rad{0.0};
  double pitch_zero_rad{0.0};

  std::optional<std::string> validate() const;
};

struct AimerOutput
{
  AimerMode mode{AimerMode::RelativeDebug};
  bool test_only{true};
  bool absolute_command_valid{false};
  std::int8_t target_lock{pipeline::kTargetUnlocked};
  std::int8_t fire_command{pipeline::kFireNone};
  double shoot_speed_mps{0.0};
  std::optional<double> relative_yaw_rad;
  std::optional<double> relative_pitch_rad;
  std::optional<double> command_yaw_rad;
  std::optional<double> command_pitch_rad;
  std::optional<double> command_yaw_degree;
  std::optional<double> command_pitch_degree;
  // Internal diagnostics only. ROS/serial adapters must not forward these
  // until external units are confirmed.
  double yaw_vel_rad_s{0.0};
  double pitch_vel_rad_s{0.0};
  double yaw_acc_rad_s2{0.0};
  double pitch_acc_rad_s2{0.0};

  // In RelativeDebug this stays unlocked with zero angles, preventing an
  // accidental caller from treating it as an absolute RobotCtrl command.
  pipeline::AimCommand safe_command() const noexcept;
};

// Failure reasons are part of the offline evidence contract.  They are
// deliberately descriptive rather than a generic "prediction failed" flag:
// a caller must be able to distinguish malformed input, an unsafe state, and
// an explicitly disallowed horizon without guessing from numeric output.
enum class PredictionFailureReason : std::uint8_t
{
  None = 0,
  Disabled,
  NoTarget,
  InvalidTrack,
  NotTracking,
  InvalidObservation,
  NegativeTimestamp,
  TimestampMismatch,
  NonMonotonicTimestamp,
  NegativeHorizon,
  HorizonExceedsMaximum,
  MissingRelativeAngle,
  NonFiniteAngle,
  NonFiniteVelocity,
  TimestampOverflow,
  NonFiniteResult,
};

const char * prediction_failure_reason_name(PredictionFailureReason reason) noexcept;

// The predictor is intentionally configured in integer nanoseconds.  This
// keeps replay timestamps exact and avoids a second floating-point time base.
// horizon_ns may be zero when an explicitly enabled zero-horizon diagnostic is
// desired.  The default-disabled configuration must not emit predictions.
struct PredictionConfig
{
  bool enabled{false};
  std::int64_t horizon_ns{0};
  std::int64_t max_horizon_ns{500'000'000};

  std::optional<std::string> validate() const;
};

struct PredictionResult
{
  bool valid{false};
  std::uint64_t track_id{0};
  std::int64_t source_stamp_ns{-1};
  std::int64_t horizon_ns{0};
  double horizon_s{0.0};
  std::int64_t predicted_stamp_ns{-1};
  std::optional<double> predicted_relative_yaw_rad;
  std::optional<double> predicted_relative_pitch_rad;
  PredictionFailureReason failure_reason{PredictionFailureReason::Disabled};
  std::string reason;
  // These flags are immutable safety evidence; no caller may promote this
  // result to a production control command.
  bool test_only{true};
  bool production_ready{false};
};

// Constant-velocity, relative-angle-only baseline for deterministic replay.
// It owns no wall clock and has no connection to Selector, Aimer, RobotCtrl,
// quaternion, world coordinates, or ballistic/fire logic.
class OfflinePredictor final
{
public:
  explicit OfflinePredictor(PredictionConfig config = {});

  PredictionResult predict(
    const std::optional<TrackedTarget> & selected,
    std::int64_t frame_stamp_ns);

  PredictionResult predict(
    const TrackedTarget & selected,
    std::int64_t frame_stamp_ns)
  {
    return predict(std::optional<TrackedTarget>(selected), frame_stamp_ns);
  }

  void reset() noexcept;
  const PredictionConfig & config() const noexcept;

private:
  PredictionConfig config_;
  std::optional<std::int64_t> last_source_stamp_ns_;
};

// Optional synthetic replay helper.  It compares a valid prediction with a
// later measured relative-angle observation at exactly the predicted stamp.
// It is explicitly diagnostic/test-only and cannot imply real hit rate.
struct SyntheticPredictionError
{
  bool valid{false};
  bool synthetic{true};
  std::uint64_t track_id{0};
  std::int64_t predicted_stamp_ns{-1};
  std::optional<double> yaw_error_rad;
  std::optional<double> pitch_error_rad;
  std::string reason;
};

SyntheticPredictionError diagnose_synthetic_prediction_error(
  const PredictionResult & prediction,
  const TargetObservation & measured_future) noexcept;

class SafeOfflineAimer final
{
public:
  explicit SafeOfflineAimer(AimerConfig config = {});

  AimerOutput aim(
    const std::optional<TrackedTarget> & selected,
    double shoot_speed_mps = 0.0) const noexcept;
  const AimerConfig & config() const noexcept;

private:
  AimerConfig config_;
};

cv::Mat annotate_offline_frame(
  const cv::Mat & bgr_image,
  const std::vector<pnp::PoseObservation> & observations,
  const TrackerUpdate & tracked,
  const std::optional<TrackedTarget> & selected,
  const AimerOutput & aimed,
  const PredictionResult * prediction = nullptr,
  const BallisticResult * ballistic = nullptr);

}  // namespace rm_auto_aim::offline

#endif  // AUTO_AIM_ROS2__OFFLINE_PIPELINE_HPP_
