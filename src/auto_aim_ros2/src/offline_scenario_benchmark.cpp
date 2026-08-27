#include "auto_aim_ros2/offline_scenario_benchmark.hpp"

#include "auto_aim_ros2/offline_ballistic.hpp"
#include "auto_aim_ros2/offline_pipeline.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <map>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace rm_auto_aim::offline
{
namespace
{

constexpr std::int64_t kFramePeriodNs = 10'000'000;
constexpr int kImageWidth = 1280;
constexpr int kImageHeight = 800;

enum class ObservationFault : std::uint8_t
{
  None = 0,
  NaNCameraPosition,
  InfiniteRelativeYaw,
  InvalidFlag,
};

enum class BallisticFixture : std::uint8_t
{
  Nominal = 0,
  MissingMuzzleTransform,
  Unreachable,
  MissingBulletSpeed,
  InvalidBulletSpeed,
  HorizonExceedsMaximum,
};

struct SyntheticTarget
{
  std::string id;
  cv::Vec3d muzzle_m{};
  double relative_yaw_rad{0.0};
  double relative_pitch_rad{0.0};
  float confidence{0.9F};
  int class_id{1};
  ObservationFault fault{ObservationFault::None};
};

struct SyntheticFrame
{
  std::int64_t stamp_ns{0};
  std::string event;
  std::vector<SyntheticTarget> targets;
  bool permute_input{false};
  BallisticFixture ballistic_fixture{BallisticFixture::Nominal};
};

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

bool finite_vector(const cv::Vec3d & value) noexcept
{
  return finite(value[0]) && finite(value[1]) && finite(value[2]);
}

std::string bool_text(bool value)
{
  return value ? "true" : "false";
}

std::string stable_double(double value)
{
  if (!finite(value)) {
    return {};
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::fixed << std::setprecision(12) << value;
  return stream.str();
}

template<typename T>
std::string optional_integer_text(const std::optional<T> & value)
{
  return value.has_value() ? std::to_string(*value) : std::string{};
}

std::string optional_double_text(const std::optional<double> & value)
{
  return value.has_value() ? stable_double(*value) : std::string{};
}

std::string csv_escape(const std::string & value)
{
  if (value.find_first_of(",\"\r\n") == std::string::npos) {
    return value;
  }
  std::string escaped;
  escaped.reserve(value.size() + 2);
  escaped.push_back('"');
  for (const char character : value) {
    if (character == '"') {
      escaped.push_back('"');
    }
    escaped.push_back(character);
  }
  escaped.push_back('"');
  return escaped;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  for (const unsigned char character : value) {
    switch (character) {
      case '\\':
        stream << "\\\\";
        break;
      case '"':
        stream << "\\\"";
        break;
      case '\b':
        stream << "\\b";
        break;
      case '\f':
        stream << "\\f";
        break;
      case '\n':
        stream << "\\n";
        break;
      case '\r':
        stream << "\\r";
        break;
      case '\t':
        stream << "\\t";
        break;
      default:
        if (character < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<unsigned int>(character) << std::dec << std::setfill(' ');
        } else {
          stream << static_cast<char>(character);
        }
        break;
    }
  }
  return stream.str();
}

SyntheticTarget target(
  std::string id,
  const cv::Vec3d & muzzle_m,
  double yaw,
  double pitch,
  float confidence = 0.9F,
  int class_id = 1,
  ObservationFault fault = ObservationFault::None)
{
  SyntheticTarget result{};
  result.id = std::move(id);
  result.muzzle_m = muzzle_m;
  result.relative_yaw_rad = yaw;
  result.relative_pitch_rad = pitch;
  result.confidence = confidence;
  result.class_id = class_id;
  result.fault = fault;
  return result;
}

SyntheticFrame frame(
  std::int64_t stamp_ns,
  std::string event,
  std::vector<SyntheticTarget> targets = {},
  bool permute_input = false,
  BallisticFixture ballistic_fixture = BallisticFixture::Nominal)
{
  SyntheticFrame result{};
  result.stamp_ns = stamp_ns;
  result.event = std::move(event);
  result.targets = std::move(targets);
  result.permute_input = permute_input;
  result.ballistic_fixture = ballistic_fixture;
  return result;
}

std::vector<SyntheticFrame> static_3m_frames()
{
  std::vector<SyntheticFrame> result;
  for (int index = 0; index < 5; ++index) {
    result.push_back(frame(
      static_cast<std::int64_t>(index) * kFramePeriodNs,
      index == 0 ? "initial_detection" : "stationary_3m_observation",
      {target("static", {3.0, 0.0, 0.0}, 0.0, 0.0)}));
  }
  return result;
}

std::vector<SyntheticFrame> spin_3m_frames()
{
  std::vector<SyntheticFrame> result;
  for (int index = 0; index < 7; ++index) {
    // The centre remains at 3 m.  The explicitly supplied relative-angle
    // waveform is synthetic observable evidence for in-place rotation; it
    // does not introduce vehicle geometry, a model, or physical parameters.
    result.push_back(frame(
      static_cast<std::int64_t>(index) * kFramePeriodNs,
      index == 0 ? "initial_detection" : "synthetic_in_place_rotation",
      {target("spin", {3.0, 0.0, 0.0}, -0.12 + 0.025 * index, 0.01 * index)}));
  }
  return result;
}

std::vector<SyntheticFrame> spin_translate_3m_frames()
{
  std::vector<SyntheticFrame> result;
  for (int index = 0; index < 7; ++index) {
    const double step = static_cast<double>(index);
    result.push_back(frame(
      static_cast<std::int64_t>(index) * kFramePeriodNs,
      index == 0 ? "initial_detection" : "synthetic_rotation_and_translation",
      {target(
        "spin_translate",
        {3.0 + 0.025 * step, -0.12 + 0.035 * step, 0.02 + 0.004 * step},
        -0.10 + 0.030 * step,
        -0.02 + 0.006 * step)}));
  }
  return result;
}

std::vector<SyntheticFrame> crossing_permuted_frames()
{
  std::vector<SyntheticFrame> result;
  constexpr std::array<double, 5> kLateral{-0.30, -0.15, 0.0, 0.15, 0.30};
  for (std::size_t index = 0; index < kLateral.size(); ++index) {
    const double left_to_right = kLateral[index];
    const double right_to_left = -left_to_right;
    const auto stamp = static_cast<std::int64_t>(index) * kFramePeriodNs;
    const float first_confidence = index < 3U ? 0.95F : 0.80F;
    const float second_confidence = index < 3U ? 0.80F : 0.95F;
    result.push_back(frame(
      stamp,
      index == 0 ? "crossing_initial_detection" : "crossing_input_order_permuted",
      {
        target(
          "cross_a", {3.0, left_to_right, 0.0},
          std::atan2(left_to_right, 3.0), 0.0, first_confidence, 1),
        target(
          "cross_b", {3.0, right_to_left, 0.0},
          std::atan2(right_to_left, 3.0), 0.0, second_confidence, 2),
      },
      true));
  }
  return result;
}

std::vector<SyntheticFrame> occlusion_reacquisition_frames()
{
  const auto subject = [](std::int64_t stamp) {
      return target("occlusion", {3.0, 0.08, 0.0}, 0.026660565, 0.0);
    };
  return {
    frame(0, "initial_detection", {subject(0)}),
    frame(kFramePeriodNs, "tracking_before_short_occlusion", {subject(kFramePeriodNs)}),
    frame(2 * kFramePeriodNs, "short_occlusion_temp_lost"),
    frame(3 * kFramePeriodNs, "short_occlusion_reacquired", {subject(3 * kFramePeriodNs)}),
    frame(4 * kFramePeriodNs, "tracking_after_reacquisition", {subject(4 * kFramePeriodNs)}),
    frame(5 * kFramePeriodNs, "begin_long_occlusion"),
    frame(16 * kFramePeriodNs, "temporary_loss_timeout_expired"),
    frame(17 * kFramePeriodNs, "new_capture_after_timeout", {subject(17 * kFramePeriodNs)}),
    frame(18 * kFramePeriodNs, "tracking_after_new_capture", {subject(18 * kFramePeriodNs)}),
  };
}

std::vector<SyntheticFrame> invalid_input_frames()
{
  return {
    frame(0, "initial_detection", {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(kFramePeriodNs, "tracking_before_invalid_inputs",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(-1, "timestamp_negative",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(9'000'000, "timestamp_rollback",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(kFramePeriodNs, "timestamp_duplicate",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(2 * kFramePeriodNs, "nan_camera_position",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0, 0.9F, 1,
        ObservationFault::NaNCameraPosition)}),
    frame(3 * kFramePeriodNs, "infinite_relative_yaw",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0, 0.9F, 1,
        ObservationFault::InfiniteRelativeYaw)}),
    frame(4 * kFramePeriodNs, "invalid_observation_flag",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0, 0.9F, 1,
        ObservationFault::InvalidFlag)}),
    frame(5 * kFramePeriodNs, "recovery_detection",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
    frame(6 * kFramePeriodNs, "recovery_tracking",
      {target("invalid", {3.0, 0.0, 0.0}, 0.0, 0.0)}),
  };
}

std::vector<SyntheticFrame> ballistic_failure_frames()
{
  const auto subject = []() {
      return target("ballistic", {3.0, 0.0, 0.0}, 0.0, 0.0);
    };
  return {
    frame(0, "initial_detection", {subject()}),
    frame(kFramePeriodNs, "nominal_ballistic_diagnostic", {subject()}),
    frame(2 * kFramePeriodNs, "ballistic_missing_muzzle_transform", {subject()}, false,
      BallisticFixture::MissingMuzzleTransform),
    frame(3 * kFramePeriodNs, "ballistic_unreachable", {subject()}, false,
      BallisticFixture::Unreachable),
    frame(4 * kFramePeriodNs, "ballistic_missing_bullet_speed", {subject()}, false,
      BallisticFixture::MissingBulletSpeed),
    frame(5 * kFramePeriodNs, "ballistic_invalid_bullet_speed", {subject()}, false,
      BallisticFixture::InvalidBulletSpeed),
    frame(6 * kFramePeriodNs, "ballistic_horizon_exceeds_maximum", {subject()}, false,
      BallisticFixture::HorizonExceedsMaximum),
  };
}

std::vector<SyntheticFrame> scenario_frames(const std::string & scenario)
{
  if (scenario == "static_3m") {
    return static_3m_frames();
  }
  if (scenario == "spin_3m") {
    return spin_3m_frames();
  }
  if (scenario == "spin_translate_3m") {
    return spin_translate_3m_frames();
  }
  if (scenario == "crossing_permuted") {
    return crossing_permuted_frames();
  }
  if (scenario == "occlusion_reacquisition") {
    return occlusion_reacquisition_frames();
  }
  if (scenario == "invalid_inputs") {
    return invalid_input_frames();
  }
  if (scenario == "ballistic_failures") {
    return ballistic_failure_frames();
  }
  throw std::invalid_argument("unknown offline scenario benchmark scenario: " + scenario);
}

std::vector<std::string> canonical_scenarios()
{
  return {
    "static_3m",
    "spin_3m",
    "spin_translate_3m",
    "crossing_permuted",
    "occlusion_reacquisition",
    "invalid_inputs",
    "ballistic_failures",
  };
}

std::vector<SyntheticTarget> ordered_targets(
  const SyntheticFrame & frame_spec,
  std::uint64_t seed,
  std::size_t frame_index)
{
  auto result = frame_spec.targets;
  if (frame_spec.permute_input && result.size() > 1U) {
    // Deliberately avoid a library distribution: this explicit seed-derived
    // ordering is reproducible by construction and exercises order-independence
    // without pretending to model detector randomness.
    const auto offset = static_cast<std::size_t>((seed + frame_index) % result.size());
    std::rotate(result.begin(), result.begin() + static_cast<std::ptrdiff_t>(offset), result.end());
    if (((seed >> (frame_index % 32U)) & 1U) != 0U) {
      std::reverse(result.begin(), result.end());
    }
  }
  return result;
}

std::string input_order_text(const std::vector<SyntheticTarget> & targets)
{
  std::string result;
  for (std::size_t index = 0; index < targets.size(); ++index) {
    if (index > 0U) {
      result += '>';
    }
    result += targets[index].id;
  }
  return result.empty() ? "none" : result;
}

TargetObservation make_observation(
  const SyntheticTarget & synthetic,
  std::int64_t stamp_ns,
  std::size_t detection_index)
{
  TargetObservation result{};
  result.detection_index = detection_index;
  result.class_id = synthetic.class_id;
  result.armor_size = pnp::ArmorSize::Small;
  result.confidence = synthetic.confidence;
  // This is an explicit synthetic association coordinate in the documented
  // OpenCV camera axis convention, not a camera/muzzle calibration claim.
  result.camera_xyz_m = cv::Vec3d{
    -synthetic.muzzle_m[1], -synthetic.muzzle_m[2], synthetic.muzzle_m[0]};
  result.relative_yaw_rad = synthetic.relative_yaw_rad;
  result.relative_pitch_rad = synthetic.relative_pitch_rad;
  result.reprojection_error_px = 0.25;
  result.stamp_ns = stamp_ns;
  result.valid = true;
  result.geometry_known = true;
  result.raw_detection.class_id = synthetic.class_id;
  result.raw_detection.color_id = 0;
  result.raw_detection.armor_type =
    detector::RawArmorDetection::ArmorTypeHint::Small;
  result.raw_detection.confidence = synthetic.confidence;
  const auto center_x = static_cast<float>(640.0 + synthetic.relative_yaw_rad * 300.0);
  const auto center_y = static_cast<float>(400.0 - synthetic.relative_pitch_rad * 300.0);
  result.raw_detection.bbox = cv::Rect2f(center_x - 20.0F, center_y - 20.0F, 40.0F, 40.0F);
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

  switch (synthetic.fault) {
    case ObservationFault::None:
      break;
    case ObservationFault::NaNCameraPosition:
      (*result.camera_xyz_m)[0] = std::numeric_limits<double>::quiet_NaN();
      break;
    case ObservationFault::InfiniteRelativeYaw:
      result.relative_yaw_rad = std::numeric_limits<double>::infinity();
      break;
    case ObservationFault::InvalidFlag:
      result.valid = false;
      break;
  }
  return result;
}

TrackerConfig benchmark_tracker_config()
{
  TrackerConfig result{};
  result.min_detect_count = 2;
  result.max_temp_lost_ms = 100;
  result.max_position_jump_m = 0.75;
  result.max_angle_jump_rad = 0.75;
  result.max_velocity_rad_s = 0.0;
  return result;
}

PredictionConfig benchmark_prediction_config()
{
  PredictionConfig result{};
  result.enabled = true;
  result.horizon_ns = kFramePeriodNs;
  result.max_horizon_ns = 500'000'000;
  return result;
}

BallisticConfig benchmark_ballistic_config(BallisticFixture fixture)
{
  BallisticConfig result{};
  result.enabled = true;
  result.bullet_speed_mps = 20.0;
  result.gravity_mps2 = 9.81;
  result.system_latency_ns = 10'000'000;
  result.max_flight_time_ns = 500'000'000;
  result.max_prediction_horizon_ns = 500'000'000;
  switch (fixture) {
    case BallisticFixture::Nominal:
    case BallisticFixture::MissingMuzzleTransform:
      break;
    case BallisticFixture::Unreachable:
      result.bullet_speed_mps = 1.0;
      break;
    case BallisticFixture::MissingBulletSpeed:
      result.bullet_speed_mps.reset();
      break;
    case BallisticFixture::InvalidBulletSpeed:
      result.bullet_speed_mps = 0.0;
      break;
    case BallisticFixture::HorizonExceedsMaximum:
      result.max_prediction_horizon_ns = 1;
      break;
  }
  return result;
}

const SyntheticTarget * selected_synthetic_target(
  const std::optional<TrackedTarget> & selected,
  const std::vector<SyntheticTarget> & targets)
{
  if (!selected.has_value()) {
    return nullptr;
  }
  const auto index = selected->observation.detection_index;
  return index < targets.size() ? &targets[index] : nullptr;
}

const SyntheticTarget * future_target(
  const std::vector<SyntheticFrame> & frames,
  const PredictionResult & prediction,
  const std::string & target_id)
{
  if (!prediction.valid || prediction.predicted_stamp_ns < 0) {
    return nullptr;
  }
  const auto frame_it = std::find_if(
    frames.begin(), frames.end(), [&](const SyntheticFrame & candidate) {
      return candidate.stamp_ns == prediction.predicted_stamp_ns;
    });
  if (frame_it == frames.end()) {
    return nullptr;
  }
  const auto target_it = std::find_if(
    frame_it->targets.begin(), frame_it->targets.end(), [&](const SyntheticTarget & candidate) {
      return candidate.id == target_id && candidate.fault == ObservationFault::None;
    });
  return target_it == frame_it->targets.end() ? nullptr : &*target_it;
}

void record_truth(
  OfflineScenarioBenchmarkRecord & record,
  const SyntheticTarget * target)
{
  if (target == nullptr || target->fault != ObservationFault::None ||
    !finite_vector(target->muzzle_m) || !finite(target->relative_yaw_rad) ||
    !finite(target->relative_pitch_rad))
  {
    return;
  }
  record.truth_valid = true;
  record.truth_id = target->id;
  record.truth_muzzle_m = target->muzzle_m;
  record.truth_relative_yaw_rad = target->relative_yaw_rad;
  record.truth_relative_pitch_rad = target->relative_pitch_rad;
}

OfflineScenarioBenchmarkRecord run_frame(
  const std::string & scenario,
  std::uint64_t seed,
  std::size_t frame_index,
  const SyntheticFrame & frame_spec,
  const std::vector<SyntheticFrame> & all_frames,
  OfflineTracker & tracker,
  TargetSelector & selector,
  SafeOfflineAimer & aimer,
  OfflinePredictor * predictor,
  bool diagnostics_enabled)
{
  const auto targets = ordered_targets(frame_spec, seed, frame_index);
  std::vector<TargetObservation> observations;
  observations.reserve(targets.size());
  for (std::size_t index = 0; index < targets.size(); ++index) {
    observations.push_back(make_observation(targets[index], frame_spec.stamp_ns, index));
  }

  const auto update = tracker.update(observations, frame_spec.stamp_ns);
  const auto selected = selector.select(update.tracks, kImageWidth, kImageHeight);
  const auto aimed = aimer.aim(selected);
  const auto safe_command = aimed.safe_command();

  OfflineScenarioBenchmarkRecord record{};
  record.scenario = scenario;
  record.seed = seed;
  record.frame_index = frame_index;
  record.stamp_ns = frame_spec.stamp_ns;
  record.event = frame_spec.event;
  record.input_order = input_order_text(targets);
  record.tracker_accepted = update.accepted;
  record.tracker_rejected = update.rejected;
  record.tracker_association_result = association_result_name(update.association_result);
  record.tracker_association_reason = update.association_reason;
  if (update.primary_track.has_value()) {
    record.primary_track_id = update.primary_track->track_id;
    record.primary_tracking_state = tracking_state_name(update.primary_track->state);
    record.primary_association_result =
      association_result_name(update.primary_track->association_result);
    record.primary_association_reason = update.primary_track->association_reason;
  } else {
    record.primary_tracking_state = tracking_state_name(TrackingState::Lost);
    record.primary_association_result = association_result_name(AssociationResult::None);
  }
  const auto & selector_diagnostics = selector.diagnostics();
  if (selected.has_value()) {
    record.selected_track_id = selected->track_id;
  }
  record.selector_switched = selector_diagnostics.switched;
  record.selector_switch_reason = selector_diagnostics.switch_reason;
  record.diagnostic_target_lock = aimed.target_lock;
  record.safe_command_target_lock = safe_command.target_lock;
  record.fire_command = safe_command.fire_command;
  record.yaw_vel_rad_s = safe_command.yaw_vel_rad_s;
  record.pitch_vel_rad_s = safe_command.pitch_vel_rad_s;
  record.yaw_acc_rad_s2 = safe_command.yaw_acc_rad_s2;
  record.pitch_acc_rad_s2 = safe_command.pitch_acc_rad_s2;

  const auto * selected_target = selected_synthetic_target(selected, targets);
  record_truth(record, selected_target);
  if (!record.truth_valid && !targets.empty()) {
    // A rejected observation is intentionally not promoted to truth evidence,
    // but its synthetic ID remains useful for diagnosing the fixture event.
    record.truth_id = targets.front().id;
  }

  if (!diagnostics_enabled) {
    record.predictor_reason = "disabled_by_benchmark_config";
    record.predictor_error_reason = "diagnostics_disabled";
    record.ballistic_reason = "disabled_by_benchmark_config";
    return record;
  }

  record.predictor_enabled = true;
  const auto prediction = predictor->predict(selected, frame_spec.stamp_ns);
  record.predictor_valid = prediction.valid;
  record.predictor_reason = prediction_failure_reason_name(prediction.failure_reason);
  if (prediction.predicted_stamp_ns >= 0) {
    record.predictor_future_stamp_ns = prediction.predicted_stamp_ns;
  }
  if (prediction.valid && selected_target != nullptr) {
    const auto * future = future_target(all_frames, prediction, selected_target->id);
    if (future != nullptr) {
      const auto error = diagnose_synthetic_prediction_error(
        prediction, make_observation(*future, prediction.predicted_stamp_ns, 0));
      record.predictor_error_reason = error.reason;
      if (error.valid) {
        record.predictor_yaw_error_rad = error.yaw_error_rad;
        record.predictor_pitch_error_rad = error.pitch_error_rad;
      }
    } else {
      record.predictor_error_reason = "no_synthetic_future_truth_at_prediction_stamp";
    }
  } else {
    record.predictor_error_reason = "prediction_not_valid";
  }

  record.ballistic_enabled = true;
  const OfflineBallisticDiagnostic diagnostic(benchmark_ballistic_config(frame_spec.ballistic_fixture));
  const bool intentional_missing_muzzle =
    frame_spec.ballistic_fixture == BallisticFixture::MissingMuzzleTransform;
  if (intentional_missing_muzzle) {
    record.origin_assumption = "omitted_for_intentional_failure";
  }
  const auto ballistic = selected_target == nullptr || intentional_missing_muzzle ?
    diagnostic.diagnose(selected, frame_spec.stamp_ns) :
    diagnostic.diagnose(selected, frame_spec.stamp_ns, selected_target->muzzle_m);
  record.ballistic_valid = ballistic.valid;
  record.ballistic_reason = ballistic_failure_reason_name(ballistic.failure_reason);
  record.ballistic_flight_time_s = ballistic.flight_time_s;
  record.ballistic_flight_time_ns = ballistic.flight_time_ns;
  record.ballistic_recommended_horizon_ns = ballistic.recommended_prediction_horizon_ns;
  return record;
}

std::vector<std::string> selected_scenarios(const std::string & scenario)
{
  if (scenario == "all") {
    return canonical_scenarios();
  }
  if (!is_offline_scenario_benchmark_scenario(scenario)) {
    throw std::invalid_argument("unknown offline scenario benchmark scenario: " + scenario);
  }
  return {scenario};
}

std::string record_csv_line(const OfflineScenarioBenchmarkRecord & record)
{
  std::vector<std::string> fields;
  fields.reserve(55);
  fields.push_back(record.scenario);
  fields.push_back(std::to_string(record.schema_version));
  fields.push_back(std::to_string(record.seed));
  fields.push_back(std::to_string(record.frame_index));
  fields.push_back(std::to_string(record.stamp_ns));
  fields.push_back(record.event);
  fields.push_back(record.input_order);
  fields.push_back(bool_text(record.synthetic));
  fields.push_back(bool_text(record.test_only));
  fields.push_back(bool_text(record.production_ready));
  fields.push_back(bool_text(record.truth_valid));
  fields.push_back(record.truth_id);
  fields.push_back(record.truth_muzzle_m.has_value() ? stable_double((*record.truth_muzzle_m)[0]) : "");
  fields.push_back(record.truth_muzzle_m.has_value() ? stable_double((*record.truth_muzzle_m)[1]) : "");
  fields.push_back(record.truth_muzzle_m.has_value() ? stable_double((*record.truth_muzzle_m)[2]) : "");
  fields.push_back(optional_double_text(record.truth_relative_yaw_rad));
  fields.push_back(optional_double_text(record.truth_relative_pitch_rad));
  fields.push_back(bool_text(record.tracker_accepted));
  fields.push_back(bool_text(record.tracker_rejected));
  fields.push_back(record.tracker_association_result);
  fields.push_back(record.tracker_association_reason);
  fields.push_back(optional_integer_text(record.primary_track_id));
  fields.push_back(record.primary_tracking_state);
  fields.push_back(record.primary_association_result);
  fields.push_back(record.primary_association_reason);
  fields.push_back(optional_integer_text(record.selected_track_id));
  fields.push_back(bool_text(record.selector_switched));
  fields.push_back(record.selector_switch_reason);
  fields.push_back(bool_text(record.predictor_enabled));
  fields.push_back(bool_text(record.predictor_valid));
  fields.push_back(record.predictor_reason);
  fields.push_back(optional_integer_text(record.predictor_future_stamp_ns));
  fields.push_back(optional_double_text(record.predictor_yaw_error_rad));
  fields.push_back(optional_double_text(record.predictor_pitch_error_rad));
  fields.push_back(record.predictor_error_reason);
  fields.push_back(bool_text(record.ballistic_enabled));
  fields.push_back(bool_text(record.ballistic_valid));
  fields.push_back(record.ballistic_reason);
  fields.push_back(optional_double_text(record.ballistic_flight_time_s));
  fields.push_back(optional_integer_text(record.ballistic_flight_time_ns));
  fields.push_back(optional_integer_text(record.ballistic_recommended_horizon_ns));
  fields.push_back(record.origin_assumption);
  fields.push_back(std::to_string(static_cast<int>(record.diagnostic_target_lock)));
  fields.push_back(std::to_string(static_cast<int>(record.safe_command_target_lock)));
  fields.push_back(std::to_string(static_cast<int>(record.fire_command)));
  fields.push_back(stable_double(record.yaw_vel_rad_s));
  fields.push_back(stable_double(record.pitch_vel_rad_s));
  fields.push_back(stable_double(record.yaw_acc_rad_s2));
  fields.push_back(stable_double(record.pitch_acc_rad_s2));
  fields.push_back(bool_text(record.serial_enabled));
  fields.push_back(bool_text(record.dry_run));
  fields.push_back(bool_text(record.allow_fire));

  std::ostringstream stream;
  for (std::size_t index = 0; index < fields.size(); ++index) {
    if (index > 0U) {
      stream << ',';
    }
    stream << csv_escape(fields[index]);
  }
  return stream.str();
}

template<typename Counter>
void write_json_counter(
  std::ostringstream & stream,
  const Counter & counter,
  const char * indentation)
{
  std::size_t index = 0;
  for (const auto & entry : counter) {
    stream << indentation << '"' << json_escape(entry.first) << "\": " << entry.second;
    if (++index < counter.size()) {
      stream << ',';
    }
    stream << '\n';
  }
}

std::map<std::string, std::size_t> scenario_counts(
  const OfflineScenarioBenchmarkResult & result)
{
  std::map<std::string, std::size_t> counts;
  for (const auto & record : result.records) {
    ++counts[record.scenario];
  }
  return counts;
}

std::map<std::string, std::size_t> ballistic_reason_counts(
  const OfflineScenarioBenchmarkResult & result)
{
  std::map<std::string, std::size_t> counts;
  for (const auto & record : result.records) {
    ++counts[record.ballistic_reason];
  }
  return counts;
}

void require_safe_new_output_directory(const std::filesystem::path & requested)
{
  if (requested.empty()) {
    throw std::invalid_argument("scenario benchmark --output-dir must not be empty");
  }
  std::error_code error;
  const auto output = std::filesystem::absolute(requested, error).lexically_normal();
  if (error) {
    throw std::runtime_error("unable to normalize scenario benchmark output directory");
  }
  const auto output_status = std::filesystem::symlink_status(output, error);
  if (!error && std::filesystem::exists(output_status)) {
    throw std::invalid_argument(
      "scenario benchmark output directory must not already exist: " + output.string());
  }
  if (error && error != std::errc::no_such_file_or_directory) {
    throw std::runtime_error(
      "unable to inspect scenario benchmark output directory: " + output.string());
  }

  auto parent = output.parent_path();
  while (!parent.empty()) {
    error.clear();
    const auto parent_status = std::filesystem::symlink_status(parent, error);
    if (error || !std::filesystem::is_directory(parent_status) ||
      std::filesystem::is_symlink(parent_status))
    {
      throw std::runtime_error(
        "scenario benchmark output parent must be an existing non-symlink directory: " +
        parent.string());
    }
    if (parent == parent.root_path()) {
      break;
    }
    parent = parent.parent_path();
  }
}

void write_new_file(const std::filesystem::path & path, const std::string & content)
{
  std::ofstream output(path, std::ios::binary | std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("unable to create scenario benchmark output: " + path.string());
  }
  output.imbue(std::locale::classic());
  output << content;
  if (!output) {
    throw std::runtime_error("unable to write scenario benchmark output: " + path.string());
  }
}

}  // namespace

std::vector<std::string> offline_scenario_benchmark_scenarios()
{
  auto result = canonical_scenarios();
  result.insert(result.begin(), "all");
  return result;
}

bool is_offline_scenario_benchmark_scenario(const std::string & scenario) noexcept
{
  const auto scenarios = canonical_scenarios();
  return scenario == "all" ||
    std::find(scenarios.begin(), scenarios.end(), scenario) != scenarios.end();
}

OfflineScenarioBenchmarkResult run_offline_scenario_benchmark(
  const OfflineScenarioBenchmarkConfig & config)
{
  if (config.scenario.empty()) {
    throw std::invalid_argument("offline scenario benchmark requires an explicit scenario");
  }

  OfflineScenarioBenchmarkResult result{};
  result.requested_scenario = config.scenario;
  result.seed = config.seed;
  result.diagnostics_enabled = config.diagnostics_enabled;

  for (const auto & scenario : selected_scenarios(config.scenario)) {
    const auto frames = scenario_frames(scenario);
    OfflineTracker tracker(benchmark_tracker_config());
    TargetSelector selector;
    SafeOfflineAimer aimer;
    std::optional<OfflinePredictor> predictor;
    if (config.diagnostics_enabled) {
      predictor.emplace(benchmark_prediction_config());
    }
    for (std::size_t index = 0; index < frames.size(); ++index) {
      result.records.push_back(run_frame(
        scenario, config.seed, index, frames[index], frames, tracker, selector, aimer,
        predictor.has_value() ? &*predictor : nullptr, config.diagnostics_enabled));
    }
  }
  return result;
}

std::string render_offline_scenario_benchmark_csv(
  const OfflineScenarioBenchmarkResult & result)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream <<
    "scenario,schema_version,seed,frame_index,stamp_ns,event,input_order,"
    "synthetic,test_only,production_ready,truth_valid,truth_id,truth_muzzle_x_m,"
    "truth_muzzle_y_m,truth_muzzle_z_m,truth_relative_yaw_rad,truth_relative_pitch_rad,"
    "tracker_accepted,tracker_rejected,tracker_association_result,tracker_association_reason,"
    "primary_track_id,primary_tracking_state,primary_association_result,"
    "primary_association_reason,selected_track_id,selector_switched,selector_switch_reason,"
    "predictor_enabled,predictor_valid,predictor_reason,predictor_future_stamp_ns,"
    "predictor_yaw_error_rad,predictor_pitch_error_rad,predictor_error_reason,"
    "ballistic_enabled,ballistic_valid,ballistic_reason,ballistic_flight_time_s,"
    "ballistic_flight_time_ns,ballistic_recommended_horizon_ns,origin_assumption,"
    "diagnostic_target_lock,safe_command_target_lock,fire_command,yaw_vel_rad_s,"
    "pitch_vel_rad_s,yaw_acc_rad_s2,pitch_acc_rad_s2,serial_enabled,dry_run,allow_fire\n";
  for (const auto & record : result.records) {
    stream << record_csv_line(record) << '\n';
  }
  return stream.str();
}

std::string render_offline_scenario_benchmark_json(
  const OfflineScenarioBenchmarkResult & result)
{
  const auto scenarios = scenario_counts(result);
  const auto ballistic_reasons = ballistic_reason_counts(result);
  const auto valid_predictions = std::count_if(
    result.records.begin(), result.records.end(), [](const OfflineScenarioBenchmarkRecord & record) {
      return record.predictor_valid;
    });
  const auto valid_ballistics = std::count_if(
    result.records.begin(), result.records.end(), [](const OfflineScenarioBenchmarkRecord & record) {
      return record.ballistic_valid;
    });
  const auto safe_records = std::count_if(
    result.records.begin(), result.records.end(), [](const OfflineScenarioBenchmarkRecord & record) {
      return !record.production_ready && record.synthetic && record.test_only &&
        record.fire_command == 0 && record.yaw_vel_rad_s == 0.0F &&
        record.pitch_vel_rad_s == 0.0F && record.yaw_acc_rad_s2 == 0.0F &&
        record.pitch_acc_rad_s2 == 0.0F && !record.serial_enabled &&
        record.dry_run && !record.allow_fire;
    });

  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "{\n"
    << "  \"schema_version\": " << result.schema_version << ",\n"
    << "  \"requested_scenario\": \"" << json_escape(result.requested_scenario) << "\",\n"
    << "  \"seed\": " << result.seed << ",\n"
    << "  \"diagnostics_enabled\": " << bool_text(result.diagnostics_enabled) << ",\n"
    << "  \"synthetic\": true,\n"
    << "  \"test_only\": true,\n"
    << "  \"production_ready\": false,\n"
    << "  \"software_only_synthetic_benchmark\": true,\n"
    << "  \"hardware_validation\": false,\n"
    << "  \"real_hit_rate_computed\": false,\n"
    << "  \"record_count\": " << result.records.size() << ",\n"
    << "  \"safe_record_count\": " << safe_records << ",\n"
    << "  \"valid_prediction_count\": " << valid_predictions << ",\n"
    << "  \"valid_ballistic_count\": " << valid_ballistics << ",\n"
    << "  \"scenario_record_counts\": {\n";
  write_json_counter(stream, scenarios, "    ");
  stream << "  },\n"
    << "  \"ballistic_reason_counts\": {\n";
  write_json_counter(stream, ballistic_reasons, "    ");
  stream << "  },\n"
    << "  \"safety\": {\n"
    << "    \"serial_enabled\": false,\n"
    << "    \"dry_run\": true,\n"
    << "    \"allow_fire\": false,\n"
    << "    \"fire_command\": 0,\n"
    << "    \"yaw_vel_rad_s\": 0,\n"
    << "    \"pitch_vel_rad_s\": 0,\n"
    << "    \"yaw_acc_rad_s2\": 0,\n"
    << "    \"pitch_acc_rad_s2\": 0,\n"
    << "    \"diagnostic_target_lock_is_not_publishable_robotctrl_lock\": true\n"
    << "  }\n"
    << "}\n";
  return stream.str();
}

std::string render_offline_scenario_benchmark_markdown(
  const OfflineScenarioBenchmarkResult & result)
{
  const auto scenarios = scenario_counts(result);
  const auto ballistic_reasons = ballistic_reason_counts(result);
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "# Offline scenario benchmark\n\n"
    << "This is a software-only synthetic benchmark. It is not hardware validation, "
    << "real trajectory validation, hit-rate evidence, latency evidence, or a competition-performance conclusion.\n\n"
    << "- Schema version: " << result.schema_version << "\n"
    << "- Requested scenario: " << result.requested_scenario << "\n"
    << "- Fixed seed: " << result.seed << "\n"
    << "- Diagnostics enabled: " << bool_text(result.diagnostics_enabled) << "\n"
    << "- Synthetic/test-only/production-ready: true / true / false\n"
    << "- Records: " << result.records.size() << "\n"
    << "- Muzzle origin label: synthetic_muzzle_frame (x forward, y left, z up; metres)\n\n"
    << "## Scenario records\n\n"
    << "| scenario | records |\n"
    << "| --- | ---: |\n";
  for (const auto & entry : scenarios) {
    stream << "| " << entry.first << " | " << entry.second << " |\n";
  }
  stream << "\n## Ballistic diagnostic outcomes\n\n"
    << "| reason | records |\n"
    << "| --- | ---: |\n";
  for (const auto & entry : ballistic_reasons) {
    stream << "| " << entry.first << " | " << entry.second << " |\n";
  }
  stream << "\n## Safety boundary\n\n"
    << "- serial_enabled=false\n"
    << "- dry_run=true\n"
    << "- allow_fire=false\n"
    << "- fire_command=0\n"
    << "- yaw/pitch velocity and acceleration=0\n"
    << "- Predictor and BallisticDiagnostic are read-only diagnostics; no output is sent to "
    << "AimCommand, ROS, RobotCtrl, serial, a gimbal, or firing.\n";
  return stream.str();
}

void write_offline_scenario_benchmark_outputs(
  const OfflineScenarioBenchmarkResult & result,
  const std::filesystem::path & output_dir)
{
  require_safe_new_output_directory(output_dir);

  std::error_code error;
  const auto normalized = std::filesystem::absolute(output_dir, error).lexically_normal();
  if (error || !std::filesystem::create_directory(normalized, error) || error) {
    throw std::runtime_error(
      "unable to create new scenario benchmark output directory: " + normalized.string());
  }
  write_new_file(normalized / "benchmark.csv", render_offline_scenario_benchmark_csv(result));
  write_new_file(normalized / "summary.json", render_offline_scenario_benchmark_json(result));
  write_new_file(normalized / "summary.md", render_offline_scenario_benchmark_markdown(result));
}

}  // namespace rm_auto_aim::offline
