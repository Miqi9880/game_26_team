#include "auto_aim_ros2/ros_backend.hpp"

#include "auto_aim_ros2/ros_adapters.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace rm_auto_aim::ros_backend
{
namespace
{

std::string csv_value(double value)
{
  if (!std::isfinite(value)) {
    return {};
  }
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

std::string csv_value(float value)
{
  return csv_value(static_cast<double>(value));
}

std::string csv_optional(const std::optional<double> & value)
{
  return value.has_value() ? csv_value(*value) : std::string{};
}

std::string sanitize_error(std::string value)
{
  std::replace(value.begin(), value.end(), ',', ';');
  std::replace(value.begin(), value.end(), '\n', ' ');
  return value;
}

void append_csv_field(std::ostringstream & stream, const std::string & value, bool & first)
{
  if (!first) {
    stream << ',';
  }
  stream << value;
  first = false;
}

std::unique_ptr<pipeline::AutoAimPipeline> make_core_pipeline(const Config & config)
{
  std::unique_ptr<pipeline::YoloStage> yolo;
  if (config.kind == BackendKind::Mock && config.mock_target) {
    pipeline::Detection mock{};
    mock.valid = true;
    mock.yaw_rad = config.mock_yaw_rad;
    mock.pitch_rad = config.mock_pitch_rad;
    mock.fire_request = config.mock_fire_request;
    yolo = std::make_unique<pipeline::MockYoloStage>(mock);
  } else {
    yolo = std::make_unique<pipeline::NullYoloStage>();
  }

  pipeline::CoreConfig core_config{};
  // Core backends are retained for compatibility, but this node's dry-run
  // boundary is always the final fire inhibit.
  core_config.allow_fire = config.allow_fire && !config.dry_run;
  return std::make_unique<pipeline::AutoAimPipeline>(
    std::move(yolo),
    std::make_unique<pipeline::PassThroughArmorStage>(),
    std::make_unique<pipeline::LatestTargetTracker>(),
    std::make_unique<pipeline::FirstTargetStage>(),
    std::make_unique<pipeline::CommandAimer>(),
    core_config);
}

}  // namespace

const char * backend_kind_name(BackendKind kind) noexcept
{
  switch (kind) {
    case BackendKind::Null:
      return "null";
    case BackendKind::Mock:
      return "mock";
    case BackendKind::OfflineReference:
      return "offline_reference";
  }
  return "unknown";
}

BackendKind parse_backend_kind(const std::string & name)
{
  if (name == "null") {
    return BackendKind::Null;
  }
  if (name == "mock") {
    return BackendKind::Mock;
  }
  if (name == "offline_reference") {
    return BackendKind::OfflineReference;
  }
  throw std::invalid_argument(
          "backend must be one of: null, mock, offline_reference; got '" + name + "'");
}

struct Backend::Impl
{
  explicit Impl(Config input) : config(std::move(input)) {}

  Config config;
  std::string calibration_profile{"none"};
  bool test_only{true};
  std::unique_ptr<pipeline::AutoAimPipeline> core_pipeline;
  std::unique_ptr<detector::OpenVinoYoloDetector> detector;
  std::unique_ptr<pnp::PnpStage> pnp_stage;
  std::unique_ptr<offline::OfflineTracker> tracker;
  std::unique_ptr<offline::TargetSelector> selector;
  std::unique_ptr<offline::SafeOfflineAimer> aimer;
};

Backend::Backend(Config config) : impl_(std::make_unique<Impl>(std::move(config)))
{
  auto & state = *impl_;
  if (state.config.kind == BackendKind::OfflineReference) {
    if (!state.config.dry_run || state.config.serial_enabled || state.config.allow_fire) {
      throw std::invalid_argument(
              "offline_reference requires dry_run=true, serial_enabled=false, allow_fire=false");
    }
    if (state.config.model_path.empty() || state.config.pnp_config_path.empty()) {
      throw std::invalid_argument(
              "offline_reference requires offline_model_path and offline_pnp_config");
    }
    if (state.config.aimer.mode == offline::AimerMode::TestAbsoluteZero &&
      !state.config.aimer.absolute_zero_configured)
    {
      throw std::invalid_argument(
              "test_absolute_zero requires explicit offline test zero configuration");
    }

    pnp::ConfigLoadOptions options{};
    options.allow_test_only = state.config.allow_test_only;
    auto pnp_config = pnp::load_pnp_configuration(state.config.pnp_config_path, options);
    if (pnp_config.test_only && !state.config.allow_test_only) {
      throw std::invalid_argument("test_only PnP configuration requires allow_test_only=true");
    }
    state.calibration_profile = pnp_config.test_only ? "test_only" : "production";
    state.test_only = pnp_config.test_only || state.config.aimer.test_only;
    state.pnp_stage = std::make_unique<pnp::PnpStage>(std::move(pnp_config));

    detector::DetectorConfig detector_config{};
    detector_config.model_path = state.config.model_path;
    detector_config.device = state.config.device;
    state.detector = std::make_unique<detector::OpenVinoYoloDetector>(
      std::move(detector_config));
    state.tracker = std::make_unique<offline::OfflineTracker>(state.config.tracker);
    state.selector = std::make_unique<offline::TargetSelector>();
    state.aimer = std::make_unique<offline::SafeOfflineAimer>(state.config.aimer);
  } else {
    state.core_pipeline = make_core_pipeline(state.config);
    state.test_only = true;
  }
}

Backend::~Backend() = default;

Backend::Backend(Backend &&) noexcept = default;
Backend & Backend::operator=(Backend &&) noexcept = default;

FrameResult Backend::safe_result(std::int64_t stamp_ns) const
{
  FrameResult result{};
  result.backend = backend_kind_name(impl_->config.kind);
  result.calibration_profile = impl_->calibration_profile;
  result.aimer_mode = impl_->config.kind == BackendKind::OfflineReference ?
    offline::aimer_mode_name(impl_->config.aimer.mode) : "core";
  result.test_only = impl_->test_only;
  result.stamp_ns = stamp_ns;
  result.command = pipeline::AutoAimPipeline::safe_command();
  result.command.fire_command = pipeline::kFireNone;
  result.diagnostic_target_lock = pipeline::kTargetUnlocked;
  result.absolute_command_valid = false;
  result.command_publishable = false;
  return result;
}

FrameResult Backend::process(const pipeline::ImageFrame & frame)
{
  auto & state = *impl_;
  FrameResult result = safe_result(frame.stamp_ns);
  try {
    if (state.config.kind != BackendKind::OfflineReference) {
      // Use the frame stamp as the deterministic time source.  This avoids
      // sampling wall time during replay or inside a ROS image callback.
      const auto safe_stamp_ns = std::max<std::int64_t>(frame.stamp_ns, 0);
      const auto timestamp = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(safe_stamp_ns));
      result.command = state.core_pipeline->process(frame, timestamp);
      result.command.fire_command = pipeline::kFireNone;
      result.diagnostic_target_lock = result.command.target_lock;
      result.aimer.target_lock = result.command.target_lock;
      result.aimer.fire_command = pipeline::kFireNone;
      result.aimer.mode = offline::AimerMode::RelativeDebug;
      result.aimer.test_only = true;
      // Null/Mock are compatibility dry-run backends.  They expose their
      // lock/angle diagnostics, but never pass fire through this boundary.
      result.absolute_command_valid = result.command.target_lock == pipeline::kTargetLocked;
      result.command_publishable = result.absolute_command_valid && !state.config.dry_run;
      return result;
    }

    const auto detections = state.detector->detect(frame);
    result.detection_count = detections.size();
    result.poses.reserve(detections.size());
    std::vector<offline::TargetObservation> target_observations;
    target_observations.reserve(detections.size());
    for (std::size_t index = 0; index < detections.size(); ++index) {
      auto pose = state.pnp_stage->solve(
        detections[index], static_cast<int>(frame.width), static_cast<int>(frame.height));
      if (pose.valid) {
        ++result.valid_pnp_count;
      }
      target_observations.push_back(
        offline::make_target_observation(pose, frame.stamp_ns, index));
      result.poses.push_back(std::move(pose));
    }

    result.tracker_update = state.tracker->update(target_observations, frame.stamp_ns);
    result.selected = state.selector->select(
      result.tracker_update.tracks, static_cast<int>(frame.width), static_cast<int>(frame.height));
    result.aimer = state.aimer->aim(result.selected, frame.shoot_speed_mps);
    result.diagnostic_target_lock = result.aimer.target_lock;
    result.absolute_command_valid = result.aimer.absolute_command_valid;
    result.command_publishable = result.absolute_command_valid &&
      !result.test_only && !state.config.dry_run && !state.config.serial_enabled &&
      !state.config.allow_fire;
    // SafeOfflineAimer::safe_command() deliberately publishes lock=50 unless
    // a complete absolute candidate exists.  RelativeDebug remains diagnostic
    // and cannot be mistaken for an absolute RobotCtrl command.
    result.command = result.aimer.safe_command();
    result.command.fire_command = pipeline::kFireNone;
    return result;
  } catch (const std::exception & error) {
    result.error = error.what();
  } catch (...) {
    result.error = "unknown backend exception";
  }

  // Fail closed on detector/PnP/tracker exceptions while retaining the frame
  // stamp and an actionable error for logs/CSV.
  result.command = pipeline::AutoAimPipeline::safe_command();
  result.command.fire_command = pipeline::kFireNone;
  result.command.target_lock = pipeline::kTargetUnlocked;
  result.diagnostic_target_lock = pipeline::kTargetUnlocked;
  result.absolute_command_valid = false;
  result.command_publishable = false;
  return result;
}

BackendKind Backend::kind() const noexcept
{
  return impl_->config.kind;
}

const std::string & Backend::calibration_profile() const noexcept
{
  return impl_->calibration_profile;
}

bool Backend::test_only() const noexcept
{
  return impl_->test_only;
}

const Config & Backend::config() const noexcept
{
  return impl_->config;
}

std::string csv_header()
{
  return "backend,calibration_profile,aimer_mode,test_only,diagnostic_target_lock,"
         "published_target_lock,absolute_command_valid,command_publishable,stamp_ns,"
         "detection_count,valid_pnp_count,track_id,tracking_state,consecutive_valid,"
         "camera_x_m,camera_y_m,camera_z_m,"
         "relative_yaw_rad,relative_pitch_rad,reprojection_error_px,command_yaw_rad_internal,"
         "command_pitch_rad_internal,command_yaw_degree,command_pitch_degree,ros_yaw_degree,"
         "ros_pitch_degree,ros_yaw_vel_external,ros_yaw_acc_external,ros_pitch_vel_external,"
         "ros_pitch_acc_external,fire_command,error\n";
}

std::string csv_row(const FrameResult & result)
{
  std::ostringstream stream;
  bool first = true;
  const auto append = [&](const std::string & value) {
      append_csv_field(stream, value, first);
  };
  append(result.backend);
  append(result.calibration_profile);
  append(result.aimer_mode);
  append(result.test_only ? "1" : "0");
  append(std::to_string(static_cast<int>(result.diagnostic_target_lock)));
  append(std::to_string(static_cast<int>(result.command.target_lock)));
  append(result.absolute_command_valid ? "1" : "0");
  append(result.command_publishable ? "1" : "0");
  append(std::to_string(result.stamp_ns));
  append(std::to_string(result.detection_count));
  append(std::to_string(result.valid_pnp_count));
  if (result.selected.has_value()) {
    append(std::to_string(result.selected->track_id));
    append(offline::tracking_state_name(result.selected->state));
    append(std::to_string(result.selected->consecutive_valid));
  } else if (result.tracker_update.primary_track.has_value()) {
    append(std::to_string(result.tracker_update.primary_track->track_id));
    append(offline::tracking_state_name(result.tracker_update.primary_track->state));
    append(std::to_string(result.tracker_update.primary_track->consecutive_valid));
  } else {
    append({});
    append("lost");
    append("0");
  }
  if (result.selected.has_value()) {
    const auto & observation = result.selected->observation;
    if (observation.camera_xyz_m.has_value()) {
      append(csv_value((*observation.camera_xyz_m)[0]));
      append(csv_value((*observation.camera_xyz_m)[1]));
      append(csv_value((*observation.camera_xyz_m)[2]));
    } else {
      append({});
      append({});
      append({});
    }
    append(csv_optional(observation.relative_yaw_rad));
    append(csv_optional(observation.relative_pitch_rad));
    append(csv_value(observation.reprojection_error_px));
  } else {
    for (int index = 0; index < 6; ++index) {
      append({});
    }
  }

  append(csv_value(result.command.yaw_rad));
  append(csv_value(result.command.pitch_rad));
  append(csv_value(units::radians_to_degrees(result.command.yaw_rad)));
  append(csv_value(units::radians_to_degrees(result.command.pitch_rad)));
  append(csv_value(units::radians_to_degrees(result.command.yaw_rad)));
  append(csv_value(units::radians_to_degrees(result.command.pitch_rad)));
  append("0");
  append("0");
  append("0");
  append("0");
  append(std::to_string(static_cast<int>(result.command.fire_command)));
  append(result.error.empty() ? std::string{} : sanitize_error(result.error));
  stream << '\n';
  return stream.str();
}

}  // namespace rm_auto_aim::ros_backend
