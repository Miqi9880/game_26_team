#include "auto_aim_ros2/auto_aim_core.hpp"

#include <cmath>

namespace rm_auto_aim::pipeline
{
namespace
{
bool finite_detection(const Detection & detection)
{
  return detection.valid && std::isfinite(detection.yaw_rad) &&
         std::isfinite(detection.pitch_rad) && std::isfinite(detection.yaw_vel_rad_s) &&
         std::isfinite(detection.pitch_vel_rad_s) && std::isfinite(detection.yaw_acc_rad_s2) &&
         std::isfinite(detection.pitch_acc_rad_s2);
}
}  // namespace

AutoAimPipeline::AutoAimPipeline(
  std::unique_ptr<YoloStage> yolo,
  std::unique_ptr<ArmorStage> armor,
  std::unique_ptr<TrackerStage> tracker,
  std::unique_ptr<TargetStage> target,
  std::unique_ptr<AimerStage> aimer,
  CoreConfig config)
: yolo_(std::move(yolo)),
  armor_(std::move(armor)),
  tracker_(std::move(tracker)),
  target_(std::move(target)),
  aimer_(std::move(aimer)),
  config_(config)
{
}

AimCommand AutoAimPipeline::process(
  const ImageFrame & frame, std::chrono::steady_clock::time_point timestamp)
{
  const auto detections = yolo_->detect(frame);
  const auto armors = armor_->build(detections);
  const auto tracked = tracker_->track(armors, timestamp);
  const auto selected = target_->select(tracked);
  return aimer_->aim(selected, config_);
}

AimCommand AutoAimPipeline::safe_command() noexcept
{
  return {};
}

MockYoloStage::MockYoloStage(std::optional<Detection> detection)
: detection_(std::move(detection))
{
}

std::vector<Detection> MockYoloStage::detect(const ImageFrame &)
{
  if (!detection_.has_value()) {
    return {};
  }
  return {detection_.value()};
}

std::vector<Detection> NullYoloStage::detect(const ImageFrame &)
{
  return {};
}

std::vector<ArmorObservation> PassThroughArmorStage::build(
  const std::vector<Detection> & detections)
{
  std::vector<ArmorObservation> armors;
  for (const auto & detection : detections) {
    if (finite_detection(detection)) {
      armors.push_back({detection});
    }
  }
  return armors;
}

std::optional<TargetState> LatestTargetTracker::track(
  const std::vector<ArmorObservation> & armors, std::chrono::steady_clock::time_point)
{
  if (armors.empty()) {
    return std::nullopt;
  }
  return TargetState{armors.front().detection};
}

std::optional<Detection> FirstTargetStage::select(const std::optional<TargetState> & target)
{
  if (!target.has_value()) {
    return std::nullopt;
  }
  return target->estimate;
}

AimCommand CommandAimer::aim(
  const std::optional<Detection> & target, const CoreConfig & config)
{
  if (!target.has_value() || !finite_detection(target.value())) {
    return AutoAimPipeline::safe_command();
  }

  AimCommand command{};
  command.yaw_rad = target->yaw_rad;
  command.yaw_vel_rad_s = target->yaw_vel_rad_s;
  command.yaw_acc_rad_s2 = target->yaw_acc_rad_s2;
  command.pitch_rad = target->pitch_rad;
  command.pitch_vel_rad_s = target->pitch_vel_rad_s;
  command.pitch_acc_rad_s2 = target->pitch_acc_rad_s2;
  command.target_lock = kTargetLocked;
  if (config.allow_fire && target->fire_request) {
    command.fire_command = config.burst_command;
  }
  return command;
}

}  // namespace rm_auto_aim::pipeline
