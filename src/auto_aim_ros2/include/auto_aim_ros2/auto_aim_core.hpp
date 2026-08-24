#ifndef AUTO_AIM_ROS2__AUTO_AIM_CORE_HPP_
#define AUTO_AIM_ROS2__AUTO_AIM_CORE_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm_auto_aim::pipeline
{

constexpr std::int8_t kTargetLocked = 49;
constexpr std::int8_t kTargetUnlocked = 50;
constexpr std::int8_t kFireNone = 0;
constexpr std::int8_t kFireBurst = 1;
constexpr std::int8_t kFireSingle = 2;

// VisionState is the ROS input adapter boundary.  Position angles have
// already been converted from ROS/serial degree to algorithm radians when a
// value reaches this type.  The serial velocity/acceleration fields are kept
// outside the core because their external unit is not confirmed; they must
// not be converted or used as non-zero algorithm motion inputs in this phase.
struct VisionState
{
  std::int64_t stamp_ns{0};
  std::string frame_id;
  std::uint16_t id{0};
  std::uint16_t mode{0};
  float yaw_rad{0.0F};
  float pitch_rad{0.0F};
  float roll_rad{0.0F};
  std::array<float, 4> quaternion_wxyz{{1.0F, 0.0F, 0.0F, 0.0F}};
  float shoot_speed_mps{0.0F};
  std::uint16_t bullet_count{0};
  std::uint8_t game_progress{0};
};

// ImageFrame is the non-ROS boundary for a camera frame.  bgr_image owns a
// clone of the pixels produced by the ROS adapter, so a detector never keeps a
// reference to a sensor_msgs buffer after the callback returns.
struct ImageFrame
{
  std::int64_t stamp_ns{0};
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  cv::Mat bgr_image;
  float shoot_speed_mps{0.0F};
  std::uint16_t bullet_count{0};
  std::uint8_t game_progress{0};

  bool has_pixels() const noexcept
  {
    return !bgr_image.empty() && bgr_image.type() == CV_8UC3;
  }
};

// This is an adapter boundary, not a claim about camera/gimbal coordinates.
// The source of yaw/pitch must document its frame and sign convention before
// a real detector is connected.
struct Detection
{
  bool valid{false};
  float yaw_rad{0.0F};
  float pitch_rad{0.0F};
  float yaw_vel_rad_s{0.0F};
  float pitch_vel_rad_s{0.0F};
  float yaw_acc_rad_s2{0.0F};
  float pitch_acc_rad_s2{0.0F};
  bool fire_request{false};
};

struct ArmorObservation
{
  Detection detection;
};

struct TargetState
{
  Detection estimate;
};

struct AimCommand
{
  float yaw_rad{0.0F};
  float yaw_vel_rad_s{0.0F};
  float yaw_acc_rad_s2{0.0F};
  float pitch_rad{0.0F};
  float pitch_vel_rad_s{0.0F};
  float pitch_acc_rad_s2{0.0F};
  std::int8_t target_lock{kTargetUnlocked};
  std::int8_t fire_command{kFireNone};
};

struct CoreConfig
{
  bool allow_fire{false};
  std::int8_t burst_command{kFireBurst};
  std::int8_t single_command{kFireSingle};
};

class YoloStage
{
public:
  virtual ~YoloStage() = default;
  virtual std::vector<Detection> detect(const ImageFrame & frame) = 0;
};

class ArmorStage
{
public:
  virtual ~ArmorStage() = default;
  virtual std::vector<ArmorObservation> build(
    const std::vector<Detection> & detections) = 0;
};

class TrackerStage
{
public:
  virtual ~TrackerStage() = default;
  virtual std::optional<TargetState> track(
    const std::vector<ArmorObservation> & armors,
    std::chrono::steady_clock::time_point timestamp) = 0;
};

class TargetStage
{
public:
  virtual ~TargetStage() = default;
  virtual std::optional<Detection> select(const std::optional<TargetState> & target) = 0;
};

class AimerStage
{
public:
  virtual ~AimerStage() = default;
  virtual AimCommand aim(
    const std::optional<Detection> & target,
    const CoreConfig & config) = 0;
};

class AutoAimPipeline
{
public:
  AutoAimPipeline(
    std::unique_ptr<YoloStage> yolo,
    std::unique_ptr<ArmorStage> armor,
    std::unique_ptr<TrackerStage> tracker,
    std::unique_ptr<TargetStage> target,
    std::unique_ptr<AimerStage> aimer,
    CoreConfig config = {});

  AimCommand process(
    const ImageFrame & frame,
    std::chrono::steady_clock::time_point timestamp);

  static AimCommand safe_command() noexcept;

private:
  std::unique_ptr<YoloStage> yolo_;
  std::unique_ptr<ArmorStage> armor_;
  std::unique_ptr<TrackerStage> tracker_;
  std::unique_ptr<TargetStage> target_;
  std::unique_ptr<AimerStage> aimer_;
  CoreConfig config_;
};

// Explicit dry-run detector. It is not a substitute for YOLO and is never
// enabled by default in the ROS node.
class MockYoloStage final : public YoloStage
{
public:
  explicit MockYoloStage(std::optional<Detection> detection);
  std::vector<Detection> detect(const ImageFrame & frame) override;

private:
  std::optional<Detection> detection_;
};

class NullYoloStage final : public YoloStage
{
public:
  std::vector<Detection> detect(const ImageFrame & frame) override;
};

class PassThroughArmorStage final : public ArmorStage
{
public:
  std::vector<ArmorObservation> build(
    const std::vector<Detection> & detections) override;
};

class LatestTargetTracker final : public TrackerStage
{
public:
  std::optional<TargetState> track(
    const std::vector<ArmorObservation> & armors,
    std::chrono::steady_clock::time_point timestamp) override;
};

class FirstTargetStage final : public TargetStage
{
public:
  std::optional<Detection> select(const std::optional<TargetState> & target) override;
};

class CommandAimer final : public AimerStage
{
public:
  AimCommand aim(
    const std::optional<Detection> & target,
    const CoreConfig & config) override;
};

}  // namespace rm_auto_aim::pipeline

#endif  // AUTO_AIM_ROS2__AUTO_AIM_CORE_HPP_
