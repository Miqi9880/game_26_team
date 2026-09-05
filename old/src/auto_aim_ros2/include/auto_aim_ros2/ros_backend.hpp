#ifndef AUTO_AIM_ROS2__ROS_BACKEND_HPP_
#define AUTO_AIM_ROS2__ROS_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/offline_pipeline.hpp"
#include "auto_aim_ros2/pnp_stage.hpp"
#include "auto_aim_ros2/raw_armor_detector.hpp"

namespace rm_auto_aim::ros_backend
{

enum class BackendKind : std::uint8_t
{
  Null = 0,
  Mock,
  OfflineReference,
};

const char * backend_kind_name(BackendKind kind) noexcept;
BackendKind parse_backend_kind(const std::string & name);

struct Config
{
  BackendKind kind{BackendKind::Null};
  bool dry_run{true};
  bool serial_enabled{false};
  bool allow_fire{false};
  bool mock_target{false};
  float mock_yaw_rad{0.0F};
  float mock_pitch_rad{0.0F};
  bool mock_fire_request{false};

  std::string model_path;
  // Versioned detector profile.  OfflineReference requires this field;
  // it is validated before OpenVINO initialization and cannot be inferred
  // from a model filename.
  std::string model_profile_path;
  std::string pnp_config_path;
  std::string device{"CPU"};
  bool allow_test_only{false};
  offline::AimerConfig aimer{};
  offline::TrackerConfig tracker{};
};

struct FrameResult
{
  std::string backend{"null"};
  std::string calibration_profile{"none"};
  std::string model_profile{"none"};
  std::string aimer_mode{"relative_debug"};
  bool test_only{true};
  // This is the lock state measured by the diagnostic tracker/aimer.  It is
  // deliberately separate from command.target_lock: a tracking lock without
  // a validated absolute command must never be forwarded as RobotCtrl lock.
  std::int8_t diagnostic_target_lock{pipeline::kTargetUnlocked};
  bool absolute_command_valid{false};
  bool command_publishable{false};
  std::int64_t stamp_ns{0};
  std::size_t detection_count{0};
  std::size_t valid_pnp_count{0};
  std::vector<pnp::PoseObservation> poses;
  offline::TrackerUpdate tracker_update;
  std::optional<offline::TrackedTarget> selected;
  offline::AimerOutput aimer;
  pipeline::AimCommand command{};
  std::string error;
};

class Backend final
{
public:
  explicit Backend(Config config);
  ~Backend();

  Backend(Backend &&) noexcept;
  Backend & operator=(Backend &&) noexcept;
  Backend(const Backend &) = delete;
  Backend & operator=(const Backend &) = delete;

  FrameResult process(const pipeline::ImageFrame & frame);
  FrameResult safe_result(std::int64_t stamp_ns) const;

  BackendKind kind() const noexcept;
  const std::string & calibration_profile() const noexcept;
  const std::string & model_profile() const noexcept;
  bool test_only() const noexcept;
  const Config & config() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// CSV header/row helpers are shared by AutoAimNode and integration tests.
std::string csv_header();
std::string csv_row(const FrameResult & result);

}  // namespace rm_auto_aim::ros_backend

#endif  // AUTO_AIM_ROS2__ROS_BACKEND_HPP_
