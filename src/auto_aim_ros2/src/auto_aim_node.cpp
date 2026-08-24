#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/angle_units.hpp"
#include "auto_aim_ros2/ros_adapters.hpp"
#include "auto_aim_ros2/ros_backend.hpp"
#include "auto_aim_ros2/ros_image_adapter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "auto_aim_interfaces/msg/vision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace rm_auto_aim
{
namespace
{
void set_error(std::string * error, const std::string & value)
{
  if (error != nullptr) {
    *error = value;
  }
}

bool valid_camera_info(
  const sensor_msgs::msg::CameraInfo & message, std::string * error = nullptr)
{
  if (message.width == 0 || message.height == 0) {
    set_error(error, "CameraInfo width/height must be positive");
    return false;
  }
  for (const auto value : message.k) {
    if (!std::isfinite(value)) {
      set_error(error, "CameraInfo K contains a non-finite value");
      return false;
    }
  }
  for (const auto value : message.d) {
    if (!std::isfinite(value)) {
      set_error(error, "CameraInfo D contains a non-finite value");
      return false;
    }
  }
  if (!(message.k[0] > 0.0) || !(message.k[4] > 0.0) ||
    std::abs(message.k[8]) <= 1e-12)
  {
    set_error(error, "CameraInfo K must have positive fx/fy and non-zero K[8]");
    return false;
  }
  return true;
}

offline::AimerMode parse_aimer_mode(const std::string & value)
{
  if (value == "relative_debug") {
    return offline::AimerMode::RelativeDebug;
  }
  if (value == "test_absolute_zero") {
    return offline::AimerMode::TestAbsoluteZero;
  }
  throw std::invalid_argument(
          "aimer_mode must be relative_debug or test_absolute_zero; got '" + value + "'");
}
}  // namespace

class AutoAimNode final : public rclcpp::Node
{
public:
  explicit AutoAimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("auto_aim", options)
  {
    dry_run_ = declare_parameter<bool>("dry_run", true);
    allow_fire_ = declare_parameter<bool>("allow_fire", false);
    serial_enabled_ = declare_parameter<bool>("serial_enabled", false);
    require_camera_info_ = declare_parameter<bool>("require_camera_info", true);
    backend_name_ = declare_parameter<std::string>("backend", "null");
    mock_target_ = declare_parameter<bool>("mock_target", false);
    mock_yaw_rad_ = declare_parameter<double>("mock_yaw_rad", 0.0);
    mock_pitch_rad_ = declare_parameter<double>("mock_pitch_rad", 0.0);
    mock_fire_request_ = declare_parameter<bool>("mock_fire_request", false);
    output_hz_ = declare_parameter<double>("output_hz", 100.0);
    input_timeout_ms_ = declare_parameter<int>("input_timeout_ms", 100);
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    offline_model_path_ = declare_parameter<std::string>("offline_model_path", "");
    offline_model_profile_ = declare_parameter<std::string>("offline_model_profile", "");
    offline_pnp_config_ = declare_parameter<std::string>("offline_pnp_config", "");
    offline_device_ = declare_parameter<std::string>("offline_device", "CPU");
    allow_test_only_ = declare_parameter<bool>("allow_test_only", false);
    const auto aimer_mode = parse_aimer_mode(
      declare_parameter<std::string>("aimer_mode", "relative_debug"));
    const auto absolute_zero_configured = declare_parameter<bool>(
      "test_absolute_zero_configured", false);
    const auto test_zero_yaw_degree = declare_parameter<double>("test_zero_yaw_degree", 0.0);
    const auto test_zero_pitch_degree = declare_parameter<double>("test_zero_pitch_degree", 0.0);

    if (!std::isfinite(test_zero_yaw_degree) || !std::isfinite(test_zero_pitch_degree)) {
      throw std::invalid_argument("test zero angles must be finite");
    }
    if (output_hz_ <= 0.0 || input_timeout_ms_ <= 0) {
      throw std::invalid_argument("output_hz and input_timeout_ms must be positive");
    }
    const auto period_ms = static_cast<std::int64_t>(
      std::llround(1000.0 / output_hz_));
    if (period_ms <= 0) {
      throw std::invalid_argument("output_hz is too high for a millisecond ROS timer");
    }
    if (serial_enabled_) {
      throw std::invalid_argument(
              "AutoAimNode never opens serial; serial_enabled must remain false");
    }

    const auto backend_kind = ros_backend::parse_backend_kind(backend_name_);
    ros_backend::Config backend_config{};
    backend_config.kind = backend_kind;
    backend_config.dry_run = dry_run_;
    backend_config.serial_enabled = serial_enabled_;
    backend_config.allow_fire = allow_fire_;
    backend_config.mock_target = mock_target_;
    backend_config.mock_yaw_rad = static_cast<float>(mock_yaw_rad_);
    backend_config.mock_pitch_rad = static_cast<float>(mock_pitch_rad_);
    backend_config.mock_fire_request = mock_fire_request_;
    backend_config.model_path = offline_model_path_;
    backend_config.model_profile_path = offline_model_profile_;
    backend_config.pnp_config_path = offline_pnp_config_;
    backend_config.device = offline_device_;
    backend_config.allow_test_only = allow_test_only_;
    backend_config.aimer.mode = aimer_mode;
    backend_config.aimer.test_only = true;
    backend_config.aimer.absolute_zero_configured = absolute_zero_configured;
    backend_config.aimer.yaw_zero_rad = units::degrees_to_radians(
      static_cast<float>(test_zero_yaw_degree));
    backend_config.aimer.pitch_zero_rad = units::degrees_to_radians(
      static_cast<float>(test_zero_pitch_degree));
    backend_config.tracker.min_detect_count = declare_parameter<int>(
      "tracker_min_detect_count", backend_config.tracker.min_detect_count);
    backend_config.tracker.max_temp_lost_ms = declare_parameter<int>(
      "tracker_max_temp_lost_ms", backend_config.tracker.max_temp_lost_ms);
    backend_config.tracker.max_position_jump_m = declare_parameter<double>(
      "tracker_max_position_jump_m", backend_config.tracker.max_position_jump_m);
    backend_config.tracker.max_angle_jump_rad = declare_parameter<double>(
      "tracker_max_angle_jump_rad", backend_config.tracker.max_angle_jump_rad);
    backend_config.tracker.max_velocity_rad_s = declare_parameter<double>(
      "tracker_max_velocity_rad_s", backend_config.tracker.max_velocity_rad_s);

    backend_ = std::make_unique<ros_backend::Backend>(std::move(backend_config));
    latest_command_ = pipeline::AutoAimPipeline::safe_command();
    latest_result_ = backend_->safe_result(0);

    if (!csv_path_.empty()) {
      csv_.open(csv_path_, std::ios::out | std::ios::trunc);
      if (!csv_) {
        throw std::runtime_error("cannot open csv_path: " + csv_path_);
      }
      csv_ << ros_backend::csv_header();
    }

    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::on_image, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS(),
      std::bind(&AutoAimNode::on_camera_info, this, std::placeholders::_1));
    vision_subscription_ = create_subscription<auto_aim_interfaces::msg::Vision>(
      "/Vision_data", 10,
      std::bind(&AutoAimNode::on_vision, this, std::placeholders::_1));
    control_publisher_ = create_publisher<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms), std::bind(&AutoAimNode::publish_control, this));
    RCLCPP_INFO(
      get_logger(),
      "AutoAimNode started: backend=%s calibration=%s test_only=%s dry_run=%s "
      "model_profile=%s serial_enabled=false output_hz=%.1f fire=disabled",
      backend_name_.c_str(), backend_->calibration_profile().c_str(),
      backend_->test_only() ? "true" : "false", dry_run_ ? "true" : "false",
      backend_->model_profile().c_str(), output_hz_);
  }

private:
  using Clock = std::chrono::steady_clock;

  pipeline::AimCommand safe_hold_command(const pipeline::AimCommand & last) const noexcept
  {
    pipeline::AimCommand command{};
    command.yaw_rad = std::isfinite(last.yaw_rad) ? last.yaw_rad : 0.0F;
    command.pitch_rad = std::isfinite(last.pitch_rad) ? last.pitch_rad : 0.0F;
    command.yaw_vel_rad_s = 0.0F;
    command.yaw_acc_rad_s2 = 0.0F;
    command.pitch_vel_rad_s = 0.0F;
    command.pitch_acc_rad_s2 = 0.0F;
    command.target_lock = pipeline::kTargetUnlocked;
    command.fire_command = pipeline::kFireNone;
    return command;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    std::string image_error;
    bool camera_ready = false;
    std::uint32_t camera_width = 0;
    std::uint32_t camera_height = 0;
    float shoot_speed_mps = 0.0F;
    std::uint16_t bullet_count = 0;
    std::uint8_t game_progress = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      camera_ready = have_camera_info_;
      camera_width = camera_info_width_;
      camera_height = camera_info_height_;
      if (have_vision_) {
        shoot_speed_mps = latest_vision_.shoot_speed_mps;
        bullet_count = latest_vision_.bullet_count;
        game_progress = latest_vision_.game_progress;
      }
    }

    const auto frame = ros_adapters::to_image_frame(
      *message, shoot_speed_mps, bullet_count, game_progress, &image_error);
    if (!frame.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring invalid image frame: %s", image_error.c_str());
      return;
    }
    if (require_camera_info_ &&
      (!camera_ready || camera_width != frame->width || camera_height != frame->height))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring image until valid matching CameraInfo is received (image=%ux%u info=%ux%u)",
        frame->width, frame->height, camera_width, camera_height);
      return;
    }

    auto result = backend_->process(*frame);
    if (!result.error.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "Backend failed closed: %s", result.error.c_str());
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      latest_result_ = result;
      latest_command_ = result.command;
      last_image_time_ = Clock::now();
      have_image_ = true;
    }
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
  {
    std::string error;
    const bool valid = valid_camera_info(*message, &error);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_camera_info_ = valid;
      camera_info_width_ = valid ? message->width : 0;
      camera_info_height_ = valid ? message->height : 0;
    }
    if (!valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring invalid CameraInfo: %s", error.c_str());
    }
  }

  void on_vision(const auto_aim_interfaces::msg::Vision::ConstSharedPtr & message)
  {
    const auto adapted = ros_adapters::to_algorithm_vision(*message);
    if (!adapted.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring Vision_data with non-finite angle, velocity, speed, or quaternion field");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    latest_vision_ = *adapted;
    have_vision_ = true;
  }

  void publish_control()
  {
    pipeline::AimCommand command{};
    ros_backend::FrameResult row;
    bool stale = true;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (have_image_ &&
        Clock::now() - last_image_time_ < std::chrono::milliseconds(input_timeout_ms_))
      {
        command = latest_command_;
        row = latest_result_;
        stale = false;
      } else {
        command = safe_hold_command(latest_command_);
        row = backend_->safe_result(0);
      }
    }

    if (stale) {
      row.command = command;
      row.diagnostic_target_lock = pipeline::kTargetUnlocked;
      row.absolute_command_valid = false;
      row.command_publishable = false;
      row.error = "input_timeout";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No fresh image for %d ms; publishing safe hold command", input_timeout_ms_);
    }
    if (backend_->kind() == ros_backend::BackendKind::OfflineReference &&
      !row.command_publishable)
    {
      // OfflineReference is diagnostic/test-only in this phase.  Even a
      // test_absolute_zero candidate must not become a RobotCtrl command.
      command = pipeline::AutoAimPipeline::safe_command();
      row.command = command;
    }
    // This node is a dry-run/control-topic producer only.  Keep the fire
    // inhibit unconditional even if a caller supplies contradictory params.
    command.fire_command = pipeline::kFireNone;
    row.command = command;
    const auto ros_message = ros_adapters::to_ros(command);
    control_publisher_->publish(ros_message);
    if (csv_) {
      csv_ << ros_backend::csv_row(row);
      csv_.flush();
    }
  }

  bool dry_run_{true};
  bool allow_fire_{false};
  bool serial_enabled_{false};
  bool require_camera_info_{true};
  bool mock_target_{false};
  bool mock_fire_request_{false};
  bool have_image_{false};
  bool have_camera_info_{false};
  bool have_vision_{false};
  std::uint32_t camera_info_width_{0};
  std::uint32_t camera_info_height_{0};
  double mock_yaw_rad_{0.0};
  double mock_pitch_rad_{0.0};
  double output_hz_{100.0};
  int input_timeout_ms_{100};
  bool allow_test_only_{false};
  std::string backend_name_;
  std::string csv_path_;
  std::string offline_model_path_;
  std::string offline_model_profile_;
  std::string offline_pnp_config_;
  std::string offline_device_;
  pipeline::VisionState latest_vision_{};
  std::ofstream csv_;
  std::mutex mutex_;
  Clock::time_point last_image_time_{};
  pipeline::AimCommand latest_command_{};
  ros_backend::FrameResult latest_result_{};
  std::unique_ptr<ros_backend::Backend> backend_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr control_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_subscription_;
};
}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::AutoAimNode)
