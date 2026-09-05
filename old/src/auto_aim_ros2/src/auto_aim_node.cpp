#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/angle_units.hpp"
#include "auto_aim_ros2/ros_adapters.hpp"
#include "auto_aim_ros2/ros_backend.hpp"
#include "auto_aim_ros2/ros_image_adapter.hpp"
#include "auto_aim_ros2/vision_time_alignment.hpp"
#include "control_interface_constraints.hpp"

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
    const auto allow_fire_requested = declare_parameter<bool>("allow_fire", false);
    // There is no reviewed production fire profile or confirmed MCU fire
    // timing contract in this repository.  Reject the request at startup
    // rather than treating allow_fire as an advisory dry-run flag.
    if (allow_fire_requested) {
      throw std::invalid_argument(
              "allow_fire requires a reviewed production fire authorization; "
              "it must remain false in the current control interface");
    }
    allow_fire_ = false;
    serial_enabled_ = declare_parameter<bool>("serial_enabled", false);
    require_camera_info_ = declare_parameter<bool>("require_camera_info", true);
    backend_name_ = declare_parameter<std::string>("backend", "null");
    mock_target_ = declare_parameter<bool>("mock_target", false);
    mock_yaw_rad_ = declare_parameter<double>("mock_yaw_rad", 0.0);
    mock_pitch_rad_ = declare_parameter<double>("mock_pitch_rad", 0.0);
    mock_fire_request_ = declare_parameter<bool>("mock_fire_request", false);
    const auto vehicle_profile_name = declare_parameter<std::string>(
      "vehicle_profile", "unselected");
    const auto parsed_vehicle_profile = auto_aim_interfaces::control::parse_vehicle_profile(
      vehicle_profile_name);
    if (!parsed_vehicle_profile.has_value()) {
      throw std::invalid_argument(
              "vehicle_profile must be unselected, new_turtle, or dog_leg");
    }
    vehicle_profile_ = *parsed_vehicle_profile;
    output_hz_ = declare_parameter<double>("output_hz", 100.0);
    input_timeout_ms_ = declare_parameter<int>("input_timeout_ms", 100);
    csv_path_ = declare_parameter<std::string>("csv_path", "");
    vision_time_alignment_csv_path_ = declare_parameter<std::string>(
      "vision_time_alignment_csv_path", "");
    offline_model_path_ = declare_parameter<std::string>("offline_model_path", "");
    offline_model_profile_ = declare_parameter<std::string>("offline_model_profile", "");
    offline_pnp_config_ = declare_parameter<std::string>("offline_pnp_config", "");
    offline_device_ = declare_parameter<std::string>("offline_device", "CPU");
    allow_test_only_ = declare_parameter<bool>("allow_test_only", false);
    const auto vision_history_capacity = declare_parameter<int>(
      "vision_history_capacity", 32);
    vision_time_alignment_tolerance_ns_ = declare_parameter<std::int64_t>(
      "vision_time_alignment_tolerance_ns", -1);
    vision_time_alignment_allow_future_ = declare_parameter<bool>(
      "vision_time_alignment_allow_future", false);
    vision_time_alignment_assume_shared_ros_clock_ = declare_parameter<bool>(
      "vision_time_alignment_assume_shared_ros_clock", false);
    if (vision_history_capacity <= 0) {
      throw std::invalid_argument("vision_history_capacity must be positive");
    }
    if (vision_time_alignment_tolerance_ns_ < -1) {
      throw std::invalid_argument(
              "vision_time_alignment_tolerance_ns must be -1 (unconfigured) or non-negative");
    }
    vision_timestamp_domain_ = vision_time_alignment_assume_shared_ros_clock_ ?
      vision_time_alignment::TimestampDomain::SharedRosHeader :
      vision_time_alignment::TimestampDomain::VisionHeader;
    image_timestamp_domain_ = vision_time_alignment_assume_shared_ros_clock_ ?
      vision_time_alignment::TimestampDomain::SharedRosHeader :
      vision_time_alignment::TimestampDomain::ImageHeader;
    vision_history_ = vision_time_alignment::VisionStateHistory(
      static_cast<std::size_t>(vision_history_capacity), vision_timestamp_domain_);
    const auto aimer_mode = parse_aimer_mode(
      declare_parameter<std::string>("aimer_mode", "relative_debug"));
    const auto absolute_zero_configured = declare_parameter<bool>(
      "test_absolute_zero_configured", false);
    const auto test_zero_yaw_degree = declare_parameter<double>("test_zero_yaw_degree", 0.0);
    const auto test_zero_pitch_degree = declare_parameter<double>("test_zero_pitch_degree", 0.0);

    if (!std::isfinite(test_zero_yaw_degree) || !std::isfinite(test_zero_pitch_degree)) {
      throw std::invalid_argument("test zero angles must be finite");
    }
    const auto control_period = auto_aim_interfaces::control::control_period_from_hz(output_hz_);
    if (!control_period.has_value() || input_timeout_ms_ <= 0) {
      throw std::invalid_argument(
              "output_hz must produce a positive nanosecond timer period and "
              "input_timeout_ms must be positive");
    }
    if (serial_enabled_) {
      throw std::invalid_argument(
              "AutoAimNode never opens serial; serial_enabled must remain false");
    }
    if (!csv_path_.empty() && !vision_time_alignment_csv_path_.empty() &&
      csv_path_ == vision_time_alignment_csv_path_)
    {
      throw std::invalid_argument(
              "csv_path and vision_time_alignment_csv_path must be different files");
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
    if (!vision_time_alignment_csv_path_.empty()) {
      vision_alignment_csv_.open(
        vision_time_alignment_csv_path_, std::ios::out | std::ios::trunc);
      if (!vision_alignment_csv_) {
        throw std::runtime_error(
                "cannot open vision_time_alignment_csv_path: " +
                vision_time_alignment_csv_path_);
      }
      vision_alignment_csv_
        << "image_header_stamp_ns,latest_received_vision_header_ns,"
        << "latest_history_vision_header_ns,matched_stamp_ns,delta_ns,status,"
        << "image_timestamp_domain,vision_timestamp_domain,"
        << "image_receive_steady_ns,vision_receive_steady_ns\n";
      vision_alignment_csv_.flush();
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
      *control_period, std::bind(&AutoAimNode::publish_control, this));
    RCLCPP_INFO(
      get_logger(),
      "AutoAimNode started: backend=%s calibration=%s test_only=%s dry_run=%s "
      "model_profile=%s vehicle_profile=%s serial_enabled=false output_hz=%.1f fire=disabled",
      backend_name_.c_str(), backend_->calibration_profile().c_str(),
      backend_->test_only() ? "true" : "false", dry_run_ ? "true" : "false",
      backend_->model_profile().c_str(),
      auto_aim_interfaces::control::vehicle_profile_name(vehicle_profile_), output_hz_);
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

  static std::int64_t steady_stamp_ns(const Clock::time_point time) noexcept
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      time.time_since_epoch()).count();
  }

  vision_time_alignment::PairConfig vision_pair_config() const noexcept
  {
    vision_time_alignment::PairConfig config{};
    if (vision_time_alignment_tolerance_ns_ >= 0) {
      config.tolerance_ns = vision_time_alignment_tolerance_ns_;
    }
    config.allow_future = vision_time_alignment_allow_future_;
    return config;
  }

  void report_alignment(
    const vision_time_alignment::PairResult & result,
    std::int64_t image_receive_steady_ns,
    std::int64_t latest_received_vision_header_ns,
    std::int64_t latest_history_vision_header_ns,
    std::int64_t vision_receive_steady_ns)
  {
    // Header stamps are deliberately kept separate from steady-clock receive
    // stamps.  The ROS headers currently have no proven common clock domain,
    // so this is a diagnostic boundary and never a control input.
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Vision/image time alignment status=%s image_header_ns=%lld "
      "matched_stamp_ns=%lld latest_history_vision_header_ns=%lld "
      "latest_received_vision_header_ns=%lld image_receive_steady_ns=%lld "
      "vision_receive_steady_ns=%lld delta_ns=%lld tolerance_ns=%lld",
      vision_time_alignment::pair_status_name(result.status),
      static_cast<long long>(result.image_stamp_ns),
      static_cast<long long>(result.matched_stamp_ns),
      static_cast<long long>(latest_history_vision_header_ns),
      static_cast<long long>(latest_received_vision_header_ns),
      static_cast<long long>(image_receive_steady_ns),
      static_cast<long long>(vision_receive_steady_ns),
      static_cast<long long>(result.delta_ns),
      static_cast<long long>(vision_time_alignment_tolerance_ns_));

    std::lock_guard<std::mutex> lock(mutex_);
    if (vision_alignment_csv_) {
      vision_alignment_csv_
        << result.image_stamp_ns << ','
        << latest_received_vision_header_ns << ','
        << latest_history_vision_header_ns << ','
        << result.matched_stamp_ns << ','
        << result.delta_ns << ','
        << vision_time_alignment::pair_status_name(result.status) << ','
        << vision_time_alignment::timestamp_domain_name(image_timestamp_domain_) << ','
        << vision_time_alignment::timestamp_domain_name(vision_timestamp_domain_) << ','
        << image_receive_steady_ns << ','
        << vision_receive_steady_ns << '\n';
      vision_alignment_csv_.flush();
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & message)
  {
    const auto image_receive_time = Clock::now();
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
      // Preserve the pre-existing bookkeeping path for shoot speed and
      // replay fields.  Position angles, quaternion, and timestamp pairing
      // remain diagnostic-only and are never copied into the control chain.
      if (have_vision_bookkeeping_) {
        shoot_speed_mps = latest_shoot_speed_mps_;
        bullet_count = latest_bullet_count_;
        game_progress = latest_game_progress_;
      }
    }

    // Vision pose/time state is intentionally not copied into ImageFrame.
    // Until the image and Vision header clock domains are proven equivalent,
    // that state is diagnostic-only and cannot influence detector, tracker,
    // aimer, or RobotCtrl output.  The existing shoot-speed/replay bookkeeping
    // fields above are kept for offline compatibility and are not pose data.
    const auto frame = ros_adapters::to_image_frame(
      *message, shoot_speed_mps, bullet_count, game_progress, &image_error);
    if (!frame.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring invalid image frame: %s", image_error.c_str());
      return;
    }

    vision_time_alignment::PairResult alignment;
    std::int64_t latest_received_vision_header_ns = 0;
    std::int64_t latest_history_vision_header_ns = 0;
    std::int64_t vision_receive_steady_ns = 0;
    const auto image_receive_steady_ns = steady_stamp_ns(image_receive_time);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      alignment = vision_history_.pair(
        frame->stamp_ns,
        image_timestamp_domain_,
        vision_pair_config());
      latest_received_vision_header_ns = last_vision_header_stamp_ns_;
      if (const auto * latest_sample = vision_history_.latest(); latest_sample != nullptr) {
        latest_history_vision_header_ns = latest_sample->state.stamp_ns;
      }
      vision_receive_steady_ns = last_vision_receive_steady_ns_;
      last_image_header_stamp_ns_ = frame->stamp_ns;
      last_image_receive_steady_ns_ = image_receive_steady_ns;
    }
    report_alignment(
      alignment, image_receive_steady_ns, latest_received_vision_header_ns,
      latest_history_vision_header_ns, vision_receive_steady_ns);

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
    const auto vision_receive_time = Clock::now();
    const auto & raw_stamp = message->header.stamp;
    const auto raw_stamp_is_canonical = raw_stamp.sec >= 0 &&
      raw_stamp.nanosec < 1'000'000'000U;
    const auto raw_vision_header_stamp_ns = raw_stamp_is_canonical ?
      static_cast<std::int64_t>(raw_stamp.sec) * 1'000'000'000LL +
      static_cast<std::int64_t>(raw_stamp.nanosec) : 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Keep the most recently received raw Header timestamp for diagnostics,
      // including samples rejected by the adapter.  Invalid ROS time is
      // represented as zero and is never inserted into the history.
      last_vision_header_stamp_ns_ = raw_vision_header_stamp_ns;
      last_vision_receive_steady_ns_ = steady_stamp_ns(vision_receive_time);
    }
    const auto adapted = ros_adapters::to_algorithm_vision(*message);
    if (!adapted.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring Vision_data with non-finite angle, velocity, speed, or quaternion field");
      return;
    }
    vision_time_alignment::InsertResult insert_result;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // Keep the ROS Header timestamp as VisionHeader.  The receive time is
      // recorded independently on the local steady clock and is never mixed
      // into header-stamp pairing.
      insert_result = vision_history_.insert(
        *adapted, vision_timestamp_domain_);
      if (insert_result.accepted()) {
        // Only accepted samples may update the existing replay bookkeeping.
        // Pose/quaternion/timestamp data never cross into ImageFrame.
        latest_shoot_speed_mps_ = adapted->shoot_speed_mps;
        latest_bullet_count_ = adapted->bullet_count;
        latest_game_progress_ = adapted->game_progress;
        have_vision_bookkeeping_ = true;
      }
    }
    if (!insert_result.accepted()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Ignoring Vision_data history sample: status=%s reason=%s size=%zu",
        vision_time_alignment::insert_status_name(insert_result.status),
        vision_time_alignment::insert_reason_name(insert_result.reason),
        insert_result.size);
    }
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
    const auto adapter_result = ros_adapters::to_ros_with_profile(command, vehicle_profile_);
    control_publisher_->publish(adapter_result.message);
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
  bool have_vision_bookkeeping_{false};
  std::uint32_t camera_info_width_{0};
  std::uint32_t camera_info_height_{0};
  double mock_yaw_rad_{0.0};
  double mock_pitch_rad_{0.0};
  double output_hz_{100.0};
  int input_timeout_ms_{100};
  bool allow_test_only_{false};
  auto_aim_interfaces::control::VehicleProfile vehicle_profile_{
    auto_aim_interfaces::control::VehicleProfile::Unselected};
  bool vision_time_alignment_allow_future_{false};
  bool vision_time_alignment_assume_shared_ros_clock_{false};
  float latest_shoot_speed_mps_{0.0F};
  std::uint16_t latest_bullet_count_{0};
  std::uint8_t latest_game_progress_{0};
  std::string backend_name_;
  std::string csv_path_;
  std::string vision_time_alignment_csv_path_;
  std::string offline_model_path_;
  std::string offline_model_profile_;
  std::string offline_pnp_config_;
  std::string offline_device_;
  std::int64_t vision_time_alignment_tolerance_ns_{-1};
  std::int64_t last_vision_header_stamp_ns_{0};
  std::int64_t last_vision_receive_steady_ns_{0};
  std::int64_t last_image_header_stamp_ns_{0};
  std::int64_t last_image_receive_steady_ns_{0};
  vision_time_alignment::TimestampDomain vision_timestamp_domain_{
    vision_time_alignment::TimestampDomain::VisionHeader};
  vision_time_alignment::TimestampDomain image_timestamp_domain_{
    vision_time_alignment::TimestampDomain::ImageHeader};
  vision_time_alignment::VisionStateHistory vision_history_{};
  std::ofstream csv_;
  std::ofstream vision_alignment_csv_;
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
