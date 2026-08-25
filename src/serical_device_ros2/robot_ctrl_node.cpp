#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include "robot_ctrl_safety.hpp"
#include "serial_main.h"
#include "serial_ros_mapping.hpp"
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"

namespace rm_auto_aim
{
class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_ctrl", options), serial()
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    const auto serial_device = this->declare_parameter<std::string>(
      "serial_device", "/dev/robomaster");
    const auto serial_requested = this->declare_parameter<bool>("serial_enabled", false);
    dry_run_ = this->declare_parameter<bool>("dry_run", true);
    const auto allow_fire_requested = this->declare_parameter<bool>("allow_fire", false);
    allow_fire_ = false;
    if (allow_fire_requested) {
      RCLCPP_WARN(
        this->get_logger(),
        "allow_fire was requested but is disabled until fire timing semantics are confirmed");
    }
    fire_burst_command_ = this->declare_parameter<int>("fire_burst_command", 1);
    fire_single_command_ = this->declare_parameter<int>("fire_single_command", 2);
    if (fire_burst_command_ <= 0 || fire_burst_command_ > 127 ||
      fire_single_command_ <= 0 || fire_single_command_ > 127 ||
      fire_burst_command_ == fire_single_command_)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid fire command configuration; disabling fire (burst=%d, single=%d)",
        fire_burst_command_, fire_single_command_);
      allow_fire_ = false;
    }

    const auto vehicle_profile_name = this->declare_parameter<std::string>(
      "vehicle_profile", "unselected");
    const auto parsed_profile = auto_aim_interfaces::control::parse_vehicle_profile(
      vehicle_profile_name);
    if (!parsed_profile.has_value()) {
      throw std::invalid_argument(
              "vehicle_profile must be unselected, new_turtle, or dog_leg");
    }
    vehicle_profile_ = *parsed_profile;

    output_hz_ = this->declare_parameter<double>("output_hz", 100.0);
    input_timeout_ms_ = this->declare_parameter<int>("input_timeout_ms", 100);
    const auto control_period = safety::control_period_from_hz(output_hz_);
    if (!control_period.has_value() || input_timeout_ms_ <= 0) {
      throw std::invalid_argument(
              "output_hz must produce a positive timer period and input_timeout_ms must be positive");
    }
    const auto input_timeout_ns = static_cast<std::int64_t>(input_timeout_ms_) * 1'000'000LL;

    serial.SetDevicePath(serial_device);
    safety_ = std::make_unique<safety::RobotCtrlSafety>(
      safety::Config{
        allow_fire_, static_cast<std::int8_t>(fire_burst_command_),
        static_cast<std::int8_t>(fire_single_command_), vehicle_profile_, input_timeout_ns});

    const auto serial_enabled = serial_requested && !dry_run_;
    if (serial_requested && dry_run_) {
      RCLCPP_WARN(
        this->get_logger(),
        "serial_enabled was requested but dry_run=true; keeping the serial device closed");
    }
    if (serial_enabled && !serial.Enable())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial device '%s'", serial_device.c_str());
    }
    else if (!serial_enabled)
    {
      RCLCPP_INFO(
        this->get_logger(), "Serial disabled; RobotCtrlSub will not write a device");
    }

    subscription_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10,
      std::bind(&RobotCtrlSub::robotCtrlReceive, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      *control_period, std::bind(&RobotCtrlSub::sendControl, this));

    RCLCPP_INFO(
      this->get_logger(),
      "-- RobotCtrlSub started: profile=%s output_hz=%.2f timeout_ms=%d dry_run=%s "
      "serial_enabled=%s fire=disabled --",
      auto_aim_interfaces::control::vehicle_profile_name(vehicle_profile_), output_hz_,
      input_timeout_ms_, dry_run_ ? "true" : "false", serial_enabled ? "true" : "false");
  }

private:
  void robotCtrlReceive(const auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr & msg)
  {
    // RobotCtrl.msg and RobotCtrlData use degree for absolute position angles.
    // This node is a raw ROS-to-serial bridge and intentionally does not
    // perform any unit conversion; ROS adapters must publish degree values.
    const auto control = serial_ros::to_serial_robot_ctrl(*msg);

    if (!safety_->Accept(control)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Rejected invalid RobotCtrl input; safe hold/unlock/fire=0 was applied");
    }
  }

  void sendControl()
  {
    const auto output = safety_->Tick();
    if (!output.fresh)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No fresh valid RobotCtrl input for %d ms; sending safe hold command",
        input_timeout_ms_);
    }
    if (output.yaw_wrapped || output.pitch_clamped) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Applied RobotCtrl position constraints (yaw_wrapped=%s pitch_clamped=%s)",
        output.yaw_wrapped ? "true" : "false", output.pitch_clamped ? "true" : "false");
    }

    serial.SenderMain(output.control);
  }

  SerialMain serial;
  std::unique_ptr<safety::RobotCtrlSafety> safety_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
  bool allow_fire_{false};
  bool dry_run_{true};
  double output_hz_{100.0};
  int input_timeout_ms_{100};
  auto_aim_interfaces::control::VehicleProfile vehicle_profile_{
    auto_aim_interfaces::control::VehicleProfile::Unselected};
  int fire_burst_command_{1};
  int fire_single_command_{2};
};

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
