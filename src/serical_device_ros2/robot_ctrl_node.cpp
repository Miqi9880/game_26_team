#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "robot_ctrl_safety.hpp"
#include "serial_main.h"
#include "serial_ros_mapping.hpp"
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"

namespace rm_auto_aim
{
namespace
{
constexpr auto kControlPeriod = std::chrono::milliseconds(10);  // 100 Hz
}  // namespace

class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("robot_ctrl", options), serial()
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    const auto serial_device = this->declare_parameter<std::string>(
      "serial_device", "/dev/robomaster");
    const auto serial_enabled = this->declare_parameter<bool>("serial_enabled", false);
    allow_fire_ = this->declare_parameter<bool>("allow_fire", false);
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

    serial.SetDevicePath(serial_device);
    safety_ = std::make_unique<safety::RobotCtrlSafety>(
      safety::Config{
        allow_fire_, static_cast<std::int8_t>(fire_burst_command_),
        static_cast<std::int8_t>(fire_single_command_)});

    if (serial_enabled && !serial.Enable())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial device '%s'", serial_device.c_str());
    }
    else if (!serial_enabled)
    {
      RCLCPP_INFO(this->get_logger(), "Serial disabled; RobotCtrlSub is running in dry-run mode");
    }

    subscription_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "/Robot_ctrl_data", 10,
      std::bind(&RobotCtrlSub::robotCtrlReceive, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      kControlPeriod, std::bind(&RobotCtrlSub::sendControl, this));

    RCLCPP_INFO(this->get_logger(), "-- RobotCtrlSub Node Started (100 Hz output) --");
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
        "Ignoring invalid RobotCtrl input; safe timeout state is unchanged");
    }
  }

  void sendControl()
  {
    const auto output = safety_->Tick();
    if (!output.fresh)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "No fresh valid RobotCtrl input for 100 ms; sending safe hold command");
    }

    serial.SenderMain(output.control);
  }

  SerialMain serial;
  std::unique_ptr<safety::RobotCtrlSafety> safety_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr subscription_;
  bool allow_fire_{false};
  int fire_burst_command_{1};
  int fire_single_command_{2};
};

}  // namespace rm_auto_aim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)
