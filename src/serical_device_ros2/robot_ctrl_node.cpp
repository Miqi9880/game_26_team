#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{
/// Downlink: subscribe /Robot_ctrl_data, pack protocol_new RobotCtrlData, write serial.
class RobotCtrlSub : public rclcpp::Node
{
public:
  explicit RobotCtrlSub(const rclcpp::NodeOptions & options)
  : Node("robot_ctrl", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    const std::string dev = this->declare_parameter<std::string>("serial_port", "/dev/robomaster");
    const int baud = this->declare_parameter<int>("serial_baud", 921600);
    if (!SerialMain::instance().init(dev, baud)) {
      RCLCPP_WARN(this->get_logger(), "serial init failed (%s @ %d); TX disabled", dev.c_str(), baud);
    }
    sub_ = this->create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      "Robot_ctrl_data", 10,
      [this](auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr m) {
        RobotCtrlData d;
        d.yaw = m->yaw;
        d.yaw_vel = m->yaw_vel;
        d.yaw_acc = m->yaw_acc;
        d.pitch = m->pitch;
        d.pitch_vel = m->pitch_vel;
        d.pitch_acc = m->pitch_acc;
        d.target_lock = m->target_lock;
        d.fire_command = m->fire_command;
        if (d.fire_command != 0) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "robot_ctrl TX fire_command=%d", static_cast<int>(d.fire_command));
        }
        SerialMain::instance().send_robot_ctrl(d);
      });
    RCLCPP_INFO(this->get_logger(), "RobotCtrlSub started (dev=%s baud=%d)", dev.c_str(), baud);
  }

private:
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr sub_;
};
}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::RobotCtrlSub)