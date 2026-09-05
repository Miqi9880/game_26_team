#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
#include "serial_ros_mapping.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{
class VisionPub : public rclcpp::Node
{
public:
  explicit VisionPub(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vision_pub", options), serial()
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    const auto serial_device = this->declare_parameter<std::string>(
      "serial_device", "/dev/robomaster");
    const auto serial_enabled = this->declare_parameter<bool>("serial_enabled", false);
    serial.SetDevicePath(serial_device);
    if (serial_enabled && !serial.Enable())
    {
      RCLCPP_ERROR(this->get_logger(), "Unable to open serial device '%s'", serial_device.c_str());
    }
    else if (!serial_enabled)
    {
      RCLCPP_INFO(this->get_logger(), "Serial disabled; VisionPub is running in dry-run mode");
    }

    publisher_ = this->create_publisher<auto_aim_interfaces::msg::Vision>("/Vision_data", 10);

    RCLCPP_INFO(this->get_logger(), "--- VisionPub Node Started ---");

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1),
      std::bind(&VisionPub::timer_callback, this));
  }

private:
  bool Get_data{false};
  SerialMain serial;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback()
  {
    if (!serial.IsEnabled())
    {
      return;
    }

    Get_data = serial.ReceiverMain();
    if (Get_data)
    {
      auto vision_msg = std::make_shared<auto_aim_interfaces::msg::Vision>(
        serial_ros::to_ros_vision(serial.vision_msg_));

      vision_msg->header.frame_id = "vision";
      vision_msg->header.stamp = this->now();
      // VisionData and Vision.msg both carry position angles in degree.  The
      // serial bridge deliberately performs no unit conversion; the
      // auto_aim_ros2 adapter converts degree to internal radians once.

      publisher_->publish(*vision_msg);
    }
  }
};

}  // namespace rm_auto_aim
// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<rm_auto_aim::VisionPub>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::VisionPub)
