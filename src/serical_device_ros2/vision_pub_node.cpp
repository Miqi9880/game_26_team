#include <rclcpp/rclcpp.hpp>
#include "serial_main.h"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace rm_auto_aim
{
/// Uplink: dedicated RX thread parses serial; this node publishes /Vision_data
/// at most once per newly parsed VISION frame (500 Hz uplink).
class VisionPub : public rclcpp::Node
{
public:
  explicit VisionPub(const rclcpp::NodeOptions & options)
  : Node("vision_pub", options)
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    const std::string dev = this->declare_parameter<std::string>("serial_port", "/dev/robomaster");
    const int baud = this->declare_parameter<int>("serial_baud", 921600);
    if (!SerialMain::instance().init(dev, baud)) {
      RCLCPP_WARN(this->get_logger(), "serial init failed (%s @ %d); RX disabled", dev.c_str(), baud);
    }
    pub_ = this->create_publisher<auto_aim_interfaces::msg::Vision>("/Vision_data", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(2),
      [this]() { poll(); });
    RCLCPP_INFO(this->get_logger(), "VisionPub started (dev=%s baud=%d)", dev.c_str(), baud);
  }

private:
  void poll()
  {
    const uint64_t seq = SerialMain::instance().vision_seq();
    if (seq == last_seq_) { return; }
    last_seq_ = seq;
    VisionData v;
    if (!SerialMain::instance().get_vision(v)) { return; }
    auto msg = std::make_shared<auto_aim_interfaces::msg::Vision>();
    msg->header.frame_id = "vision";
    msg->header.stamp = this->now();
    msg->id = v.id;
    msg->mode = v.mode;
    msg->yaw = v.yaw;
    msg->yaw_vel = v.yaw_vel;
    msg->pitch = v.pitch;
    msg->pitch_vel = v.pitch_vel;
    msg->roll = v.roll;
    msg->quaternion[0] = v.quaternion[0];
    msg->quaternion[1] = v.quaternion[1];
    msg->quaternion[2] = v.quaternion[2];
    msg->quaternion[3] = v.quaternion[3];
    msg->shoot_speed = v.shoot_speed;
    msg->bullet_count = v.bullet_count;
    msg->game_progress = v.game_progress;
    pub_->publish(*msg);
  }

  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint64_t last_seq_ = 0;
};
}  // namespace rm_auto_aim

RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::VisionPub)