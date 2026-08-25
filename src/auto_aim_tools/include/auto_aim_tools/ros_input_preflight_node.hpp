#ifndef AUTO_AIM_TOOLS__ROS_INPUT_PREFLIGHT_NODE_HPP_
#define AUTO_AIM_TOOLS__ROS_INPUT_PREFLIGHT_NODE_HPP_

#include "auto_aim_tools/preflight_analyzer.hpp"

#include <memory>

#include "auto_aim_interfaces/msg/vision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace auto_aim_tools
{

class RosInputPreflightNode final : public rclcpp::Node
{
public:
  explicit RosInputPreflightNode(
    std::shared_ptr<PreflightAnalyzer> analyzer,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr & message);
  void on_camera_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message);
  void on_vision(
    const auto_aim_interfaces::msg::Vision::ConstSharedPtr & message);

  std::shared_ptr<PreflightAnalyzer> analyzer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Vision>::SharedPtr vision_subscription_;
};

}  // namespace auto_aim_tools

#endif  // AUTO_AIM_TOOLS__ROS_INPUT_PREFLIGHT_NODE_HPP_
