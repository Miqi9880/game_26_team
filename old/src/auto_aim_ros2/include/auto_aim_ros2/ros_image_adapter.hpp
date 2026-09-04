#ifndef AUTO_AIM_ROS2__ROS_IMAGE_ADAPTER_HPP_
#define AUTO_AIM_ROS2__ROS_IMAGE_ADAPTER_HPP_

#include <cstdint>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "auto_aim_interfaces/msg/vision.hpp"
#include "auto_aim_ros2/auto_aim_core.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace rm_auto_aim::ros_adapters
{

// Convert supported ROS encodings into an owned BGR CV_8UC3 image.  The
// returned Mat is a clone and does not alias the ROS message buffer.
std::optional<cv::Mat> to_bgr_image(
  const sensor_msgs::msg::Image & message,
  std::string * error = nullptr);

// Build the non-ROS ImageFrame boundary and copy all camera bookkeeping from
// the message.  Vision fields are supplied separately by the caller.
std::optional<pipeline::ImageFrame> to_image_frame(
  const sensor_msgs::msg::Image & message,
  float shoot_speed_mps,
  std::uint16_t bullet_count,
  std::uint8_t game_progress,
  std::string * error = nullptr);

}  // namespace rm_auto_aim::ros_adapters

#endif  // AUTO_AIM_ROS2__ROS_IMAGE_ADAPTER_HPP_
