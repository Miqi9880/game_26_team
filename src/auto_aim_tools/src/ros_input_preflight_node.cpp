#include "auto_aim_tools/ros_input_preflight_node.hpp"

#include <exception>
#include <functional>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace auto_aim_tools
{
namespace
{
template<typename Message, typename = void>
struct HasYawAcceleration : std::false_type {};

template<typename Message>
struct HasYawAcceleration<
  Message, std::void_t<decltype(std::declval<Message>().yaw_acc)>>: std::true_type {};

template<typename Message, typename = void>
struct HasPitchAcceleration : std::false_type {};

template<typename Message>
struct HasPitchAcceleration<
  Message, std::void_t<decltype(std::declval<Message>().pitch_acc)>>: std::true_type {};

template<typename Message>
std::optional<float> yaw_acceleration(const Message & message)
{
  if constexpr (HasYawAcceleration<Message>::value) {
    return static_cast<float>(message.yaw_acc);
  }
  return std::nullopt;
}

template<typename Message>
std::optional<float> pitch_acceleration(const Message & message)
{
  if constexpr (HasPitchAcceleration<Message>::value) {
    return static_cast<float>(message.pitch_acc);
  }
  return std::nullopt;
}

HeaderStamp to_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return HeaderStamp{stamp.sec, stamp.nanosec, true};
}

}  // namespace

RosInputPreflightNode::RosInputPreflightNode(
  std::shared_ptr<PreflightAnalyzer> analyzer, const rclcpp::NodeOptions & options)
: Node("ros_input_preflight", options), analyzer_(std::move(analyzer))
{
  if (!analyzer_) {
    throw std::invalid_argument("analyzer must not be null");
  }
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    kImageTopic, rclcpp::SensorDataQoS(),
    std::bind(&RosInputPreflightNode::on_image, this, std::placeholders::_1));
  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, rclcpp::SensorDataQoS(),
    std::bind(&RosInputPreflightNode::on_camera_info, this, std::placeholders::_1));
  vision_subscription_ = create_subscription<auto_aim_interfaces::msg::Vision>(
    kVisionTopic, rclcpp::SensorDataQoS(),
    std::bind(&RosInputPreflightNode::on_vision, this, std::placeholders::_1));
}

void RosInputPreflightNode::on_image(
  const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  try {
    analyzer_->observe_image(
      ImageSample{
        to_stamp(message->header.stamp), message->width, message->height,
        message->encoding, message->step, message->data.size(),
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    analyzer_->record_callback_error(kImageTopic, error.what());
  } catch (...) {
    analyzer_->record_callback_error(kImageTopic, "unknown exception");
  }
}

void RosInputPreflightNode::on_camera_info(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
{
  try {
    analyzer_->observe_camera_info(
      CameraInfoSample{
        message->width, message->height, message->distortion_model,
        std::vector<double>(message->k.begin(), message->k.end()), message->d,
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    analyzer_->record_callback_error(kCameraInfoTopic, error.what());
  } catch (...) {
    analyzer_->record_callback_error(kCameraInfoTopic, "unknown exception");
  }
}

void RosInputPreflightNode::on_vision(
  const auto_aim_interfaces::msg::Vision::ConstSharedPtr & message)
{
  try {
    analyzer_->observe_vision(
      VisionSample{
        to_stamp(message->header.stamp),
        message->yaw, message->yaw_vel, yaw_acceleration(*message),
        message->pitch, message->pitch_vel, pitch_acceleration(*message),
        message->roll,
        std::vector<float>(message->quaternion.begin(), message->quaternion.end()),
        message->shoot_speed,
      },
      monotonic_seconds());
  } catch (const std::exception & error) {
    analyzer_->record_callback_error(kVisionTopic, error.what());
  } catch (...) {
    analyzer_->record_callback_error(kVisionTopic, "unknown exception");
  }
}

}  // namespace auto_aim_tools
