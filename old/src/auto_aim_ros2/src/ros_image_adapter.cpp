#include "auto_aim_ros2/ros_image_adapter.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <sstream>

#include <opencv2/imgproc.hpp>

namespace rm_auto_aim::ros_adapters
{
namespace
{
void set_error(std::string * error, const std::string & message)
{
  if (error != nullptr) {
    *error = message;
  }
}

struct EncodingInfo
{
  int cv_type;
  int channels;
};

std::optional<EncodingInfo> encoding_info(const std::string & encoding)
{
  if (encoding == "bgr8" || encoding == "rgb8") {
    return EncodingInfo{CV_8UC3, 3};
  }
  if (encoding == "bgra8" || encoding == "rgba8") {
    return EncodingInfo{CV_8UC4, 4};
  }
  if (encoding == "mono8") {
    return EncodingInfo{CV_8UC1, 1};
  }
  return std::nullopt;
}
}  // namespace

std::optional<cv::Mat> to_bgr_image(
  const sensor_msgs::msg::Image & message,
  std::string * error)
{
  if (message.width == 0 || message.height == 0) {
    set_error(error, "sensor_msgs/Image has zero width or height");
    return std::nullopt;
  }
  const auto info = encoding_info(message.encoding);
  if (!info.has_value()) {
    set_error(error, "unsupported sensor_msgs/Image encoding: " + message.encoding);
    return std::nullopt;
  }

  const std::size_t minimum_step = static_cast<std::size_t>(message.width) *
    static_cast<std::size_t>(info->channels);
  if (message.step < minimum_step) {
    set_error(error, "sensor_msgs/Image step is smaller than the encoded row width");
    return std::nullopt;
  }
  const auto required_bytes = static_cast<std::size_t>(message.step) *
    static_cast<std::size_t>(message.height);
  if (required_bytes > message.data.size()) {
    set_error(error, "sensor_msgs/Image data is shorter than height * step");
    return std::nullopt;
  }

  // cv::Mat's external-data constructor is non-const, but the clone below is
  // immediate and the source buffer is never modified.
  cv::Mat source(
    static_cast<int>(message.height), static_cast<int>(message.width), info->cv_type,
    const_cast<std::uint8_t *>(message.data.data()), static_cast<std::size_t>(message.step));
  cv::Mat bgr;
  if (message.encoding == "bgr8") {
    bgr = source;
  } else if (message.encoding == "rgb8") {
    cv::cvtColor(source, bgr, cv::COLOR_RGB2BGR);
  } else if (message.encoding == "bgra8") {
    cv::cvtColor(source, bgr, cv::COLOR_BGRA2BGR);
  } else if (message.encoding == "rgba8") {
    cv::cvtColor(source, bgr, cv::COLOR_RGBA2BGR);
  } else {  // mono8
    cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
  }
  if (bgr.empty() || bgr.type() != CV_8UC3) {
    set_error(error, "failed to convert sensor_msgs/Image to BGR CV_8UC3");
    return std::nullopt;
  }
  return bgr.clone();
}

std::optional<pipeline::ImageFrame> to_image_frame(
  const sensor_msgs::msg::Image & message,
  float shoot_speed_mps,
  std::uint16_t bullet_count,
  std::uint8_t game_progress,
  std::string * error)
{
  if (message.header.stamp.sec < 0 ||
    message.header.stamp.nanosec >= 1'000'000'000U ||
    (message.header.stamp.sec == 0 && message.header.stamp.nanosec == 0))
  {
    set_error(error, "sensor_msgs/Image has an invalid or unset timestamp");
    return std::nullopt;
  }
  auto bgr = to_bgr_image(message, error);
  if (!bgr.has_value()) {
    return std::nullopt;
  }

  pipeline::ImageFrame frame{};
  frame.stamp_ns = static_cast<std::int64_t>(message.header.stamp.sec) * 1'000'000'000LL +
    static_cast<std::int64_t>(message.header.stamp.nanosec);
  frame.width = message.width;
  frame.height = message.height;
  frame.encoding = message.encoding;
  frame.bgr_image = std::move(*bgr);
  frame.shoot_speed_mps = shoot_speed_mps;
  frame.bullet_count = bullet_count;
  frame.game_progress = game_progress;
  return frame;
}

}  // namespace rm_auto_aim::ros_adapters
