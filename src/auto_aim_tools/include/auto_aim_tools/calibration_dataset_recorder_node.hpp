#ifndef AUTO_AIM_TOOLS__CALIBRATION_DATASET_RECORDER_NODE_HPP_
#define AUTO_AIM_TOOLS__CALIBRATION_DATASET_RECORDER_NODE_HPP_

#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "auto_aim_tools/calibration_dataset.hpp"

namespace auto_aim_tools
{

class CalibrationDatasetRecorderNode : public rclcpp::Node
{
public:
  CalibrationDatasetRecorderNode(
    calibration_dataset::DatasetConfig config,
    std::filesystem::path output_directory, std::size_t max_frames,
    std::chrono::milliseconds timeout, std::string git_commit);

  bool finished() const;
  std::optional<calibration_dataset::DatasetResult> result() const;
  void stop(const std::string & reason = {});

private:
  struct PendingImage
  {
    std::size_t frame_index{0};
  };

  static std::optional<std::int64_t> key(
    const builtin_interfaces::msg::Time & stamp) noexcept;
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & message);
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message);
  void finish_locked(const std::string & reason);

  calibration_dataset::DatasetRequest request_;
  std::filesystem::path output_directory_;
  std::size_t max_frames_{0};
  std::size_t paired_frames_{0};
  std::chrono::milliseconds timeout_{0};
  std::string git_commit_;
  std::chrono::steady_clock::time_point started_;
  mutable std::mutex mutex_;
  bool finished_{false};
  std::optional<calibration_dataset::DatasetResult> result_;
  std::multimap<std::int64_t, PendingImage> pending_images_;
  std::multimap<std::int64_t, calibration_dataset::CameraInfoEvidence> pending_camera_info_;
  std::optional<std::int64_t> last_image_stamp_;
  std::optional<std::int64_t> last_camera_info_stamp_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace auto_aim_tools

#endif  // AUTO_AIM_TOOLS__CALIBRATION_DATASET_RECORDER_NODE_HPP_
