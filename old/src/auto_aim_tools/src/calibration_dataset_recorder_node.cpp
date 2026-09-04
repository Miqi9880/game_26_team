#include "auto_aim_tools/calibration_dataset_recorder_node.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

namespace auto_aim_tools
{
namespace
{

calibration_dataset::HeaderStamp image_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return calibration_dataset::HeaderStamp{
    stamp.sec, stamp.nanosec, "ros:/image_raw.header.stamp"};
}

calibration_dataset::HeaderStamp camera_stamp(const builtin_interfaces::msg::Time & stamp)
{
  return calibration_dataset::HeaderStamp{
    stamp.sec, stamp.nanosec, "ros:/camera_info.header.stamp"};
}

}  // namespace

CalibrationDatasetRecorderNode::CalibrationDatasetRecorderNode(
  calibration_dataset::DatasetConfig config,
  std::filesystem::path output_directory, std::size_t max_frames,
  std::chrono::milliseconds timeout, std::string git_commit,
  std::size_t max_buffered_image_bytes)
: Node(
    "auto_aim_calibration_dataset_recorder",
    rclcpp::NodeOptions()
    .start_parameter_services(false)
    .start_parameter_event_publisher(false)
    .enable_rosout(false)),
  output_directory_(std::move(output_directory)),
  max_frames_(max_frames),
  max_buffered_image_bytes_(max_buffered_image_bytes),
  timeout_(timeout),
  git_commit_(std::move(git_commit)),
  started_(std::chrono::steady_clock::now())
{
  if (config.source_mode != "ros" || !config.camera_info_required) {
    throw std::invalid_argument("recorder requires ROS config with camera_info_required=true");
  }
  if (config.timestamp_source != "ros_header") {
    throw std::invalid_argument("ROS recorder timestamp_source must be ros_header");
  }
  if (output_directory_.empty() || std::filesystem::exists(output_directory_)) {
    throw std::invalid_argument("recorder output must be a new directory");
  }
  if (max_frames_ == 0U || max_buffered_image_bytes_ == 0U || timeout_.count() <= 0) {
    throw std::invalid_argument("max_frames, max_buffered_image_bytes and timeout must be positive");
  }
  if (config.min_views <= 0 || max_frames_ < static_cast<std::size_t>(config.min_views)) {
    throw std::invalid_argument("max_frames must be at least the configured min_views");
  }
  request_.config = std::move(config);
  request_.image_count_limit = max_frames_;
  request_.image_bytes_limit = max_buffered_image_bytes_;

  const auto qos = rclcpp::SensorDataQoS();
  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", qos,
    [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
      image_callback(message);
    });
  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", qos,
    [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info_callback(message);
    });
  timer_ = create_wall_timer(
    std::chrono::milliseconds(20),
    [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!finished_) {
        publishers_valid_locked();
      }
      if (!finished_ && std::chrono::steady_clock::now() - started_ >= timeout_) {
        finish_locked("input_timeout");
      }
    });
}

std::optional<std::int64_t> CalibrationDatasetRecorderNode::key(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
    stamp.sec > std::numeric_limits<std::int64_t>::max() / 1000000000LL)
  {
    return std::nullopt;
  }
  const auto value =
    stamp.sec * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
  return value == 0 ? std::nullopt : std::optional<std::int64_t>(value);
}

void CalibrationDatasetRecorderNode::image_callback(
  const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (finished_) {
    return;
  }
  ++request_.received_image_count;
  publishers_valid_locked();
  if (finished_) {
    return;
  }
  if (request_.frames.size() >= max_frames_) {
    finish_locked("image_count_limit_exceeded");
    return;
  }
  if (message->data.size() > max_buffered_image_bytes_ - request_.buffered_image_bytes) {
    finish_locked("image_bytes_limit_exceeded");
    return;
  }
  calibration_dataset::FrameInput frame;
  frame.source_image = "ros:/image_raw";
  frame.stamp = image_stamp(message->header.stamp);
  frame.width = message->width;
  frame.height = message->height;
  frame.encoding = message->encoding;
  frame.step = message->step;
  frame.declared_data_size = message->data.size();
  frame.frame_id = message->header.frame_id;
  frame.rgb8 = message->data;

  const auto timestamp = key(message->header.stamp);
  if (timestamp.has_value() && last_image_stamp_.has_value() &&
    *timestamp < *last_image_stamp_)
  {
    frame.input_errors.push_back("image_timestamp_rollback");
  }
  if (timestamp.has_value()) {
    last_image_stamp_ = timestamp;
  }
  const auto frame_index = request_.frames.size();
  request_.frames.push_back(std::move(frame));
  request_.buffered_image_bytes += message->data.size();
  if (!timestamp.has_value()) {
    request_.frames.back().input_errors.push_back("image_timestamp_not_canonical_nonzero");
  } else {
    const auto camera = pending_camera_info_.find(*timestamp);
    if (camera == pending_camera_info_.end()) {
      pending_images_.emplace(*timestamp, PendingImage{frame_index});
      unpaired_image_bytes_ += message->data.size();
      update_unpaired_peak_locked();
      return;
    }
    request_.frames.back().camera_info = camera->second;
    pending_camera_info_.erase(camera);
    ++paired_frames_;
  }
  if (paired_frames_ >= max_frames_) {
    finish_locked({});
  }
}

void CalibrationDatasetRecorderNode::camera_info_callback(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (finished_) {
    return;
  }
  ++request_.received_camera_info_count;
  publishers_valid_locked();
  if (finished_) {
    return;
  }
  calibration_dataset::CameraInfoEvidence camera;
  camera.stamp = camera_stamp(message->header.stamp);
  camera.width = message->width;
  camera.height = message->height;
  camera.frame_id = message->header.frame_id;
  const auto timestamp = key(message->header.stamp);
  if (!timestamp.has_value()) {
    request_.input_errors.push_back("camera_info_timestamp_not_canonical_nonzero");
    return;
  }
  if (last_camera_info_stamp_.has_value() && *timestamp < *last_camera_info_stamp_) {
    request_.input_errors.push_back("camera_info_timestamp_rollback");
  }
  last_camera_info_stamp_ = timestamp;
  const auto image = pending_images_.find(*timestamp);
  if (image == pending_images_.end()) {
    pending_camera_info_.emplace(*timestamp, std::move(camera));
    return;
  }
  const auto frame_index = image->second.frame_index;
  pending_images_.erase(image);
  unpaired_image_bytes_ -= request_.frames.at(frame_index).rgb8.size();
  request_.frames.at(frame_index).camera_info = std::move(camera);
  ++paired_frames_;
  if (paired_frames_ >= max_frames_) {
    finish_locked({});
  }
}

bool CalibrationDatasetRecorderNode::publishers_valid_locked()
{
  const auto validate = [this](const char * topic, const char * expected_type) {
      const auto endpoints = get_publishers_info_by_topic(topic);
      if (endpoints.size() > 1U) {
        finish_locked(std::string("publisher_count_not_one:") + topic + "=" +
          std::to_string(endpoints.size()));
        return false;
      }
      if (endpoints.size() == 1U && endpoints.front().topic_type() != expected_type) {
        finish_locked(std::string("publisher_type_mismatch:") + topic + "=" +
          endpoints.front().topic_type());
        return false;
      }
      if (endpoints.size() == 1U) {
        const auto qos = endpoints.front().qos_profile();
        if (qos.reliability() != rclcpp::ReliabilityPolicy::BestEffort ||
          qos.durability() != rclcpp::DurabilityPolicy::Volatile)
        {
          finish_locked(std::string("publisher_qos_mismatch:") + topic);
          return false;
        }
      }
      return endpoints.size() == 1U;
    };
  const bool image_valid = validate("/image_raw", "sensor_msgs/msg/Image");
  if (finished_) {
    return false;
  }
  const bool camera_valid = validate("/camera_info", "sensor_msgs/msg/CameraInfo");
  if (finished_) {
    return false;
  }
  if (image_valid && camera_valid) {
    publisher_graph_validated_ = true;
    return true;
  }
  if (publisher_graph_validated_) {
    finish_locked("required_publisher_disappeared");
  }
  return false;
}

void CalibrationDatasetRecorderNode::update_unpaired_peak_locked()
{
  request_.peak_unpaired_image_count =
    std::max(request_.peak_unpaired_image_count, pending_images_.size());
  request_.peak_unpaired_image_bytes =
    std::max(request_.peak_unpaired_image_bytes, unpaired_image_bytes_);
}

void CalibrationDatasetRecorderNode::finish_locked(const std::string & reason)
{
  if (finished_) {
    return;
  }
  for (auto & entry : pending_images_) {
    request_.frames.at(entry.second.frame_index).input_errors.push_back("camera_info_missing");
  }
  pending_images_.clear();
  if (!pending_camera_info_.empty()) {
    request_.surplus_camera_info_count = pending_camera_info_.size();
    request_.input_warnings.push_back(
      "surplus_camera_info_count=" + std::to_string(pending_camera_info_.size()) +
      "; all archived Images were exactly paired; BEST_EFFORT/VOLATILE delivery may have "
      "lost Images on the subscriber path; this is not end-to-end lossless-delivery proof");
    pending_camera_info_.clear();
  }
  if (!reason.empty()) {
    request_.input_errors.push_back(reason);
  }
  result_ = calibration_dataset::build_dataset(
    request_, output_directory_, git_commit_);
  finished_ = true;
  timer_->cancel();
}

void CalibrationDatasetRecorderNode::stop(const std::string & reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  finish_locked(reason);
}

bool CalibrationDatasetRecorderNode::finished() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return finished_;
}

std::optional<calibration_dataset::DatasetResult>
CalibrationDatasetRecorderNode::result() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return result_;
}

}  // namespace auto_aim_tools
