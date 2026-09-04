#include "auto_aim_tools/calibration_dataset_recorder_node.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

using namespace std::chrono_literals;
namespace dataset = auto_aim_tools::calibration_dataset;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    static std::size_t sequence = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("calibration_dataset_ros_test_" + std::to_string(++sequence));
    std::filesystem::create_directories(path_);
  }
  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }
  const std::filesystem::path & path() const { return path_; }

private:
  std::filesystem::path path_;
};

dataset::DatasetConfig ros_config(int min_views = 2)
{
  dataset::DatasetConfig config;
  config.schema_version = 1;
  config.source_mode = "ros";
  config.timestamp_source = "ros_header";
  config.camera_info_required = true;
  config.min_views = min_views;
  config.max_global_rms_reprojection_error_px = 0.5;
  config.board = {"chessboard", 9, 6, 0.024};
  config.metadata = {
    "ros-synthetic", "ros-synthetic-report", "unknown", "unknown",
    "2026-08-27", "automated-test"};
  return config;
}

sensor_msgs::msg::Image image(std::int32_t sec, std::uint8_t value)
{
  sensor_msgs::msg::Image message;
  message.header.stamp.sec = sec;
  message.header.stamp.nanosec = 123;
  message.header.frame_id = "camera_optical_frame";
  message.width = 4;
  message.height = 3;
  message.encoding = "rgb8";
  message.step = 12;
  message.data.assign(36, value);
  return message;
}

sensor_msgs::msg::CameraInfo camera_info(std::int32_t sec)
{
  sensor_msgs::msg::CameraInfo message;
  message.header.stamp.sec = sec;
  message.header.stamp.nanosec = 123;
  message.header.frame_id = "camera_optical_frame";
  message.width = 4;
  message.height = 3;
  return message;
}

class CalibrationDatasetRosTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

}  // namespace

TEST_F(CalibrationDatasetRosTest, ExactPairsCreateDatasetAndRecorderHasNoPublishers)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(), temporary.path() / "dataset", 2, 2s, "abcdef0");
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_fake_publisher");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  auto camera_publisher = publisher_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while ((image_publisher->get_subscription_count() == 0U ||
    camera_publisher->get_subscription_count() == 0U) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_GT(image_publisher->get_subscription_count(), 0U);
  ASSERT_GT(camera_publisher->get_subscription_count(), 0U);
  image_publisher->publish(image(10, 17));
  camera_publisher->publish(camera_info(10));
  image_publisher->publish(image(11, 29));
  camera_publisher->publish(camera_info(11));
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->accepted);
  EXPECT_EQ(YAML::LoadFile(result->manifest_path.string())["records"].size(), 2U);
  const auto publishers =
    recorder->get_node_graph_interface()->get_publisher_names_and_types_by_node(
    recorder->get_name(), recorder->get_namespace());
  EXPECT_TRUE(publishers.empty());
  EXPECT_EQ(recorder->count_publishers("/Robot_ctrl_data"), 0U);
}

TEST_F(CalibrationDatasetRosTest, MissingCameraInfoTimesOutToInputEvidence)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 250ms, "abcdef0");
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_missing_info_pub");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    image_publisher->publish(image(20, 45));
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["status"].as<std::string>(), "rejected");
  ASSERT_GT(manifest["records"].size(), 0U);
  EXPECT_FALSE(manifest["records"][0]["camera_info"]["present"].as<bool>());
}

TEST_F(CalibrationDatasetRosTest, SurplusCameraInfoCreatesAcceptedManifestWithWarning)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(), temporary.path() / "dataset", 2, 2s, "abcdef0");
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_surplus_info_pub");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  auto camera_publisher = publisher_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);

  const auto discovery_deadline = std::chrono::steady_clock::now() + 1s;
  while ((image_publisher->get_subscription_count() == 0U ||
    camera_publisher->get_subscription_count() == 0U) &&
    std::chrono::steady_clock::now() < discovery_deadline)
  {
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_GT(image_publisher->get_subscription_count(), 0U);
  ASSERT_GT(camera_publisher->get_subscription_count(), 0U);
  camera_publisher->publish(camera_info(9));
  image_publisher->publish(image(10, 17));
  camera_publisher->publish(camera_info(10));
  image_publisher->publish(image(11, 29));
  camera_publisher->publish(camera_info(11));
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["profile"].as<std::string>(), "evidence_only");
  EXPECT_FALSE(manifest["production_ready"].as<bool>());
  EXPECT_EQ(manifest["summary"]["received_image_count"].as<std::size_t>(), 2U);
  EXPECT_EQ(manifest["summary"]["received_camera_info_count"].as<std::size_t>(), 3U);
  EXPECT_EQ(manifest["summary"]["surplus_camera_info_count"].as<std::size_t>(), 1U);
  ASSERT_EQ(manifest["warnings"].size(), 1U);
  const auto warning = manifest["warnings"][0].as<std::string>();
  EXPECT_NE(warning.find("surplus_camera_info_count=1"), std::string::npos);
  EXPECT_NE(warning.find("BEST_EFFORT/VOLATILE"), std::string::npos);
  EXPECT_NE(warning.find("not end-to-end lossless-delivery proof"), std::string::npos);
}

TEST_F(CalibrationDatasetRosTest, HighRateMissingCameraInfoIsHardBoundedByMaxFrames)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 2s, "abcdef0");
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_bounded_pub");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);

  const auto deadline = std::chrono::steady_clock::now() + 1s;
  std::int32_t stamp = 100;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    image_publisher->publish(image(stamp++, 45));
    executor.spin_some();
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["summary"]["record_count"].as<std::size_t>(), 1U);
  EXPECT_EQ(manifest["summary"]["peak_unpaired_image_count"].as<std::size_t>(), 1U);
  EXPECT_EQ(manifest["summary"]["peak_unpaired_image_bytes"].as<std::size_t>(), 36U);
  EXPECT_EQ(manifest["limits"]["image_count"].as<std::size_t>(), 1U);
  const auto reasons = manifest["rejection_reasons"];
  ASSERT_TRUE(reasons.IsSequence());
  EXPECT_TRUE(std::any_of(
    reasons.begin(), reasons.end(), [](const YAML::Node & reason) {
      return reason.as<std::string>() == "image_count_limit_exceeded";
    }));
}

TEST_F(CalibrationDatasetRosTest, MultipleImagePublishersImmediatelyRejectEvidence)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 2s, "abcdef0");
  auto first = std::make_shared<rclcpp::Node>("calibration_dataset_first_pub");
  auto second = std::make_shared<rclcpp::Node>("calibration_dataset_second_pub");
  auto camera_node = std::make_shared<rclcpp::Node>("calibration_dataset_camera_pub");
  auto first_image = first->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  auto second_image = second->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  auto camera = camera_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(first);
  executor.add_node(second);
  executor.add_node(camera_node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["records"].size(), 0U);
  EXPECT_TRUE(std::any_of(
    manifest["rejection_reasons"].begin(), manifest["rejection_reasons"].end(),
    [](const YAML::Node & reason) {
      return reason.as<std::string>() == "publisher_count_not_one:/image_raw=2";
    }));
  (void)first_image;
  (void)second_image;
  (void)camera;
}

TEST_F(CalibrationDatasetRosTest, ImageByteLimitRejectsBeforeCopyingOversizedFrame)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 2s, "abcdef0", 35U);
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_byte_limit_pub");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::SensorDataQoS());
  auto camera_publisher = publisher_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    image_publisher->publish(image(300, 45));
    executor.spin_some();
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["records"].size(), 0U);
  EXPECT_EQ(manifest["limits"]["image_bytes"].as<std::size_t>(), 35U);
  EXPECT_TRUE(std::any_of(
    manifest["rejection_reasons"].begin(), manifest["rejection_reasons"].end(),
    [](const YAML::Node & reason) {
      return reason.as<std::string>() == "image_bytes_limit_exceeded";
    }));
  (void)camera_publisher;
}

TEST_F(CalibrationDatasetRosTest, ReliablePublisherQosIsRejected)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 2s, "abcdef0");
  auto publisher_node = std::make_shared<rclcpp::Node>("calibration_dataset_wrong_qos_pub");
  auto image_publisher = publisher_node->create_publisher<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::QoS(5).reliable().durability_volatile());
  auto camera_publisher = publisher_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(recorder);
  executor.add_node(publisher_node);
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!recorder->finished() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(recorder->finished());
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  EXPECT_TRUE(std::any_of(
    result->rejection_reasons.begin(), result->rejection_reasons.end(),
    [](const std::string & reason) {
      return reason == "publisher_qos_mismatch:/image_raw";
    }));
  (void)image_publisher;
  (void)camera_publisher;
}

TEST_F(CalibrationDatasetRosTest, UnsupportedTimestampClaimIsRejectedBeforeRecording)
{
  TemporaryDirectory temporary;
  auto config = ros_config(1);
  config.timestamp_source = "mcu_time";
  EXPECT_THROW(
    std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
      config, temporary.path() / "dataset", 1, 2s, "abcdef0"),
    std::invalid_argument);
}

TEST_F(CalibrationDatasetRosTest, EmptyInputAndRepeatedStopAreIdempotent)
{
  TemporaryDirectory temporary;
  auto recorder = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
    ros_config(1), temporary.path() / "dataset", 1, 2s, "abcdef0");
  EXPECT_NO_THROW(recorder->stop("input_unavailable"));
  EXPECT_NO_THROW(recorder->stop("second_stop_must_not_rewrite"));
  const auto result = recorder->result();
  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->accepted);
  const auto manifest = YAML::LoadFile(result->manifest_path.string());
  EXPECT_EQ(manifest["source"]["input_status"].as<std::string>(), "input_unavailable");
  EXPECT_EQ(manifest["source"]["camera_sdk_status"].as<std::string>(), "not_used");
  EXPECT_EQ(manifest["records"].size(), 0U);
}
