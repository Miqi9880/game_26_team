#include "auto_aim_tools/preflight_analyzer.hpp"
#include "auto_aim_tools/ros_input_preflight_node.hpp"

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "auto_aim_interfaces/msg/vision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace auto_aim_tools
{
namespace
{
using namespace std::chrono_literals;

class FakeInputPublisher final : public rclcpp::Node
{
public:
  FakeInputPublisher(
    std::string scenario, bool image, bool info, bool vision)
  : Node("preflight_fake_input_publisher"), scenario_(std::move(scenario))
  {
    const auto qos = scenario_ == "wrong_qos" ?
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile() :
      rclcpp::QoS(rclcpp::SensorDataQoS());
    if (image) {
      image_publisher_ = create_publisher<sensor_msgs::msg::Image>(
        scenario_ == "wrong_topic" ? "/wrong_image" : kImageTopic, qos);
    }
    if (info) {
      info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        kCameraInfoTopic, qos);
    }
    if (vision) {
      vision_publisher_ = create_publisher<auto_aim_interfaces::msg::Vision>(
        kVisionTopic, rclcpp::SensorDataQoS());
    }
  }

  void publish(std::int32_t stamp_sec)
  {
    if (image_publisher_) {
      sensor_msgs::msg::Image message;
      message.header.stamp.sec = scenario_ == "unset_stamp" ? 0 : stamp_sec;
      message.header.frame_id = "camera_optical_frame";
      message.encoding = scenario_ == "wrong_encoding" ? "bgr8" : "rgb8";
      if (scenario_ != "empty_image") {
        message.width = 2U;
        message.height = 2U;
        message.step = 6U;
        message.data.resize(12U);
      }
      if (scenario_ == "bad_stride") {
        message.step = 7U;
        message.data.resize(14U);
      } else if (scenario_ == "short_data") {
        message.data.resize(11U);
      }
      image_publisher_->publish(message);
    }
    if (info_publisher_) {
      sensor_msgs::msg::CameraInfo message;
      message.header.stamp.sec = scenario_ == "unset_stamp" ? 0 :
        stamp_sec + (scenario_ == "timestamp_mismatch" ? 100 : 0);
      message.header.frame_id = scenario_ == "frame_id_mismatch" ?
        "other_camera_frame" : "camera_optical_frame";
      message.width = 2U;
      message.height = 2U;
      message.distortion_model = "plumb_bob";
      message.d.assign(5U, 0.0);
      message.k = {{100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0}};
      if (scenario_ == "invalid_camera_info") {
        message.k[0] = std::numeric_limits<double>::quiet_NaN();
      } else if (scenario_ == "invalid_distortion") {
        message.d[0] = std::numeric_limits<double>::infinity();
      } else if (scenario_ == "dimension_mismatch") {
        message.width = 3U;
      }
      info_publisher_->publish(message);
    }
    if (vision_publisher_) {
      auto_aim_interfaces::msg::Vision message;
      message.header.stamp.sec = stamp_sec;
      message.yaw = scenario_ == "invalid_vision" ? 181.0F : 10.0F;
      message.yaw_vel = 1.0F;
      message.pitch = 5.0F;
      message.pitch_vel = 1.0F;
      message.roll = 0.0F;
      message.quaternion = {{1.0F, 0.0F, 0.0F, 0.0F}};
      message.shoot_speed = 20.0F;
      vision_publisher_->publish(message);
    }
  }

private:
  std::string scenario_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr vision_publisher_;
};

const Finding * finding(
  const Report & report, const std::string & check,
  const std::string & topic = "")
{
  for (const auto & item : report.findings) {
    if (item.check == check && (topic.empty() || item.topic == topic)) {
      return &item;
    }
  }
  return nullptr;
}

void spin_for(rclcpp::Executor * executor, std::chrono::milliseconds duration)
{
  const auto deadline = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < deadline) {
    executor->spin_some();
    std::this_thread::sleep_for(5ms);
  }
}

Report run_scenario(
  const std::string & scenario, bool publish_image = true,
  bool publish_info = true, bool publish_vision = true,
  double timeout_s = 0.3)
{
  PreflightConfig config;
  config.timeout_s = timeout_s;
  config.vehicle_profile = "new_turtle";
  config.shared_clock_domain = true;
  auto analyzer = std::make_shared<PreflightAnalyzer>(config, monotonic_seconds());
  auto preflight = std::make_shared<RosInputPreflightNode>(analyzer);
  auto publisher = std::make_shared<FakeInputPublisher>(
    scenario, publish_image, publish_info, publish_vision);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(preflight->get_node_base_interface());
  executor.add_node(publisher);
  spin_for(&executor, 100ms);

  const std::array<std::int32_t, 3> normal_stamps{{1, 2, 3}};
  const std::array<std::int32_t, 3> rollback_stamps{{10, 9, 11}};
  const auto & stamps = scenario == "timestamp_rollback" ? rollback_stamps : normal_stamps;
  for (const auto stamp : stamps) {
    publisher->publish(stamp);
    spin_for(&executor, 50ms);
  }
  if (scenario == "timeout") {
    spin_for(
      &executor,
      std::chrono::milliseconds(static_cast<int>(timeout_s * 1000.0) + 80));
  }

  preflight->inspect_graph_contract();
  const auto report = analyzer->build_report(monotonic_seconds());
  EXPECT_TRUE(
    preflight->get_node_graph_interface()->get_publishers_info_by_topic(
      "/Robot_ctrl_data").empty());
  executor.remove_node(publisher);
  executor.remove_node(preflight->get_node_base_interface());
  return report;
}

class FakeRosPublishersTest : public ::testing::Test
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

TEST_F(FakeRosPublishersTest, NormalInputIsAccepted)
{
  const auto report = run_scenario("normal");
  EXPECT_EQ(finding(report, "image.data_length")->status, Status::Pass);
  EXPECT_EQ(finding(report, "camera_info.K")->status, Status::Pass);
  EXPECT_EQ(finding(report, "vision.yaw_range")->status, Status::Pass);
  EXPECT_EQ(finding(report, "vision.acceleration_finite")->status, Status::Warn);
}

TEST_F(FakeRosPublishersTest, NodeGraphContainsOnlyThreeInputSubscriptions)
{
  auto analyzer = std::make_shared<PreflightAnalyzer>();
  auto preflight = std::make_shared<RosInputPreflightNode>(analyzer);
  const auto graph = preflight->get_node_graph_interface();

  const auto publishers = graph->get_publisher_names_and_types_by_node(
    "ros_input_preflight", "/");
  const auto subscriptions = graph->get_subscriber_names_and_types_by_node(
    "ros_input_preflight", "/");
  const auto services = graph->get_service_names_and_types_by_node(
    "ros_input_preflight", "/");
  const auto clients = graph->get_client_names_and_types_by_node(
    "ros_input_preflight", "/");

  EXPECT_TRUE(publishers.empty());
  EXPECT_TRUE(services.empty());
  EXPECT_TRUE(clients.empty());
  EXPECT_EQ(subscriptions.size(), 3U);
  EXPECT_EQ(subscriptions.count(kImageTopic), 1U);
  EXPECT_EQ(subscriptions.count(kCameraInfoTopic), 1U);
  EXPECT_EQ(subscriptions.count(kVisionTopic), 1U);
}

TEST_F(FakeRosPublishersTest, MissingTopicFails)
{
  const auto report = run_scenario("missing_topic", true, true, false);
  EXPECT_EQ(finding(report, "topic.received", kVisionTopic)->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, MissingCameraInfoFails)
{
  const auto report = run_scenario("missing_camera_info", true, false, true);
  EXPECT_EQ(finding(report, "topic.received", kCameraInfoTopic)->status, Status::Fail);
  EXPECT_EQ(finding(report, "topic.publisher_count", kCameraInfoTopic)->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, EmptyImageFails)
{
  const auto report = run_scenario("empty_image");
  EXPECT_EQ(finding(report, "image.data_length")->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, InvalidCameraInfoFails)
{
  const auto report = run_scenario("invalid_camera_info");
  EXPECT_EQ(finding(report, "camera_info.K")->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, InvalidDistortionAndDimensionMismatchFail)
{
  const auto distortion = run_scenario("invalid_distortion");
  EXPECT_EQ(finding(distortion, "camera_info.D")->status, Status::Fail);

  const auto dimensions = run_scenario("dimension_mismatch");
  EXPECT_EQ(finding(dimensions, "image_camera.dimensions")->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, MalformedImageContractFails)
{
  const auto encoding = run_scenario("wrong_encoding");
  EXPECT_EQ(finding(encoding, "image.encoding")->status, Status::Fail);

  const auto stride = run_scenario("bad_stride");
  EXPECT_EQ(finding(stride, "image.step")->status, Status::Fail);

  const auto data = run_scenario("short_data");
  EXPECT_EQ(finding(data, "image.data_length")->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, PairingStampAndFrameIdFailuresAreExplicit)
{
  const auto timestamp = run_scenario("timestamp_mismatch");
  EXPECT_EQ(finding(timestamp, "image_camera.timestamp_pairing")->status, Status::Fail);

  const auto unset = run_scenario("unset_stamp");
  EXPECT_EQ(
    finding(unset, "header.timestamp_monotonic", kImageTopic)->status,
    Status::Fail);

  const auto frame_id = run_scenario("frame_id_mismatch");
  EXPECT_EQ(finding(frame_id, "camera_info.frame_id")->status, Status::Fail);
  EXPECT_EQ(finding(frame_id, "image_camera.frame_id")->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, WrongTopicAndQosProduceGraphFailures)
{
  const auto topic = run_scenario("wrong_topic");
  EXPECT_EQ(finding(topic, "topic.publisher_count", kImageTopic)->status, Status::Fail);

  const auto qos = run_scenario("wrong_qos");
  EXPECT_EQ(finding(qos, "topic.qos", kImageTopic)->status, Status::Fail);
  EXPECT_EQ(finding(qos, "topic.qos", kCameraInfoTopic)->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, TimestampRollbackFails)
{
  const auto report = run_scenario("timestamp_rollback");
  EXPECT_EQ(
    finding(report, "header.timestamp_monotonic", kImageTopic)->status,
    Status::Fail);
  EXPECT_EQ(
    finding(report, "header.timestamp_monotonic", kCameraInfoTopic)->status,
    Status::Fail);
  EXPECT_EQ(
    finding(report, "header.timestamp_monotonic", kVisionTopic)->status,
    Status::Fail);
}

TEST_F(FakeRosPublishersTest, TimeoutFails)
{
  const auto report = run_scenario("timeout", true, true, true, 0.1);
  EXPECT_EQ(finding(report, "topic.timeout", kImageTopic)->status, Status::Fail);
  EXPECT_EQ(finding(report, "topic.timeout", kVisionTopic)->status, Status::Fail);
}

TEST_F(FakeRosPublishersTest, InvalidVisionFails)
{
  const auto report = run_scenario("invalid_vision");
  EXPECT_EQ(finding(report, "vision.yaw_range")->status, Status::Fail);
}

}  // namespace
}  // namespace auto_aim_tools
