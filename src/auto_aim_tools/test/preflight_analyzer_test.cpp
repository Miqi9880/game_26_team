#include "auto_aim_tools/preflight_analyzer.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace auto_aim_tools
{
namespace
{
ImageSample image(std::int64_t stamp_sec = 1)
{
  return ImageSample{HeaderStamp{stamp_sec, 0U, true}, 2U, 2U, "bgr8", 6U, 12U};
}

CameraInfoSample camera_info()
{
  return CameraInfoSample{
    2U, 2U, "plumb_bob",
    {100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0},
    {0.0, 0.0, 0.0, 0.0, 0.0},
  };
}

VisionSample vision(std::int64_t stamp_sec = 1)
{
  return VisionSample{
    HeaderStamp{stamp_sec, 0U, true},
    10.0F, 2.0F, 0.5F, 5.0F, -1.0F, -0.25F, 0.0F,
    {1.0F, 0.0F, 0.0F, 0.0F}, 20.0F,
  };
}

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

void observe_normal(PreflightAnalyzer * analyzer)
{
  for (int index = 0; index < 2; ++index) {
    const double arrival_s = 0.1 + static_cast<double>(index) * 0.1;
    analyzer->observe_image(image(index + 1), arrival_s);
    analyzer->observe_camera_info(camera_info(), arrival_s);
    analyzer->observe_vision(vision(index + 1), arrival_s);
  }
}

TEST(PreflightAnalyzerTest, NormalMessagesPassWithExplicitProfileAndClockDomain)
{
  PreflightConfig config;
  config.vehicle_profile = "new_turtle";
  config.shared_clock_domain = true;
  PreflightAnalyzer analyzer(config, 0.0);
  observe_normal(&analyzer);

  const auto report = analyzer.build_report(0.25);
  EXPECT_EQ(report.overall, Status::Pass);
  ASSERT_NE(finding(report, "image.data_length"), nullptr);
  EXPECT_EQ(finding(report, "image.data_length")->status, Status::Pass);
  EXPECT_EQ(finding(report, "camera_info.K")->status, Status::Pass);
  EXPECT_EQ(finding(report, "vision.acceleration_finite")->status, Status::Pass);
  EXPECT_EQ(finding(report, "vision.quaternion_format")->status, Status::Pass);
  EXPECT_EQ(finding(report, "image_vision.timestamp_delta")->status, Status::Pass);
  EXPECT_EQ(finding(report, "vision.scalar_finite")->details.at("yaw_vel"), "2 degree/s");
  EXPECT_EQ(
    finding(report, "vision.acceleration_finite")->details.at("pitch_acc"),
    "-0.25 degree/s^2");
}

TEST(PreflightAnalyzerTest, MissingTopicAndTimeoutAreFailures)
{
  PreflightConfig config;
  config.timeout_s = 0.2;
  PreflightAnalyzer analyzer(config, 0.0);
  analyzer.observe_image(image(), 0.1);
  analyzer.observe_camera_info(camera_info(), 0.1);

  const auto report = analyzer.build_report(0.5);
  EXPECT_EQ(finding(report, "topic.received", kVisionTopic)->status, Status::Fail);
  EXPECT_EQ(finding(report, "topic.timeout", kImageTopic)->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, EmptyImageAndBadStepFailWithoutException)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = image();
  sample.width = 0U;
  sample.height = 0U;
  sample.step = 0U;
  sample.data_size = 0U;
  EXPECT_NO_THROW(analyzer.observe_image(sample, 0.1));
  const auto report = analyzer.build_report(0.2);
  EXPECT_EQ(finding(report, "image.dimensions")->status, Status::Fail);
  EXPECT_EQ(finding(report, "image.step")->status, Status::Fail);
  EXPECT_EQ(finding(report, "image.data_length")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, NonFiniteCameraMatrixFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = camera_info();
  sample.k[0] = std::numeric_limits<double>::quiet_NaN();
  analyzer.observe_camera_info(sample, 0.1);
  EXPECT_EQ(finding(analyzer.build_report(0.2), "camera_info.K")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, DistortionLengthMismatchFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = camera_info();
  sample.d.resize(4U);
  analyzer.observe_camera_info(sample, 0.1);
  EXPECT_EQ(finding(analyzer.build_report(0.2), "camera_info.D")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, ZeroCameraDimensionFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = camera_info();
  sample.width = 0U;
  analyzer.observe_camera_info(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "camera_info.dimensions")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, TimestampRollbackRemainsAfterLaterGoodStamp)
{
  PreflightAnalyzer analyzer({}, 0.0);
  analyzer.observe_image(image(10), 0.1);
  analyzer.observe_image(image(9), 0.2);
  analyzer.observe_image(image(11), 0.3);
  const auto * result = finding(
    analyzer.build_report(0.4), "header.timestamp_monotonic", kImageTopic);
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->status, Status::Fail);
  EXPECT_EQ(result->details.at("rollbacks"), "1");
}

TEST(PreflightAnalyzerTest, YawOutsideCanonicalRangeFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.yaw_degree = 180.1F;
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(finding(analyzer.build_report(0.2), "vision.yaw_range")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, NonFiniteVelocityFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.yaw_vel_degree_s = std::numeric_limits<float>::infinity();
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "vision.scalar_finite")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, NonFiniteAccelerationFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.yaw_acc_degree_s2 = std::numeric_limits<float>::quiet_NaN();
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "vision.acceleration_finite")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, ThreeItemQuaternionFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.quaternion_wxyz.resize(3U);
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "vision.quaternion_format")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, NonFiniteQuaternionFails)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.quaternion_wxyz[3] = std::numeric_limits<float>::infinity();
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "vision.quaternion_format")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, PitchIsUndeterminedWithoutExplicitProfile)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.pitch_degree = 999.0F;
  analyzer.observe_vision(sample, 0.1);
  const auto * result = finding(analyzer.build_report(0.2), "vision.pitch_profile");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->status, Status::Warn);
  EXPECT_NE(result->reason.find("无法判定"), std::string::npos);
}

TEST(PreflightAnalyzerTest, ExplicitPitchProfileRangeIsApplied)
{
  PreflightConfig config;
  config.vehicle_profile = "dog_leg";
  PreflightAnalyzer analyzer(config, 0.0);
  auto sample = vision();
  sample.pitch_degree = 31.1F;
  analyzer.observe_vision(sample, 0.1);
  EXPECT_EQ(
    finding(analyzer.build_report(0.2), "vision.pitch_profile")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, MissingAccelerationFieldsAreExplicitlyUnavailable)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = vision();
  sample.yaw_acc_degree_s2.reset();
  sample.pitch_acc_degree_s2.reset();
  analyzer.observe_vision(sample, 0.1);
  const auto * result = finding(
    analyzer.build_report(0.2), "vision.acceleration_finite");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->status, Status::Warn);
  EXPECT_NE(result->reason.find("unavailable"), std::string::npos);
}

TEST(PreflightAnalyzerTest, TimestampsAreNotComparedWithoutClockDeclaration)
{
  PreflightAnalyzer analyzer({}, 0.0);
  analyzer.observe_image(image(100), 0.1);
  analyzer.observe_vision(vision(1), 0.1);
  const auto report = analyzer.build_report(0.2);
  const auto * result = finding(report, "image_vision.clock_domain");
  ASSERT_NE(result, nullptr);
  EXPECT_EQ(result->status, Status::Warn);
  EXPECT_EQ(result->details.at("compared"), "false");
  EXPECT_NE(result->reason.find("时间基准未确认"), std::string::npos);
  EXPECT_EQ(finding(report, "image_vision.timestamp_delta"), nullptr);
}

TEST(PreflightAnalyzerTest, JsonReportIncludesAllStatusNames)
{
  PreflightAnalyzer analyzer({}, 0.0);
  analyzer.observe_image(image(), 0.1);
  const auto rendered = format_report_json(analyzer.build_report(0.2));
  EXPECT_NE(rendered.find("\"PASS\""), std::string::npos);
  EXPECT_NE(rendered.find("\"WARN\""), std::string::npos);
  EXPECT_NE(rendered.find("\"FAIL\""), std::string::npos);
}

}  // namespace
}  // namespace auto_aim_tools
