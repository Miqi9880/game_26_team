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
  return ImageSample{
    HeaderStamp{stamp_sec, 0U, true}, "camera_optical_frame",
    2U, 2U, "rgb8", 6U, 12U,
  };
}

CameraInfoSample camera_info(std::int64_t stamp_sec = 1)
{
  return CameraInfoSample{
    HeaderStamp{stamp_sec, 0U, true}, "camera_optical_frame",
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
    analyzer->observe_camera_info(camera_info(index + 1), arrival_s);
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

TEST(PreflightAnalyzerTest, CameraContractRejectsWrongEncodingStrideAndShortData)
{
  PreflightAnalyzer encoding_analyzer({}, 0.0);
  auto wrong_encoding = image();
  wrong_encoding.encoding = "bgr8";
  encoding_analyzer.observe_image(wrong_encoding, 0.1);
  EXPECT_EQ(
    finding(encoding_analyzer.build_report(0.2), "image.encoding")->status,
    Status::Fail);

  PreflightAnalyzer stride_analyzer({}, 0.0);
  auto wrong_stride = image();
  wrong_stride.step = 7U;
  wrong_stride.data_size = 14U;
  stride_analyzer.observe_image(wrong_stride, 0.1);
  EXPECT_EQ(
    finding(stride_analyzer.build_report(0.2), "image.step")->status,
    Status::Fail);

  PreflightAnalyzer data_analyzer({}, 0.0);
  auto short_data = image();
  short_data.data_size = 11U;
  data_analyzer.observe_image(short_data, 0.1);
  EXPECT_EQ(
    finding(data_analyzer.build_report(0.2), "image.data_length")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, PairedFramesRequireExactStampDimensionsAndFrameId)
{
  PreflightAnalyzer timestamp_analyzer({}, 0.0);
  timestamp_analyzer.observe_image(image(1), 0.1);
  timestamp_analyzer.observe_camera_info(camera_info(2), 0.1);
  EXPECT_EQ(
    finding(
      timestamp_analyzer.build_report(0.2),
      "image_camera.timestamp_pairing")->status,
    Status::Fail);

  PreflightAnalyzer dimensions_analyzer({}, 0.0);
  dimensions_analyzer.observe_image(image(1), 0.1);
  auto wrong_dimensions = camera_info(1);
  wrong_dimensions.width = 3U;
  dimensions_analyzer.observe_camera_info(wrong_dimensions, 0.1);
  EXPECT_EQ(
    finding(dimensions_analyzer.build_report(0.2), "image_camera.dimensions")->status,
    Status::Fail);

  PreflightAnalyzer frame_analyzer({}, 0.0);
  frame_analyzer.observe_image(image(1), 0.1);
  auto wrong_frame = camera_info(1);
  wrong_frame.frame_id = "other_camera_frame";
  frame_analyzer.observe_camera_info(wrong_frame, 0.1);
  const auto frame_report = frame_analyzer.build_report(0.2);
  EXPECT_EQ(finding(frame_report, "camera_info.frame_id")->status, Status::Fail);
  EXPECT_EQ(finding(frame_report, "image_camera.frame_id")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, CameraInfoOnlySurplusWarnsButUnmatchedImageFails)
{
  PreflightConfig config;
  config.vehicle_profile = "new_turtle";
  config.shared_clock_domain = true;
  PreflightAnalyzer surplus(config, 0.0);
  surplus.observe_camera_info(camera_info(1), 0.1);
  surplus.observe_image(image(2), 0.2);
  surplus.observe_camera_info(camera_info(2), 0.2);
  surplus.observe_vision(vision(2), 0.2);
  surplus.observe_image(image(3), 0.3);
  surplus.observe_camera_info(camera_info(3), 0.3);
  surplus.observe_vision(vision(3), 0.3);
  surplus.observe_camera_info(camera_info(4), 0.4);
  const auto surplus_report = surplus.build_report(0.6);
  EXPECT_EQ(surplus_report.overall, Status::Warn);
  const auto * pairing = finding(surplus_report, "image_camera.timestamp_pairing");
  ASSERT_NE(pairing, nullptr);
  EXPECT_EQ(pairing->status, Status::Warn);
  EXPECT_EQ(pairing->details.at("matched"), "2");
  EXPECT_EQ(pairing->details.at("unmatched_images"), "0");
  EXPECT_EQ(pairing->details.at("unmatched_camera_info"), "2");
  EXPECT_EQ(pairing->details.at("surplus_camera_info"), "2");
  EXPECT_EQ(pairing->details.at("stale_unmatched_camera_info"), "2");
  EXPECT_EQ(pairing->details.at("camera_info_only_surplus"), "true");
  EXPECT_NE(pairing->reason.find("image-side BEST_EFFORT DDS delivery loss"), std::string::npos);

  PreflightAnalyzer unmatched_image({}, 0.0);
  unmatched_image.observe_image(image(1), 0.1);
  unmatched_image.observe_camera_info(camera_info(1), 0.1);
  unmatched_image.observe_image(image(2), 0.15);
  EXPECT_EQ(
    finding(unmatched_image.build_report(0.2), "image_camera.timestamp_pairing")->status,
    Status::Fail);
}

TEST(PreflightAnalyzerTest, ImageAndCameraInfoUnsetTimestampsFail)
{
  PreflightAnalyzer analyzer({}, 0.0);
  analyzer.observe_image(image(0), 0.1);
  analyzer.observe_camera_info(camera_info(0), 0.1);
  const auto report = analyzer.build_report(0.2);
  EXPECT_EQ(
    finding(report, "header.timestamp_monotonic", kImageTopic)->status,
    Status::Fail);
  EXPECT_EQ(
    finding(report, "header.timestamp_monotonic", kCameraInfoTopic)->status,
    Status::Fail);
  EXPECT_EQ(finding(report, "image_camera.timestamp_pairing")->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, GraphContractRejectsWrongTypeAndQos)
{
  PreflightAnalyzer analyzer({}, 0.0);
  analyzer.observe_topic_publishers(
    kImageTopic, kExpectedImageType,
    {PublisherEndpointSample{
        "std_msgs/msg/String", "reliable", "volatile", "keep_last", 10U}});
  const auto report = analyzer.build_report(0.1);
  EXPECT_EQ(finding(report, "topic.type", kImageTopic)->status, Status::Fail);
  EXPECT_EQ(finding(report, "topic.qos", kImageTopic)->status, Status::Fail);
}

TEST(PreflightAnalyzerTest, TypedEncodingRowWidthOverflowFailsClosed)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = image();
  sample.encoding = "64UC1152921504606846976";
  sample.width = 2U;
  sample.height = 1U;
  sample.step = 1U;
  sample.data_size = 1U;
  EXPECT_NO_THROW(analyzer.observe_image(sample, 0.1));

  const auto report = analyzer.build_report(0.2);
  EXPECT_EQ(finding(report, "image.encoding")->status, Status::Fail);
  EXPECT_EQ(finding(report, "image.step")->status, Status::Fail);
  EXPECT_EQ(finding(report, "image.data_length")->status, Status::Fail);
  EXPECT_NE(finding(report, "image.step")->reason.find("width * 3"), std::string::npos);
}

TEST(PreflightAnalyzerTest, TypedEncodingNumericOverflowFailsClosed)
{
  PreflightAnalyzer analyzer({}, 0.0);
  auto sample = image();
  sample.encoding = "64UC999999999999999999999999999999999999999999999999";
  EXPECT_NO_THROW(analyzer.observe_image(sample, 0.1));

  const auto report = analyzer.build_report(0.2);
  EXPECT_EQ(finding(report, "image.encoding")->status, Status::Fail);
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
