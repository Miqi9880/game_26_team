#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::detector::DetectorConfig;
using rm_auto_aim::detector::make_letterbox;
using rm_auto_aim::detector::make_raw_armor_detection;
using rm_auto_aim::detector::model_point_to_image;
using rm_auto_aim::detector::validate_input_shape;
using rm_auto_aim::detector::validate_output_shape;
}

TEST(RawArmorDetection, RejectsWrongKeypointCount)
{
  const cv::Rect2f box(1.0F, 2.0F, 10.0F, 8.0F);
  const std::vector<cv::Point2f> points(3, cv::Point2f(1.0F, 1.0F));
  EXPECT_FALSE(make_raw_armor_detection(1, 0, 0.9F, box, points, 9, 4).has_value());
}

TEST(RawArmorDetection, RejectsInvalidConfidenceAndClassBounds)
{
  const cv::Rect2f box(1.0F, 2.0F, 10.0F, 8.0F);
  const std::vector<cv::Point2f> points{
    {1.0F, 2.0F}, {11.0F, 2.0F}, {11.0F, 10.0F}, {1.0F, 10.0F}};
  EXPECT_FALSE(
    make_raw_armor_detection(9, 0, 0.9F, box, points, 9, 4).has_value());
  EXPECT_FALSE(
    make_raw_armor_detection(1, 0, std::numeric_limits<float>::quiet_NaN(), box, points, 9, 4)
      .has_value());
  EXPECT_FALSE(make_raw_armor_detection(1, 4, 0.9F, box, points, 9, 4).has_value());
}

TEST(RawArmorDetection, AcceptsFiniteFourPointEvidenceOnly)
{
  const cv::Rect2f box(1.0F, 2.0F, 10.0F, 8.0F);
  const std::vector<cv::Point2f> points{
    {1.0F, 2.0F}, {11.0F, 2.0F}, {11.0F, 10.0F}, {1.0F, 10.0F}};
  const auto detection = make_raw_armor_detection(1, 0, 0.9F, box, points, 9, 4);
  ASSERT_TRUE(detection.has_value());
  EXPECT_EQ(detection->class_id, 1);
  EXPECT_EQ(detection->color_id, 0);
  EXPECT_FLOAT_EQ(detection->confidence, 0.9F);
}

TEST(RawArmorDetection, EmptyImageDoesNotProduceLetterbox)
{
  DetectorConfig config{};
  EXPECT_FALSE(make_letterbox(cv::Mat{}, config).has_value());
}

TEST(RawArmorDetection, LetterboxPreservesExplicitTopLeftContract)
{
  DetectorConfig config{};
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto result = make_letterbox(image, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->image.size(), cv::Size(640, 640));
  EXPECT_EQ(result->pad_x, 0);
  EXPECT_EQ(result->pad_y, 0);
  EXPECT_FLOAT_EQ(result->scale, 3.2F);
  EXPECT_EQ(result->image.type(), CV_8UC3);
}

TEST(RawArmorDetection, ModelPointInverseRestoresTopLeftLetterboxCoordinates)
{
  DetectorConfig config{};
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto letterbox = make_letterbox(image, config);
  ASSERT_TRUE(letterbox.has_value());

  const cv::Point2f original{50.0F, 25.0F};
  const cv::Point2f model{
    original.x * letterbox->scale + static_cast<float>(letterbox->pad_x),
    original.y * letterbox->scale + static_cast<float>(letterbox->pad_y)};
  const auto restored = model_point_to_image(model, *letterbox);
  ASSERT_TRUE(restored.has_value());
  EXPECT_NEAR(restored->x, original.x, 1e-5F);
  EXPECT_NEAR(restored->y, original.y, 1e-5F);
}

TEST(RawArmorDetection, ModelPointInverseRestoresCenterPaddedCoordinates)
{
  DetectorConfig config{};
  config.center_padding = true;
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto letterbox = make_letterbox(image, config);
  ASSERT_TRUE(letterbox.has_value());
  ASSERT_GT(letterbox->pad_y, 0);

  const cv::Point2f original{50.0F, 25.0F};
  const cv::Point2f model{
    original.x * letterbox->scale + static_cast<float>(letterbox->pad_x),
    original.y * letterbox->scale + static_cast<float>(letterbox->pad_y)};
  const auto restored = model_point_to_image(model, *letterbox);
  ASSERT_TRUE(restored.has_value());
  EXPECT_NEAR(restored->x, original.x, 1e-5F);
  EXPECT_NEAR(restored->y, original.y, 1e-5F);
}

TEST(RawArmorDetection, ModelPointInverseRestoresAllFourArmorCorners)
{
  DetectorConfig config{};
  config.center_padding = true;
  cv::Mat image(240, 320, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto letterbox = make_letterbox(image, config);
  ASSERT_TRUE(letterbox.has_value());

  const std::array<cv::Point2f, 4> original{{
    {40.0F, 60.0F}, {160.0F, 60.0F}, {160.0F, 140.0F}, {40.0F, 140.0F}}};
  for (const auto & point : original) {
    const cv::Point2f model{
      point.x * letterbox->scale + static_cast<float>(letterbox->pad_x),
      point.y * letterbox->scale + static_cast<float>(letterbox->pad_y)};
    const auto restored = model_point_to_image(model, *letterbox);
    ASSERT_TRUE(restored.has_value());
    EXPECT_NEAR(restored->x, point.x, 1e-5F);
    EXPECT_NEAR(restored->y, point.y, 1e-5F);
  }
}

TEST(RawArmorDetection, ModelPointInverseRejectsInvalidScaleAndPoint)
{
  rm_auto_aim::detector::LetterboxResult invalid_scale{};
  invalid_scale.scale = 0.0F;
  EXPECT_FALSE(model_point_to_image({1.0F, 2.0F}, invalid_scale).has_value());

  DetectorConfig config{};
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto letterbox = make_letterbox(image, config);
  ASSERT_TRUE(letterbox.has_value());
  EXPECT_FALSE(model_point_to_image(
    {std::numeric_limits<float>::quiet_NaN(), 2.0F}, *letterbox).has_value());
}

TEST(RawArmorDetection, RejectsUnexpectedModelShapes)
{
  DetectorConfig config{};
  EXPECT_TRUE(validate_input_shape({1, 3, 320, 320}, config).has_value());
  EXPECT_TRUE(validate_output_shape({1, 25200, 21}, config).has_value());
  EXPECT_TRUE(validate_output_shape({1, 100, 22}, config).has_value());
  EXPECT_FALSE(validate_output_shape({1, 25200, 22}, config).has_value());
}

TEST(OpenVinoYoloDetector, MissingModelIsReportedClearly)
{
  DetectorConfig config{};
  config.model_path = "/definitely/not/a/game26/model.xml";
  try {
    rm_auto_aim::detector::OpenVinoYoloDetector detector(config);
    FAIL() << "expected missing model to throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("does not exist"), std::string::npos);
  }
}
