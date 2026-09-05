#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::detector::DetectorConfig;
using rm_auto_aim::detector::ModelProfileLoadOptions;
using rm_auto_aim::detector::make_letterbox;
using rm_auto_aim::detector::make_preprocessed_tensor;
using rm_auto_aim::detector::make_raw_armor_detection;
using rm_auto_aim::detector::model_point_to_image;
using rm_auto_aim::detector::load_model_profile;
using rm_auto_aim::detector::sigmoid_probability;
using rm_auto_aim::detector::validate_input_shape;
using rm_auto_aim::detector::validate_output_shape;
using rm_auto_aim::detector::validate_runtime_layout;

class ScopedTemporaryPath
{
public:
  explicit ScopedTemporaryPath(std::filesystem::path value) : path(std::move(value)) {}

  ~ScopedTemporaryPath()
  {
    std::error_code error;
    std::filesystem::remove(path, error);
  }

  std::filesystem::path path;
};

void write_test_artifact(const std::filesystem::path & path, const std::string & contents)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    throw std::runtime_error("cannot create temporary test artifact: " + path.string());
  }
  stream.write(contents.data(), static_cast<std::streamsize>(contents.size()));
  if (!stream) {
    throw std::runtime_error("cannot write temporary test artifact: " + path.string());
  }
}
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

TEST(RawArmorDetection, ImageFrameRejectsMismatchedPixelMetadata)
{
  rm_auto_aim::pipeline::ImageFrame frame{};
  frame.bgr_image = cv::Mat(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
  frame.width = 3;
  frame.height = 2;
  EXPECT_TRUE(frame.has_pixels());

  frame.width = 2;
  EXPECT_FALSE(frame.has_pixels());
  frame.width = 3;
  frame.height = 1;
  EXPECT_FALSE(frame.has_pixels());
}

TEST(RawArmorDetection, PreprocessingGoldenTensorIsExplicitRgbDivide255Nchw)
{
  DetectorConfig config{};
  config.input_width = 4;
  config.input_height = 2;
  cv::Mat image(2, 4, CV_8UC3);
  const std::array<cv::Vec3b, 8> bgr{{
    {10, 20, 30}, {40, 50, 60}, {70, 80, 90}, {100, 110, 120},
    {130, 140, 150}, {160, 170, 180}, {190, 200, 210}, {220, 230, 240}}};
  for (int row = 0; row < image.rows; ++row) {
    for (int column = 0; column < image.cols; ++column) {
      image.at<cv::Vec3b>(row, column) = bgr[static_cast<std::size_t>(row * image.cols + column)];
    }
  }

  const auto result = make_preprocessed_tensor(image, config);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->nchw_rgb_f32.size(), 24U);
  EXPECT_FLOAT_EQ(result->letterbox.scale, 1.0F);
  EXPECT_EQ(result->letterbox.pad_x, 0);
  EXPECT_EQ(result->letterbox.pad_y, 0);

  const std::array<int, 24> expected_rgb{{
    30, 60, 90, 120, 150, 180, 210, 240,
    20, 50, 80, 110, 140, 170, 200, 230,
    10, 40, 70, 100, 130, 160, 190, 220}};
  for (std::size_t index = 0; index < expected_rgb.size(); ++index) {
    EXPECT_FLOAT_EQ(
      result->nchw_rgb_f32[index], static_cast<float>(expected_rgb[index]) / 255.0F)
      << "NCHW tensor index " << index;
  }
}

TEST(RawArmorDetection, PreprocessingPreservesConfiguredTopLeftAndCenterPadding)
{
  cv::Mat image(2, 4, CV_8UC3, cv::Scalar(9, 18, 27));
  DetectorConfig config{};
  config.input_width = 6;
  config.input_height = 6;

  const auto top_left = make_preprocessed_tensor(image, config);
  ASSERT_TRUE(top_left.has_value());
  EXPECT_EQ(top_left->letterbox.pad_x, 0);
  EXPECT_EQ(top_left->letterbox.pad_y, 0);
  EXPECT_FLOAT_EQ(top_left->nchw_rgb_f32[0], 27.0F / 255.0F);
  EXPECT_FLOAT_EQ(top_left->nchw_rgb_f32[3U * 6U], 0.0F);

  config.center_padding = true;
  const auto center = make_preprocessed_tensor(image, config);
  ASSERT_TRUE(center.has_value());
  EXPECT_EQ(center->letterbox.pad_x, 0);
  EXPECT_EQ(center->letterbox.pad_y, 1);
  EXPECT_FLOAT_EQ(center->nchw_rgb_f32[0], 0.0F);
  EXPECT_FLOAT_EQ(center->nchw_rgb_f32[6U], 27.0F / 255.0F);
  EXPECT_FLOAT_EQ(center->nchw_rgb_f32[4U * 6U], 0.0F);
}

TEST(RawArmorDetection, PreprocessingRejectsEmptyZeroSizeAndUnsupportedInputs)
{
  DetectorConfig config{};
  EXPECT_FALSE(make_preprocessed_tensor(cv::Mat{}, config).has_value());

  cv::Mat mono(2, 2, CV_8UC1, cv::Scalar(1));
  EXPECT_FALSE(make_preprocessed_tensor(mono, config).has_value());
  cv::Mat floating(2, 2, CV_32FC3, cv::Scalar(1.0F, 2.0F, 3.0F));
  EXPECT_FALSE(make_preprocessed_tensor(floating, config).has_value());

  config.input_width = 0;
  cv::Mat bgr(2, 2, CV_8UC3, cv::Scalar(1, 2, 3));
  EXPECT_FALSE(make_preprocessed_tensor(bgr, config).has_value());
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
  EXPECT_FLOAT_EQ(result->scale_x, 3.2F);
  EXPECT_FLOAT_EQ(result->scale_y, 3.2F);
  EXPECT_EQ(result->resized_width, 640);
  EXPECT_EQ(result->resized_height, 320);
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
    original.x * letterbox->scale_x + static_cast<float>(letterbox->pad_x),
    original.y * letterbox->scale_y + static_cast<float>(letterbox->pad_y)};
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
    original.x * letterbox->scale_x + static_cast<float>(letterbox->pad_x),
    original.y * letterbox->scale_y + static_cast<float>(letterbox->pad_y)};
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
      point.x * letterbox->scale_x + static_cast<float>(letterbox->pad_x),
      point.y * letterbox->scale_y + static_cast<float>(letterbox->pad_y)};
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
  EXPECT_FALSE(model_point_to_image(
    {std::numeric_limits<float>::infinity(), 2.0F}, *letterbox).has_value());
}

TEST(RawArmorDetection, CheckedModelPointInverseRejectsPointsOutsideSourceImage)
{
  DetectorConfig config{};
  cv::Mat image(100, 200, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto letterbox = make_letterbox(image, config);
  ASSERT_TRUE(letterbox.has_value());

  const auto inside = model_point_to_image({100.0F, 100.0F}, *letterbox, image.size());
  ASSERT_TRUE(inside.has_value());
  EXPECT_NEAR(inside->x, 31.25F, 1e-5F);
  EXPECT_NEAR(inside->y, 31.25F, 1e-5F);
  EXPECT_FALSE(model_point_to_image({-1.0F, 0.0F}, *letterbox, image.size()).has_value());
  EXPECT_FALSE(model_point_to_image(
    {640.0F, 0.0F}, *letterbox, image.size()).has_value());
  EXPECT_FALSE(model_point_to_image({0.0F, 0.0F}, *letterbox, cv::Size{}).has_value());
}

TEST(RawArmorDetection, CheckedInverseRejectsIntegerRoundedPaddingForBothModes)
{
  DetectorConfig config{};
  cv::Mat image(333, 1000, CV_8UC3, cv::Scalar(3, 4, 5));
  const auto top_left = make_letterbox(image, config);
  ASSERT_TRUE(top_left.has_value());
  EXPECT_FLOAT_EQ(top_left->scale, 0.64F);
  EXPECT_EQ(top_left->resized_width, 640);
  EXPECT_EQ(top_left->resized_height, 213);
  EXPECT_LT(top_left->scale_y, top_left->scale);

  const cv::Point2f original{500.0F, 100.0F};
  const cv::Point2f model{
    original.x * top_left->scale_x + static_cast<float>(top_left->pad_x),
    original.y * top_left->scale_y + static_cast<float>(top_left->pad_y)};
  const auto restored = model_point_to_image(model, *top_left, image.size());
  ASSERT_TRUE(restored.has_value());
  EXPECT_NEAR(restored->x, original.x, 1e-4F);
  EXPECT_NEAR(restored->y, original.y, 1e-4F);
  EXPECT_FALSE(model_point_to_image({0.0F, 213.0F}, *top_left, image.size()).has_value());

  config.center_padding = true;
  const auto center = make_letterbox(image, config);
  ASSERT_TRUE(center.has_value());
  EXPECT_EQ(center->pad_y, 213);
  EXPECT_FALSE(model_point_to_image({0.0F, 0.0F}, *center, image.size()).has_value());
  EXPECT_FALSE(model_point_to_image({0.0F, 426.0F}, *center, image.size()).has_value());
}

TEST(RawArmorDetection, RejectsUnexpectedModelShapes)
{
  DetectorConfig config{};
  EXPECT_TRUE(validate_input_shape({1, 3, 320, 320}, config).has_value());
  EXPECT_TRUE(validate_output_shape({1, 25200, 21}, config).has_value());
  EXPECT_TRUE(validate_output_shape({1, 100, 22}, config).has_value());
  EXPECT_FALSE(validate_output_shape({1, 25200, 22}, config).has_value());
}

TEST(RawArmorDetection, RuntimeLayoutsRequireExplicitMatchingAxisOrder)
{
  EXPECT_FALSE(validate_runtime_layout("[N,C,H,W]", "NCHW", "input").has_value());
  EXPECT_FALSE(validate_runtime_layout("[N,R,C]", "NRC", "output").has_value());
  EXPECT_FALSE(validate_runtime_layout("NHWC", {}, "input").has_value());

  const auto missing = validate_runtime_layout({}, "NCHW", "input");
  ASSERT_TRUE(missing.has_value());
  EXPECT_NE(missing->find("unavailable"), std::string::npos);

  const auto mismatch = validate_runtime_layout("[N,H,W,C]", "NCHW", "input");
  ASSERT_TRUE(mismatch.has_value());
  EXPECT_NE(mismatch->find("does not match profile"), std::string::npos);
}

TEST(ModelProfile, TestOnlyRequiresExplicitOptIn)
{
  EXPECT_THROW(load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH), std::runtime_error);
  ModelProfileLoadOptions options{};
  options.allow_test_only = true;
  const auto profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  EXPECT_TRUE(profile.test_only);
  EXPECT_EQ(profile.schema_version, 1);
  EXPECT_EQ(profile.model_id, "legacy_yolov5_reference");
  EXPECT_EQ(profile.model_format, "legacy_single_path");
  EXPECT_EQ(profile.model_artifacts.xml.path, "external://sp_vision_25/assets/yolov5.xml");
  EXPECT_FALSE(profile.model_artifacts.bin.sha256.has_value());
  EXPECT_TRUE(profile.model_artifacts.bin.path.empty());
  EXPECT_EQ(profile.input_shape, (std::array<std::size_t, 4>{{1, 3, 640, 640}}));
  EXPECT_EQ(profile.output_shape, (std::array<std::size_t, 3>{{1, 25200, 22}}));
  EXPECT_EQ(profile.color_class_count, 4U);
  EXPECT_EQ(profile.armor_class_count, 9U);
  EXPECT_EQ(profile.keypoint_order, (std::array<int, 4>{{0, 3, 2, 1}}));
  ASSERT_EQ(profile.class_to_armor_type.size(), 9U);
}

TEST(ModelProfile, ConvertsOnlyValidatedProfileToDetectorConfig)
{
  ModelProfileLoadOptions options{};
  options.allow_test_only = true;
  const auto profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  const auto config = rm_auto_aim::detector::detector_config_from_model_profile(
    profile, "/tmp/reference.xml");
  EXPECT_EQ(config.model_path, "/tmp/reference.xml");
  EXPECT_TRUE(config.model_bin_path.empty());
  EXPECT_EQ(config.input_width, 640);
  EXPECT_EQ(config.input_height, 640);
  EXPECT_EQ(config.expected_output_rows, 25200U);
  EXPECT_EQ(config.expected_output_columns, 22U);
  EXPECT_EQ(config.expected_input_element_type, "f32");
  EXPECT_EQ(config.expected_output_element_type, "f32");
  EXPECT_EQ(config.expected_input_layout, "NCHW");
  EXPECT_EQ(config.expected_output_layout, "NRC");
  EXPECT_EQ(config.color_logits_offset, 9U);
  EXPECT_EQ(config.armor_logits_offset, 13U);
  ASSERT_EQ(config.class_to_armor_type.size(), 9U);
  EXPECT_EQ(
    config.class_to_armor_type.at(0),
    rm_auto_aim::detector::RawArmorDetection::ArmorTypeHint::Small);
  EXPECT_FALSE(config.center_padding);
}

TEST(ModelProfile, SchemaV2ParsesAndBindsExplicitXmlBinManifest)
{
  const auto unique_suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  ScopedTemporaryPath fixture{
    std::filesystem::temp_directory_path() /
      ("game26_model_profile_v2_manifest_" + unique_suffix + ".yaml")};
  {
    std::ofstream yaml(fixture.path);
    ASSERT_TRUE(yaml.is_open());
    yaml << R"(schema_version: 2
profile: test_only
model:
  id: test_only_openvino_ir
  format: openvino_ir
  artifacts:
    xml:
      path: external://test-fixtures/reviewed.xml
      sha256: aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
    bin:
      path: external://test-fixtures/reviewed.bin
      sha256: bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb
  source: synthetic manifest fixture
  version: fixture-v2
input:
  shape: [1, 3, 640, 640]
  layout: NCHW
  element_type: f32
  source_color_order: BGR
  model_color_order: RGB
  normalization: divide_255
  resize_mode: top_left
output:
  shape: [1, 25200, 22]
  layout: NRC
  element_type: f32
  keypoint_count: 4
  objectness_index: 8
  color_logits_offset: 9
  color_class_count: 4
  armor_logits_offset: 13
  armor_class_count: 9
postprocess:
  objectness_threshold: 0.7
  nms_threshold: 0.3
  keypoint_order: [0, 3, 2, 1]
semantics:
  color_id_to_name: [blue, red, gray, purple]
  armor_class_names: [hero, engineer, infantry3, infantry4, infantry5, sentry, outpost, base, unknown]
  class_to_armor_type:
    0: small
    1: small
    2: small
    3: small
    4: small
    5: large
    6: large
    7: large
    8: small
)";
  }

  ModelProfileLoadOptions options{};
  options.allow_test_only = true;
  const auto profile = load_model_profile(fixture.path.string(), options);
  EXPECT_TRUE(profile.test_only);
  EXPECT_EQ(profile.schema_version, 2);
  EXPECT_EQ(profile.model_format, "openvino_ir");
  EXPECT_EQ(profile.model_artifacts.xml.path, "external://test-fixtures/reviewed.xml");
  EXPECT_EQ(profile.model_artifacts.bin.path, "external://test-fixtures/reviewed.bin");
  ASSERT_TRUE(profile.model_artifacts.xml.sha256.has_value());
  ASSERT_TRUE(profile.model_artifacts.bin.sha256.has_value());

  const auto config = rm_auto_aim::detector::detector_config_from_model_profile(
    profile, "/tmp/test-only.xml", "CPU", "/tmp/test-only.bin");
  EXPECT_EQ(config.model_path, "/tmp/test-only.xml");
  EXPECT_EQ(config.model_bin_path, "/tmp/test-only.bin");
  EXPECT_FALSE(config.require_model_path_match);
  EXPECT_FALSE(config.require_model_bin_path_match);
  EXPECT_FALSE(config.require_model_hash_match);
}

TEST(ModelProfile, ProductionProfileBindsRuntimePathToReviewedArtifact)
{
  ModelProfileLoadOptions options{};
  options.allow_test_only = true;
  auto profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.test_only = false;
  profile.schema_version = 2;
  profile.model_format = "openvino_ir";
  profile.model_artifacts.xml.path = "/opt/game26/models/reviewed.xml";
  profile.model_artifacts.xml.sha256 = std::string(64, 'a');
  profile.model_artifacts.bin.path = "/opt/game26/models/reviewed.bin";
  profile.model_artifacts.bin.sha256 = std::string(64, 'b');

  EXPECT_FALSE(profile.validate().has_value());
  EXPECT_THROW(
    rm_auto_aim::detector::detector_config_from_model_profile(
      profile, "/opt/game26/models/other.xml", "CPU", "/opt/game26/models/reviewed.bin"),
    std::invalid_argument);
  EXPECT_THROW(
    rm_auto_aim::detector::detector_config_from_model_profile(
      profile, "/opt/game26/models/reviewed.xml", "CPU", "/opt/game26/models/other.bin"),
    std::invalid_argument);

  const auto config = rm_auto_aim::detector::detector_config_from_model_profile(
    profile, "/opt/game26/models/reviewed.xml");
  EXPECT_TRUE(config.require_model_path_match);
  EXPECT_EQ(config.reviewed_model_path, profile.model_artifacts.xml.path);
  EXPECT_TRUE(config.require_model_bin_path_match);
  EXPECT_EQ(config.model_bin_path, profile.model_artifacts.bin.path);
  EXPECT_EQ(config.reviewed_model_bin_path, profile.model_artifacts.bin.path);
  EXPECT_TRUE(config.require_model_hash_match);
  EXPECT_EQ(config.reviewed_model_sha256, *profile.model_artifacts.xml.sha256);
  EXPECT_EQ(config.reviewed_model_bin_sha256, *profile.model_artifacts.bin.sha256);

  profile.model_artifacts.xml.path = "relative/reviewed.xml";
  EXPECT_TRUE(profile.validate().has_value());

  profile.model_artifacts.xml.path = "/opt/game26/models/reviewed.xml";
  profile.model_artifacts.xml.sha256.reset();
  EXPECT_TRUE(profile.validate().has_value());

  profile.model_artifacts.xml.sha256 = "not-a-sha256";
  EXPECT_TRUE(profile.validate().has_value());

  profile.model_artifacts.xml.sha256 = std::string(64, 'a');
  profile.model_artifacts.bin.sha256.reset();
  EXPECT_TRUE(profile.validate().has_value());

  profile.model_artifacts.bin.sha256 = "not-a-sha256";
  EXPECT_TRUE(profile.validate().has_value());
}

TEST(ModelProfile, RejectsNonFiniteObjectnessLogits)
{
  EXPECT_FALSE(sigmoid_probability(std::numeric_limits<float>::infinity()).has_value());
  EXPECT_FALSE(sigmoid_probability(-std::numeric_limits<float>::infinity()).has_value());
  EXPECT_FALSE(sigmoid_probability(std::numeric_limits<float>::quiet_NaN()).has_value());
  ASSERT_TRUE(sigmoid_probability(0.0F).has_value());
  EXPECT_FLOAT_EQ(*sigmoid_probability(0.0F), 0.5F);
}

TEST(ModelProfile, RejectsInvalidTensorOrSemanticContract)
{
  ModelProfileLoadOptions options{};
  options.allow_test_only = true;
  auto profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);

  profile.output_layout = "NHWC";
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.keypoint_order[1] = profile.keypoint_order[0];
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.class_to_armor_type.erase(8);
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.input_shape[2] = static_cast<std::size_t>(std::numeric_limits<int>::max()) + 1U;
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.input_element_type = "u8";
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.normalization = "none";
  EXPECT_TRUE(profile.validate().has_value());

  profile = load_model_profile(MODEL_PROFILE_TEST_CONFIG_PATH, options);
  profile.resize_mode = "implicit";
  EXPECT_TRUE(profile.validate().has_value());
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

TEST(OpenVinoYoloDetector, ReviewedManifestMissingBinFailsClosedBeforeRuntimeLoad)
{
  DetectorConfig config{};
  config.model_path = MODEL_PROFILE_TEST_CONFIG_PATH;
  config.model_bin_path = "/definitely/not/a/game26/model.bin";
  config.require_model_bin_path_match = true;
  config.reviewed_model_bin_path = config.model_bin_path;
  try {
    rm_auto_aim::detector::OpenVinoYoloDetector detector(config);
    FAIL() << "expected missing reviewed BIN to throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("BIN file does not exist"), std::string::npos);
  }
}

TEST(OpenVinoYoloDetector, ReviewedBinHashMismatchFailsBeforeOpenVinoParsesXml)
{
  const auto unique_suffix = std::to_string(
    std::chrono::steady_clock::now().time_since_epoch().count());
  ScopedTemporaryPath xml{
    std::filesystem::temp_directory_path() /
      ("game26_manifest_hash_" + unique_suffix + ".xml")};
  ScopedTemporaryPath bin{
    std::filesystem::temp_directory_path() /
      ("game26_manifest_hash_" + unique_suffix + ".bin")};
  // The reviewed BIN digest below belongs to "original-bin". Replacing the
  // weights after the XML declaration is accepted must still fail before an
  // invalid XML graph reaches OpenVINO.
  write_test_artifact(xml.path, "<not-a-model/>");
  write_test_artifact(bin.path, "original-bin");

  DetectorConfig config{};
  config.model_path = xml.path.string();
  config.model_bin_path = bin.path.string();
  config.require_model_hash_match = true;
  config.reviewed_model_sha256 =
    "1f3268a67ab26e216887a191b995332088421f25b32b9b117ec31aea9e293fda";
  config.reviewed_model_bin_sha256 =
    "2b273dd96b861dc397e16656794c5b146fc7ae16c87bd45b1989d252924e5d0d";
  write_test_artifact(bin.path, "replaced-bin");

  try {
    rm_auto_aim::detector::OpenVinoYoloDetector detector(config);
    FAIL() << "expected reviewed BIN digest mismatch to throw";
  } catch (const std::runtime_error & error) {
    const std::string message(error.what());
    EXPECT_NE(message.find("SHA-256"), std::string::npos);
    // A build without OpenSSL correctly fails closed before it can verify any
    // digest. When the verifier is present, the required regression is that
    // the BIN failure is observed before OpenVINO can parse the invalid XML.
    if (message.find("verification is unavailable") == std::string::npos) {
      EXPECT_NE(message.find("model BIN artifact SHA-256 does not match"), std::string::npos);
    }
  }
}

TEST(OpenVinoYoloDetector, ReviewedHashMismatchIsReportedBeforeModelLoad)
{
  DetectorConfig config{};
  config.model_path = MODEL_PROFILE_TEST_CONFIG_PATH;
  config.model_bin_path = MODEL_PROFILE_TEST_CONFIG_PATH;
  config.require_model_hash_match = true;
  config.reviewed_model_sha256 = std::string(64, '0');
  config.reviewed_model_bin_sha256 = std::string(64, '0');
  try {
    rm_auto_aim::detector::OpenVinoYoloDetector detector(config);
    FAIL() << "expected reviewed hash mismatch to throw";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("SHA-256"), std::string::npos);
  }
}
