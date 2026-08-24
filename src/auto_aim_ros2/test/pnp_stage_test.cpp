#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/pnp_stage.hpp"
#include "auto_aim_ros2/ros_adapters.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>

namespace
{
using rm_auto_aim::detector::RawArmorDetection;
using rm_auto_aim::pnp::ConfigLoadOptions;
using rm_auto_aim::pnp::PnpConfiguration;
using rm_auto_aim::pnp::PnpStage;
using rm_auto_aim::pnp::PoseFailure;

PnpConfiguration load_test_configuration()
{
  ConfigLoadOptions options{};
  options.allow_test_only = true;
  return rm_auto_aim::pnp::load_pnp_configuration(PNP_TEST_CONFIG_PATH, options);
}

RawArmorDetection make_synthetic_detection(
  const PnpConfiguration & config,
  const cv::Vec3d & rvec = {0.04, -0.03, 0.02},
  const cv::Vec3d & tvec = {0.15, -0.07, 3.40})
{
  const std::vector<cv::Point3d> object_points(
    config.small_armor.object_points_m.begin(), config.small_armor.object_points_m.end());
  const cv::Mat distortion(config.camera.distortion_coefficients);
  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(
    object_points, rvec, tvec, config.camera.camera_matrix, distortion, projected_points);
  EXPECT_EQ(projected_points.size(), 4U);
  std::vector<cv::Point2f> image_points;
  image_points.reserve(projected_points.size());
  for (const auto & point : projected_points) {
    image_points.emplace_back(static_cast<float>(point.x), static_cast<float>(point.y));
  }

  float min_x = image_points.front().x;
  float max_x = image_points.front().x;
  float min_y = image_points.front().y;
  float max_y = image_points.front().y;
  for (const auto & point : image_points) {
    min_x = std::min(min_x, point.x);
    max_x = std::max(max_x, point.x);
    min_y = std::min(min_y, point.y);
    max_y = std::max(max_y, point.y);
  }

  RawArmorDetection detection{};
  detection.class_id = 0;
  detection.color_id = 0;
  detection.armor_type = RawArmorDetection::ArmorTypeHint::Unknown;
  detection.confidence = 0.95F;
  detection.bbox = {min_x - 1.0F, min_y - 1.0F, max_x - min_x + 2.0F, max_y - min_y + 2.0F};
  std::copy(image_points.begin(), image_points.end(), detection.keypoints.begin());
  return detection;
}

std::unique_ptr<rm_auto_aim::pipeline::AutoAimPipeline> make_null_pipeline()
{
  return std::make_unique<rm_auto_aim::pipeline::AutoAimPipeline>(
    std::make_unique<rm_auto_aim::pipeline::NullYoloStage>(),
    std::make_unique<rm_auto_aim::pipeline::PassThroughArmorStage>(),
    std::make_unique<rm_auto_aim::pipeline::LatestTargetTracker>(),
    std::make_unique<rm_auto_aim::pipeline::FirstTargetStage>(),
    std::make_unique<rm_auto_aim::pipeline::CommandAimer>());
}
}  // namespace

TEST(PnpConfiguration, TestOnlyRequiresExplicitOptIn)
{
  EXPECT_THROW(rm_auto_aim::pnp::load_pnp_configuration(PNP_TEST_CONFIG_PATH), std::runtime_error);
  const auto config = load_test_configuration();
  EXPECT_TRUE(config.test_only);
  EXPECT_EQ(config.camera.image_width, 1440);
  EXPECT_EQ(config.camera.image_height, 1080);
  EXPECT_TRUE(config.camera_to_gimbal.configured);
}

TEST(PnpConfiguration, ProductionProfileCannotOmitCameraToGimbalExtrinsic)
{
  auto config = load_test_configuration();
  config.test_only = false;
  config.camera.test_only = false;
  config.small_armor.test_only = false;
  config.large_armor.test_only = false;
  config.camera_to_gimbal.test_only = false;
  config.camera_to_gimbal.configured = false;
  EXPECT_THROW(PnpStage{config}, std::invalid_argument);

  config.camera_to_gimbal.configured = true;
  EXPECT_NO_THROW(PnpStage{config});
}

TEST(PnpStage, SyntheticProjectionRecoversCameraTranslationInMetres)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  const cv::Vec3d expected_tvec{0.15, -0.07, 3.40};
  const auto observation = stage.solve(make_synthetic_detection(config));

  ASSERT_TRUE(observation.valid);
  EXPECT_EQ(observation.failure, PoseFailure::None);
  EXPECT_NEAR(observation.translation_in_camera_m[0], expected_tvec[0], 1e-3);
  EXPECT_NEAR(observation.translation_in_camera_m[1], expected_tvec[1], 1e-3);
  EXPECT_NEAR(observation.translation_in_camera_m[2], expected_tvec[2], 1e-3);
  EXPECT_LT(observation.reprojection_error_px, 1e-3);
}

TEST(PnpStage, RejectsMismatchedImageDimensionsWithoutRescalingIntrinsics)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  const auto observation = stage.solve(make_synthetic_detection(config), 1439, 1080);
  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(observation.failure, PoseFailure::ImageDimensionsMismatch);
}

TEST(PnpStage, RejectsCrossedAndMirroredKeypointOrder)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  const auto detection = make_synthetic_detection(config);

  auto crossed = detection;
  std::swap(crossed.keypoints[1], crossed.keypoints[2]);
  const auto crossed_observation = stage.solve(crossed);
  EXPECT_FALSE(crossed_observation.valid);
  EXPECT_EQ(crossed_observation.failure, PoseFailure::KeypointOrderRejected);

  auto mirrored = detection;
  std::swap(mirrored.keypoints[1], mirrored.keypoints[3]);
  const auto mirrored_observation = stage.solve(mirrored);
  EXPECT_FALSE(mirrored_observation.valid);
  EXPECT_EQ(mirrored_observation.failure, PoseFailure::KeypointOrderRejected);
}

TEST(PnpStage, RejectsInvalidRawFields)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  const auto detection = make_synthetic_detection(config);

  auto nan_keypoint = detection;
  nan_keypoint.keypoints[0].x = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(stage.solve(nan_keypoint).failure, PoseFailure::InvalidRawDetection);

  auto invalid_confidence = detection;
  invalid_confidence.confidence = 1.01F;
  EXPECT_EQ(stage.solve(invalid_confidence).failure, PoseFailure::InvalidRawDetection);

  auto invalid_bbox = detection;
  invalid_bbox.bbox.width = -1.0F;
  EXPECT_EQ(stage.solve(invalid_bbox).failure, PoseFailure::InvalidRawDetection);
}

TEST(PnpStage, InvalidCalibrationDistortionAndGeometryAreRejected)
{
  auto invalid_camera = load_test_configuration();
  invalid_camera.camera.camera_matrix(0, 0) = 0.0;
  EXPECT_THROW(PnpStage{invalid_camera}, std::invalid_argument);

  auto invalid_camera_bottom_row = load_test_configuration();
  invalid_camera_bottom_row.camera.camera_matrix(2, 2) = 2.0;
  EXPECT_THROW(PnpStage{invalid_camera_bottom_row}, std::invalid_argument);

  auto invalid_distortion = load_test_configuration();
  invalid_distortion.camera.distortion_coefficients.clear();
  EXPECT_THROW(PnpStage{invalid_distortion}, std::invalid_argument);

  auto invalid_geometry = load_test_configuration();
  invalid_geometry.small_armor.width_m = 0.0;
  EXPECT_THROW(PnpStage{invalid_geometry}, std::invalid_argument);

  auto invalid_extrinsic = load_test_configuration();
  invalid_extrinsic.camera_to_gimbal.rotation_gimbal_from_camera(0, 0) = 2.0;
  EXPECT_THROW(PnpStage{invalid_extrinsic}, std::invalid_argument);
}

TEST(PnpStage, UnknownClassWithoutArmorHintDoesNotGuessGeometry)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  auto detection = make_synthetic_detection(config);
  detection.class_id = 999;
  detection.armor_type = RawArmorDetection::ArmorTypeHint::Unknown;
  const auto observation = stage.solve(detection);
  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(observation.failure, PoseFailure::GeometryNotConfigured);
}

TEST(PnpStage, ConflictingModelHintAndPnpClassMappingFailClosed)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  auto detection = make_synthetic_detection(config);
  // The synthetic PnP profile maps class 0 to small.  A reviewed detector
  // profile hint that says large must not silently select the wrong geometry.
  detection.armor_type = RawArmorDetection::ArmorTypeHint::Large;
  const auto observation = stage.solve(detection);
  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(observation.failure, PoseFailure::GeometrySemanticConflict);
}

TEST(PnpStage, ReprojectionThresholdFailsClosed)
{
  auto config = load_test_configuration();
  config.max_reprojection_error_px = 0.01;
  PnpStage stage(config);
  auto detection = make_synthetic_detection(config);
  detection.keypoints[2].x += 40.0F;
  const auto observation = stage.solve(detection);
  EXPECT_FALSE(observation.valid);
  EXPECT_EQ(observation.failure, PoseFailure::ReprojectionErrorTooLarge);
  EXPECT_GT(observation.reprojection_error_px, config.max_reprojection_error_px);
}

TEST(PnpStage, OmittedExtrinsicLeavesOnlyCameraPose)
{
  auto config = load_test_configuration();
  config.camera_to_gimbal.configured = false;
  PnpStage stage(config);
  const auto observation = stage.solve(make_synthetic_detection(config));
  ASSERT_TRUE(observation.valid);
  EXPECT_FALSE(observation.translation_in_gimbal_m.has_value());
  EXPECT_FALSE(observation.rotation_gimbal_from_armor.has_value());
  EXPECT_FALSE(observation.relative_angles_in_gimbal.has_value());
}

TEST(PnpStage, SyntheticExtrinsicTransformsCameraPoseAndAngles)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  const cv::Vec3d expected_camera{0.15, -0.07, 3.40};
  const auto observation = stage.solve(make_synthetic_detection(config));

  ASSERT_TRUE(observation.valid);
  ASSERT_TRUE(observation.translation_in_gimbal_m.has_value());
  const auto expected_gimbal =
    config.camera_to_gimbal.rotation_gimbal_from_camera * expected_camera +
    config.camera_to_gimbal.translation_gimbal_from_camera_m;
  for (int index = 0; index < 3; ++index) {
    EXPECT_NEAR(observation.translation_in_gimbal_m->operator[](index), expected_gimbal[index], 1e-3);
  }
  ASSERT_TRUE(observation.relative_angles_in_gimbal.has_value());
  EXPECT_NEAR(
    observation.relative_angles_in_gimbal->relative_yaw_rad,
    std::atan2(expected_gimbal[1], expected_gimbal[0]), 1e-3);
  EXPECT_NEAR(
    observation.relative_angles_in_gimbal->relative_pitch_rad,
    std::atan2(expected_gimbal[2], std::hypot(expected_gimbal[0], expected_gimbal[1])), 1e-3);
}

TEST(PnpStage, RelativeAngleSignsFollowConfirmedGimbalConvention)
{
  const auto left = PnpStage::relative_angles_from_gimbal_translation({2.0, 1.0, 0.0});
  ASSERT_TRUE(left.has_value());
  EXPECT_GT(left->relative_yaw_rad, 0.0);

  const auto up = PnpStage::relative_angles_from_gimbal_translation({2.0, 0.0, 1.0});
  ASSERT_TRUE(up.has_value());
  EXPECT_GT(up->relative_pitch_rad, 0.0);
}

TEST(PnpStage, InvalidPnpCannotCreateLockedOrFireControlOutput)
{
  const auto config = load_test_configuration();
  PnpStage stage(config);
  auto invalid_detection = make_synthetic_detection(config);
  std::swap(invalid_detection.keypoints[1], invalid_detection.keypoints[2]);
  EXPECT_FALSE(stage.solve(invalid_detection).valid);

  // PnP is deliberately not connected to the control pipeline in this phase.
  // The default pipeline remains a null detector and emits the safe command.
  const auto pipeline = make_null_pipeline();
  const auto command = pipeline->process({}, std::chrono::steady_clock::now());
  EXPECT_EQ(command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(command.fire_command, rm_auto_aim::pipeline::kFireNone);

  rm_auto_aim::pipeline::AimCommand unsafe{};
  unsafe.fire_command = rm_auto_aim::pipeline::kFireBurst;
  EXPECT_EQ(
    rm_auto_aim::ros_adapters::force_dry_run_safe(unsafe).fire_command,
    rm_auto_aim::pipeline::kFireNone);
}
