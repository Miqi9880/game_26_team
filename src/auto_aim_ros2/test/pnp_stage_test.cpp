#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/offline_pipeline.hpp"
#include "auto_aim_ros2/pnp_stage.hpp"
#include "auto_aim_ros2/raw_armor_detector.hpp"
#include "auto_aim_ros2/ros_adapters.hpp"
#include "auto_aim_ros2/ros_backend.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
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

rm_auto_aim::pipeline::ImageFrame make_synthetic_image_frame(
  const PnpConfiguration & config,
  std::int64_t stamp_ns)
{
  rm_auto_aim::pipeline::ImageFrame frame{};
  frame.stamp_ns = stamp_ns;
  frame.width = static_cast<std::uint32_t>(config.camera.image_width);
  frame.height = static_cast<std::uint32_t>(config.camera.image_height);
  frame.encoding = "bgr8";
  frame.bgr_image = cv::Mat::zeros(
    config.camera.image_height, config.camera.image_width, CV_8UC3);
  return frame;
}

std::optional<RawArmorDetection> detector_contract_fixture(
  const rm_auto_aim::pipeline::ImageFrame & frame,
  const PnpConfiguration & config)
{
  // This is a post-decoder fixture, not a fake inference result.  It only
  // connects a valid ImageFrame to the public RawArmorDetection contract so
  // the following PnP/tracker/selector/aimer test has no model artifact or
  // OpenVINO dependency.
  if (!frame.has_pixels()) {
    return std::nullopt;
  }
  const auto projected = make_synthetic_detection(config);
  const std::vector<cv::Point2f> keypoints(
    projected.keypoints.begin(), projected.keypoints.end());
  auto detection = rm_auto_aim::detector::make_raw_armor_detection(
    projected.class_id, projected.color_id, projected.confidence, projected.bbox, keypoints,
    9, 4);
  if (!detection.has_value()) {
    return std::nullopt;
  }
  // class 0 is explicitly small in both test-only fixtures.  The test does
  // not infer a physical armor size from an unknown class.
  detection->armor_type = RawArmorDetection::ArmorTypeHint::Small;
  return detection;
}

void expect_safe_offline_command(const rm_auto_aim::pipeline::AimCommand & command)
{
  EXPECT_FLOAT_EQ(command.yaw_rad, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_rad, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(command.yaw_acc_rad_s2, 0.0F);
  EXPECT_FLOAT_EQ(command.pitch_acc_rad_s2, 0.0F);
  EXPECT_EQ(command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(command.fire_command, rm_auto_aim::pipeline::kFireNone);
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

TEST(PnpConfiguration, TestOnlyFlagsMustMatchAcrossNestedConfiguration)
{
  auto config = load_test_configuration();
  config.camera.test_only = false;
  EXPECT_THROW(PnpStage{config}, std::invalid_argument);

  config = load_test_configuration();
  config.small_armor.test_only = false;
  EXPECT_THROW(PnpStage{config}, std::invalid_argument);

  config = load_test_configuration();
  config.large_armor.test_only = false;
  EXPECT_THROW(PnpStage{config}, std::invalid_argument);

  config = load_test_configuration();
  config.camera_to_gimbal.test_only = false;
  EXPECT_THROW(PnpStage{config}, std::invalid_argument);
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

TEST(OfflineApiSmoke, SyntheticImageFrameFlowsOnlyToDiagnosticAimerAndPredictor)
{
  const auto config = load_test_configuration();
  PnpStage pnp_stage(config);
  rm_auto_aim::offline::OfflineTracker tracker;
  rm_auto_aim::offline::TargetSelector selector;
  rm_auto_aim::offline::SafeOfflineAimer aimer;

  const auto first_frame = make_synthetic_image_frame(config, 1'000'000'000);
  ASSERT_TRUE(first_frame.has_pixels());
  const auto first_detection = detector_contract_fixture(first_frame, config);
  ASSERT_TRUE(first_detection.has_value());
  const auto first_pose = pnp_stage.solve(
    *first_detection, static_cast<int>(first_frame.width), static_cast<int>(first_frame.height));
  ASSERT_TRUE(first_pose.valid);

  const auto first_update = tracker.update(
    std::vector<rm_auto_aim::offline::TargetObservation>{
      rm_auto_aim::offline::make_target_observation(first_pose, first_frame.stamp_ns)},
    first_frame.stamp_ns);
  EXPECT_EQ(first_update.state, rm_auto_aim::offline::TrackingState::Detecting);
  EXPECT_FALSE(first_update.target_lock());
  EXPECT_FALSE(selector.select(
      first_update.tracks, static_cast<int>(first_frame.width),
      static_cast<int>(first_frame.height)).has_value());

  const auto second_frame = make_synthetic_image_frame(config, 1'010'000'000);
  const auto second_detection = detector_contract_fixture(second_frame, config);
  ASSERT_TRUE(second_detection.has_value());
  const auto second_pose = pnp_stage.solve(
    *second_detection, static_cast<int>(second_frame.width),
    static_cast<int>(second_frame.height));
  ASSERT_TRUE(second_pose.valid);

  const auto second_update = tracker.update(
    std::vector<rm_auto_aim::offline::TargetObservation>{
      rm_auto_aim::offline::make_target_observation(second_pose, second_frame.stamp_ns)},
    second_frame.stamp_ns);
  ASSERT_TRUE(second_update.target_lock());
  const auto selected_before = selector.select(
    second_update.tracks, static_cast<int>(second_frame.width),
    static_cast<int>(second_frame.height));
  ASSERT_TRUE(selected_before.has_value());
  const auto aimed_before = aimer.aim(selected_before);
  EXPECT_TRUE(aimed_before.test_only);
  EXPECT_EQ(aimed_before.target_lock, rm_auto_aim::pipeline::kTargetLocked);
  EXPECT_FALSE(aimed_before.absolute_command_valid);
  EXPECT_FALSE(aimed_before.command_yaw_rad.has_value());
  EXPECT_FALSE(aimed_before.command_pitch_rad.has_value());
  EXPECT_EQ(aimed_before.fire_command, rm_auto_aim::pipeline::kFireNone);
  const auto command_before = aimed_before.safe_command();
  expect_safe_offline_command(command_before);

  rm_auto_aim::offline::PredictionConfig prediction_config{};
  prediction_config.enabled = true;
  prediction_config.horizon_ns = 10'000'000;
  prediction_config.max_horizon_ns = 100'000'000;
  rm_auto_aim::offline::OfflinePredictor predictor(prediction_config);
  const auto prediction = predictor.predict(selected_before, second_frame.stamp_ns);
  ASSERT_TRUE(prediction.valid);
  EXPECT_TRUE(prediction.test_only);
  EXPECT_FALSE(prediction.production_ready);
  EXPECT_EQ(prediction.track_id, selected_before->track_id);

  // Predictor accepts a const selected target and is not a feedback path into
  // selection or aiming.  The same tracker evidence therefore yields the
  // same target and safety command after a prediction diagnostic.
  const auto selected_after = selector.select(
    second_update.tracks, static_cast<int>(second_frame.width),
    static_cast<int>(second_frame.height));
  ASSERT_TRUE(selected_after.has_value());
  EXPECT_EQ(selected_after->track_id, selected_before->track_id);
  const auto aimed_after = aimer.aim(selected_after);
  EXPECT_EQ(aimed_after.target_lock, aimed_before.target_lock);
  EXPECT_EQ(aimed_after.fire_command, aimed_before.fire_command);
  expect_safe_offline_command(aimed_after.safe_command());
}

TEST(OfflineApiSmoke, OutOfBoundsDetectorEvidenceFailsClosedBeforeTracking)
{
  const auto config = load_test_configuration();
  PnpStage pnp_stage(config);
  const auto frame = make_synthetic_image_frame(config, 1'000'000'000);
  auto detection = detector_contract_fixture(frame, config);
  ASSERT_TRUE(detection.has_value());
  detection->keypoints[0].x = -1.0F;

  const auto pose = pnp_stage.solve(
    *detection, static_cast<int>(frame.width), static_cast<int>(frame.height));
  EXPECT_FALSE(pose.valid);
  EXPECT_EQ(pose.failure, PoseFailure::KeypointOrderRejected);

  rm_auto_aim::offline::OfflineTracker tracker;
  const auto update = tracker.update(
    std::vector<rm_auto_aim::offline::TargetObservation>{
      rm_auto_aim::offline::make_target_observation(pose, frame.stamp_ns)},
    frame.stamp_ns);
  EXPECT_TRUE(update.rejected);
  EXPECT_FALSE(update.target_lock());
  EXPECT_TRUE(update.tracks.empty());

  rm_auto_aim::offline::TargetSelector selector;
  const auto selected = selector.select(
    update.tracks, static_cast<int>(frame.width), static_cast<int>(frame.height));
  EXPECT_FALSE(selected.has_value());
  const auto aimed = rm_auto_aim::offline::SafeOfflineAimer{}.aim(selected);
  EXPECT_EQ(aimed.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(aimed.fire_command, rm_auto_aim::pipeline::kFireNone);
  expect_safe_offline_command(aimed.safe_command());
}

TEST(OfflineApiSmoke, OfflineBackendRejectsUnsafeOperatingFlagsBeforeModelLoad)
{
  using rm_auto_aim::ros_backend::Backend;
  using rm_auto_aim::ros_backend::BackendKind;
  using rm_auto_aim::ros_backend::Config;

  Config config{};
  config.kind = BackendKind::OfflineReference;
  config.dry_run = false;
  EXPECT_THROW({Backend backend(config);}, std::invalid_argument);

  config.dry_run = true;
  config.serial_enabled = true;
  EXPECT_THROW({Backend backend(config);}, std::invalid_argument);

  config.serial_enabled = false;
  config.allow_fire = true;
  EXPECT_THROW({Backend backend(config);}, std::invalid_argument);
}

TEST(OfflineApiSmoke, OfflineBackendRejectsUnavailableModelArtifactBeforeAnyFrame)
{
  using rm_auto_aim::ros_backend::Backend;
  using rm_auto_aim::ros_backend::BackendKind;
  using rm_auto_aim::ros_backend::Config;

  Config config{};
  config.kind = BackendKind::OfflineReference;
  config.dry_run = true;
  config.serial_enabled = false;
  config.allow_fire = false;
  config.allow_test_only = true;
  config.pnp_config_path = PNP_TEST_CONFIG_PATH;
  config.model_profile_path = (
    std::filesystem::path(PNP_TEST_CONFIG_PATH).parent_path() / "model_profile_test.yaml").string();
  config.model_path = "/definitely/not/a/game26/model.xml";

  try {
    Backend backend(config);
    FAIL() << "expected unavailable model artifact to fail before frame processing";
  } catch (const std::runtime_error & error) {
    EXPECT_NE(std::string(error.what()).find("does not exist"), std::string::npos);
  }
}
