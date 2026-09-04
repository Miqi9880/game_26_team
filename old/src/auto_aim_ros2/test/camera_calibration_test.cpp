#include "auto_aim_ros2/camera_calibration.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

namespace calibration = rm_auto_aim::camera_calibration;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    static std::uint64_t sequence = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("auto_aim_camera_calibration_test_" + std::to_string(++sequence));
    std::filesystem::create_directories(path_);
  }

  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept { return path_; }

private:
  std::filesystem::path path_;
};

calibration::CalibrationInput load_fixture()
{
  return calibration::load_calibration_input(CAMERA_CALIBRATION_TEST_CONFIG_PATH);
}

std::vector<cv::Point3d> make_board_points(const calibration::CalibrationInput & input)
{
  std::vector<cv::Point3d> points;
  for (int row = 0; row < input.board.inner_corners_rows; ++row) {
    for (int col = 0; col < input.board.inner_corners_cols; ++col) {
      points.emplace_back(
        col * input.board.square_size_m,
        row * input.board.square_size_m, 0.0);
    }
  }
  return points;
}

std::vector<calibration::DetectedChessboardView> make_synthetic_views(
  const calibration::CalibrationInput & input,
  int count = 15,
  double noise_sigma_px = 0.0)
{
  const cv::Matx33d camera_matrix{
    820.0, 0.0, 320.0,
    0.0, 805.0, 240.0,
    0.0, 0.0, 1.0};
  const std::vector<double> distortion{0.025, -0.012, 0.001, -0.0007, 0.002};
  const auto board_points = make_board_points(input);
  std::mt19937 generator(42);
  std::normal_distribution<double> noise(0.0, noise_sigma_px);
  std::vector<calibration::DetectedChessboardView> views;
  views.reserve(count);
  for (int index = 0; index < count; ++index) {
    const cv::Vec3d rvec{
      0.08 + 0.012 * index,
      -0.10 + 0.018 * (index % 5),
      0.015 * ((index % 4) - 1.5)};
    const cv::Vec3d tvec{
      -0.20 + 0.032 * (index % 6),
      -0.12 + 0.025 * ((index * 2) % 7),
      2.2 + 0.14 * (index % 5)};
    std::vector<cv::Point2d> projected;
    cv::projectPoints(
      board_points, rvec, tvec, camera_matrix, cv::Mat(distortion), projected);
    std::vector<cv::Point2f> corners;
    corners.reserve(projected.size());
    for (const auto & point : projected) {
      corners.emplace_back(
        static_cast<float>(point.x + noise(generator)),
        static_cast<float>(point.y + noise(generator)));
    }
    views.push_back(calibration::DetectedChessboardView{
      "synthetic_view_" + std::to_string(index) + ".png",
      input.image.width_px, input.image.height_px, std::move(corners)});
  }
  return views;
}

void write_manifest(const std::filesystem::path & path, const std::vector<std::string> & entries)
{
  std::ofstream output(path);
  ASSERT_TRUE(output.good());
  for (const auto & entry : entries) {
    output << entry << '\n';
  }
}

void write_linked_dataset_manifest(
  const std::filesystem::path & path, const std::string & image_path,
  const std::string & sha256)
{
  YAML::Node root(YAML::NodeType::Map);
  root["manifest_type"] = "calibration_dataset_evidence";
  root["status"] = "accepted";
  YAML::Node record(YAML::NodeType::Map);
  record["status"] = "accepted";
  record["image_path"] = image_path;
  record["sha256"] = sha256;
  root["records"].push_back(record);
  std::ofstream output(path, std::ios::binary);
  ASSERT_TRUE(output.good());
  output << root;
}

}  // namespace

TEST(CameraCalibrationInput, StrictEvidenceOnlyValidation)
{
  const auto input = load_fixture();
  EXPECT_EQ(input.profile, "evidence_only");
  EXPECT_EQ(input.board.inner_corners_cols, 9);
  EXPECT_EQ(input.board.inner_corners_rows, 6);
  EXPECT_EQ(input.image.pipeline, calibration::kRawImagePipeline);
  EXPECT_FALSE(input.validate().has_value());

  auto invalid_profile = input;
  invalid_profile.profile = "production";
  EXPECT_TRUE(invalid_profile.validate().has_value());

  auto invalid_square = input;
  invalid_square.board.square_size_m = 0.0;
  EXPECT_TRUE(invalid_square.validate().has_value());

  auto invalid_board = input;
  invalid_board.board.inner_corners_cols = 1;
  EXPECT_TRUE(invalid_board.validate().has_value());
}

TEST(CameraCalibrationInput, MalformedYamlAndEmptyManifestFailClosed)
{
  TemporaryDirectory temporary;
  const auto malformed = temporary.path() / "malformed.yaml";
  {
    std::ofstream output(malformed);
    output << "schema_version: [not-an-integer\n";
  }
  EXPECT_THROW(calibration::load_calibration_input(malformed.string()), std::runtime_error);

  const auto manifest = temporary.path() / "empty.txt";
  write_manifest(manifest, {});
  EXPECT_TRUE(calibration::load_image_manifest(manifest.string()).empty());

  const auto input = load_fixture();
  const auto result = calibration::run_image_calibration(input, {});
  EXPECT_FALSE(result.quality_accepted);
  EXPECT_FALSE(result.rejection_reasons.empty());
  const auto report = temporary.path() / "empty_report.yaml";
  EXPECT_NO_THROW(calibration::write_evidence_report(result, report.string()));
  EXPECT_TRUE(std::filesystem::is_regular_file(report));
  const auto yaml = YAML::LoadFile(report.string());
  EXPECT_EQ(yaml["profile"].as<std::string>(), "evidence_only");
  EXPECT_FALSE(yaml["production_ready"].as<bool>());
}

TEST(CameraCalibration, SyntheticProjectPointsRecoverReasonableIntrinsics)
{
  const auto input = load_fixture();
  const auto result = calibration::calibrate_detected_views(input, make_synthetic_views(input));
  ASSERT_TRUE(result.calibration_succeeded);
  ASSERT_TRUE(result.quality_accepted);
  ASSERT_TRUE(result.global_rms_reprojection_error_px.has_value());
  EXPECT_LT(*result.global_rms_reprojection_error_px, 1e-3);
  EXPECT_NEAR(result.camera_matrix(0, 0), 820.0, 8.0);
  EXPECT_NEAR(result.camera_matrix(1, 1), 805.0, 8.0);
  EXPECT_NEAR(result.camera_matrix(0, 2), 320.0, 3.0);
  EXPECT_NEAR(result.camera_matrix(1, 2), 240.0, 3.0);
  EXPECT_EQ(result.distortion_coefficients.size(), 5U);
  ASSERT_EQ(result.image_evidence.size(), 15U);
  for (const auto & evidence : result.image_evidence) {
    EXPECT_TRUE(evidence.accepted);
    ASSERT_TRUE(evidence.reprojection_error_px.has_value());
    EXPECT_LT(*evidence.reprojection_error_px, 1e-3);
  }
}

TEST(CameraCalibration, MixedDimensionsAndInvalidCornersAreRecorded)
{
  const auto input = load_fixture();
  auto views = make_synthetic_views(input, 2);
  views[1].image_width_px = input.image.width_px + 1;
  views.push_back(calibration::DetectedChessboardView{
    "bad_corners.png", input.image.width_px, input.image.height_px, {cv::Point2f{1.0F, 2.0F}}});
  const auto result = calibration::calibrate_detected_views(input, views);
  ASSERT_EQ(result.image_evidence.size(), 3U);
  EXPECT_TRUE(result.image_evidence[0].accepted);
  EXPECT_EQ(
    result.image_evidence[1].failure_reason,
    calibration::ImageFailureReason::ImageDimensionsMismatch);
  EXPECT_EQ(
    result.image_evidence[2].failure_reason,
    calibration::ImageFailureReason::InvalidDetectedCorners);
  EXPECT_FALSE(result.quality_accepted);
}

TEST(CameraCalibration, BlankUnreadableAndMixedResolutionImagesAreRejected)
{
  TemporaryDirectory temporary;
  const auto input = load_fixture();
  const auto blank_path = temporary.path() / "blank.png";
  const auto mixed_path = temporary.path() / "mixed.png";
  const auto missing_path = temporary.path() / "missing.png";
  ASSERT_TRUE(cv::imwrite(blank_path.string(), cv::Mat::zeros(480, 640, CV_8UC3)));
  ASSERT_TRUE(cv::imwrite(mixed_path.string(), cv::Mat::zeros(320, 240, CV_8UC3)));

  const auto result = calibration::run_image_calibration(
    input, {blank_path.string(), mixed_path.string(), missing_path.string()});
  ASSERT_EQ(result.image_evidence.size(), 3U);
  EXPECT_EQ(
    result.image_evidence[0].failure_reason,
    calibration::ImageFailureReason::ChessboardNotFound);
  EXPECT_EQ(
    result.image_evidence[1].failure_reason,
    calibration::ImageFailureReason::ImageDimensionsMismatch);
  EXPECT_EQ(
    result.image_evidence[2].failure_reason,
    calibration::ImageFailureReason::ImageUnreadable);
  EXPECT_FALSE(result.quality_accepted);
}

TEST(CameraCalibration, QualityFailuresWriteRejectedEvidenceAndReturnNonzero)
{
  TemporaryDirectory temporary;
  auto input = load_fixture();
  input.acceptance.min_accepted_views = 20;
  const auto insufficient = calibration::calibrate_detected_views(
    input, make_synthetic_views(input, 3));
  EXPECT_FALSE(insufficient.quality_accepted);
  EXPECT_NE(calibration::quality_exit_code(insufficient), 0);
  const auto insufficient_report = temporary.path() / "insufficient.yaml";
  ASSERT_NO_THROW(calibration::write_evidence_report(insufficient, insufficient_report.string()));
  const auto insufficient_yaml = YAML::LoadFile(insufficient_report.string());
  EXPECT_EQ(insufficient_yaml["evidence_status"].as<std::string>(), "rejected");
  EXPECT_FALSE(insufficient_yaml["calibration_result"]["candidate_available"].as<bool>());

  input.acceptance.min_accepted_views = 10;
  input.acceptance.max_global_rms_reprojection_error_px = 1e-12;
  const auto rms_rejected = calibration::calibrate_detected_views(
    input, make_synthetic_views(input, 12, 0.02));
  EXPECT_FALSE(rms_rejected.quality_accepted);
  EXPECT_NE(calibration::quality_exit_code(rms_rejected), 0);
  const auto rms_report = temporary.path() / "rms_rejected.yaml";
  ASSERT_NO_THROW(calibration::write_evidence_report(rms_rejected, rms_report.string()));
  EXPECT_EQ(
    YAML::LoadFile(rms_report.string())["evidence_status"].as<std::string>(), "rejected");

  const auto empty_manifest = temporary.path() / "cli_empty.txt";
  write_manifest(empty_manifest, {});
  const auto cli_report = temporary.path() / "cli_rejected.yaml";
  const std::string command =
    std::string("\"") + CAMERA_CALIBRATION_EXECUTABLE_PATH + "\" --config \"" +
    CAMERA_CALIBRATION_TEST_CONFIG_PATH + "\" --image-list \"" + empty_manifest.string() +
    "\" --report \"" + cli_report.string() + "\"";
  const int cli_status = std::system(command.c_str());
  EXPECT_NE(cli_status, 0);
  EXPECT_TRUE(std::filesystem::is_regular_file(cli_report));
  EXPECT_EQ(
    YAML::LoadFile(cli_report.string())["evidence_status"].as<std::string>(), "rejected");
}

TEST(CameraCalibration, AcceptedReportContainsEvidenceAndNoProductionPnPRoot)
{
  TemporaryDirectory temporary;
  const auto input = load_fixture();
  const auto result = calibration::calibrate_detected_views(input, make_synthetic_views(input));
  ASSERT_TRUE(result.quality_accepted);
  const auto report = temporary.path() / "accepted.yaml";
  ASSERT_NO_THROW(calibration::write_evidence_report(result, report.string()));

  const auto yaml = YAML::LoadFile(report.string());
  EXPECT_EQ(yaml["report_schema_version"].as<int>(), 1);
  EXPECT_EQ(yaml["report_type"].as<std::string>(), "camera_intrinsic_calibration_evidence");
  EXPECT_EQ(yaml["profile"].as<std::string>(), "evidence_only");
  EXPECT_FALSE(yaml["production_ready"].as<bool>());
  EXPECT_EQ(yaml["summary"]["accepted_view_count"].as<int>(), 15);
  EXPECT_TRUE(yaml["summary"]["global_rms_reprojection_error_px"]);
  EXPECT_EQ(yaml["images"].size(), 15U);
  EXPECT_EQ(yaml["input"]["metadata"]["camera_serial"].as<std::string>(), "synthetic-camera");

  const auto manual = yaml["pnp_camera_fields_for_manual_review"];
  EXPECT_EQ(manual["image_width"].as<int>(), 640);
  EXPECT_EQ(manual["image_height"].as<int>(), 480);
  EXPECT_EQ(manual["camera_matrix"].size(), 9U);
  EXPECT_EQ(manual["coordinate_frame"].as<std::string>(), "opencv_camera_optical");
  EXPECT_FALSE(yaml["camera"]);
  EXPECT_FALSE(yaml["armor_geometry"]);
  EXPECT_FALSE(yaml["pnp"]);
  EXPECT_FALSE(yaml["camera_to_gimbal"]);
}

TEST(CameraCalibration, DoesNotOverwriteCameraInfoYaml)
{
  TemporaryDirectory temporary;
  const auto result = calibration::calibrate_detected_views(load_fixture(), {});
  const auto protected_path = temporary.path() / "camera_info.yaml";
  EXPECT_THROW(
    calibration::write_evidence_report(result, protected_path.string()), std::invalid_argument);
}

TEST(CameraCalibration, DatasetManifestSha256IsOptionalButVerifiedWhenDeclared)
{
  TemporaryDirectory temporary;
  const auto manifest = temporary.path() / "dataset_manifest.yaml";
  {
    std::ofstream output(manifest, std::ios::binary);
  }
  auto input = load_fixture();
  input.metadata.dataset_manifest = "dataset_manifest.yaml";
  input.metadata.dataset_manifest_sha256 =
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
  ASSERT_FALSE(input.validate().has_value());
  EXPECT_NO_THROW(calibration::verify_dataset_manifest(input, manifest.string()));
  EXPECT_THROW(calibration::verify_dataset_manifest(input, ""), std::invalid_argument);

  {
    std::ofstream output(manifest, std::ios::app);
    output << "changed\n";
  }
  EXPECT_THROW(calibration::verify_dataset_manifest(input, manifest.string()), std::runtime_error);

  input.metadata.dataset_manifest_sha256 = "not-a-sha";
  ASSERT_TRUE(input.validate().has_value());
}

TEST(CameraCalibration, LinkedDatasetHashIsArchivedInEvidenceReport)
{
  TemporaryDirectory temporary;
  auto input = load_fixture();
  input.metadata.dataset_manifest = "dataset_manifest.yaml";
  input.metadata.dataset_manifest_sha256 =
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
  const auto result = calibration::calibrate_detected_views(input, make_synthetic_views(input));
  ASSERT_TRUE(result.quality_accepted);
  const auto report = temporary.path() / "linked.yaml";
  calibration::write_evidence_report(result, report.string());
  const auto yaml = YAML::LoadFile(report.string());
  EXPECT_EQ(
    yaml["dataset_manifest_sha256"].as<std::string>(),
    *input.metadata.dataset_manifest_sha256);
  EXPECT_EQ(
    yaml["input"]["metadata"]["dataset_manifest"].as<std::string>(),
    "dataset_manifest.yaml");
}

TEST(CameraCalibration, LinkedDatasetRejectsReplacedImageListAndPng)
{
  TemporaryDirectory temporary;
  const auto images = temporary.path() / "images";
  std::filesystem::create_directory(images);
  const auto archived = images / "frame_000001.png";
  ASSERT_TRUE(cv::imwrite(archived.string(), cv::Mat::zeros(8, 8, CV_8UC3)));
  const auto archived_sha = calibration::sha256_file(archived.string());

  const auto manifest = temporary.path() / "dataset_manifest.yaml";
  write_linked_dataset_manifest(manifest, "images/frame_000001.png", archived_sha);
  auto input = load_fixture();
  input.metadata.dataset_manifest = "dataset_manifest.yaml";
  input.metadata.dataset_manifest_sha256 = calibration::sha256_file(manifest.string());
  const auto config_path = temporary.path() / "calibration_input.yaml";
  auto config_yaml = YAML::LoadFile(CAMERA_CALIBRATION_TEST_CONFIG_PATH);
  config_yaml["metadata"]["dataset_manifest"] = *input.metadata.dataset_manifest;
  config_yaml["metadata"]["dataset_manifest_sha256"] =
    *input.metadata.dataset_manifest_sha256;
  std::ofstream(config_path) << config_yaml;

  const auto image_list = temporary.path() / "images.txt";
  write_manifest(image_list, {"images/frame_000001.png"});
  const auto verified = calibration::load_verified_calibration_images(
    input, config_path.string(), manifest.string(), image_list.string());
  ASSERT_EQ(verified.size(), 1U);
  EXPECT_EQ(std::filesystem::path(verified.front()), archived);

  write_manifest(image_list, {"images/alternate_board.png"});
  EXPECT_THROW(
    calibration::load_verified_calibration_images(
      input, config_path.string(), manifest.string(), image_list.string()),
    std::runtime_error);
  const auto report = temporary.path() / "must_not_exist.yaml";
  const auto cli_command = [&]() {
      return std::string("\"") + CAMERA_CALIBRATION_EXECUTABLE_PATH +
             "\" --config \"" + config_path.string() +
             "\" --dataset-manifest \"" + manifest.string() +
             "\" --image-list \"" + image_list.string() +
             "\" --report \"" + report.string() + "\"";
    };
  EXPECT_NE(std::system(cli_command().c_str()), 0);
  EXPECT_FALSE(std::filesystem::exists(report));

  write_manifest(image_list, {"images/frame_000001.png"});
  ASSERT_TRUE(cv::imwrite(archived.string(), cv::Mat::ones(8, 8, CV_8UC3)));
  EXPECT_THROW(
    calibration::load_verified_calibration_images(
      input, config_path.string(), manifest.string(), image_list.string()),
    std::runtime_error);
  EXPECT_NE(std::system(cli_command().c_str()), 0);
  EXPECT_FALSE(std::filesystem::exists(report));
}
