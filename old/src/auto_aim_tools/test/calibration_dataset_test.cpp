#include "auto_aim_tools/calibration_dataset.hpp"

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

namespace
{

namespace dataset = auto_aim_tools::calibration_dataset;

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    static std::size_t sequence = 0;
    path_ = std::filesystem::temp_directory_path() /
      ("calibration_dataset_test_" + std::to_string(++sequence));
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

dataset::DatasetConfig config(bool camera_info_required = false)
{
  dataset::DatasetConfig value;
  value.schema_version = 1;
  value.source_mode = camera_info_required ? "ros" : "offline_fixture";
  value.timestamp_source = camera_info_required ?
    "ros_header" : "fixture_declared_ros_header";
  value.camera_info_required = camera_info_required;
  value.min_views = 2;
  value.max_global_rms_reprojection_error_px = 0.5;
  value.board = {"chessboard", 9, 6, 0.024};
  value.metadata = {
    "synthetic-dataset", "synthetic-report", "unknown", "unknown",
    "2026-08-27", "automated-test"};
  return value;
}

dataset::FrameInput frame(std::uint32_t sequence, bool with_camera_info = false)
{
  dataset::FrameInput value;
  value.source_image = "synthetic:" + std::to_string(sequence);
  value.stamp = {
    100 + static_cast<std::int64_t>(sequence), sequence,
    "fixture:/image_raw.header.stamp"};
  value.width = 4;
  value.height = 3;
  value.encoding = "rgb8";
  value.step = 12;
  value.declared_data_size = 36;
  value.frame_id = "camera_optical_frame";
  value.rgb8.resize(36);
  for (std::size_t index = 0; index < value.rgb8.size(); ++index) {
    value.rgb8[index] = static_cast<std::uint8_t>((index + sequence * 31U) % 251U);
  }
  if (with_camera_info) {
    value.camera_info = dataset::CameraInfoEvidence{
      {value.stamp.sec, value.stamp.nanosec, "fixture:/camera_info.header.stamp"},
      value.width, value.height, value.frame_id};
  }
  return value;
}

bool has_reason(const dataset::DatasetResult & result, const std::string & text)
{
  for (const auto & reason : result.rejection_reasons) {
    if (reason.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(CalibrationDataset, NormalOfflineFixtureWritesVerifiableHandoff)
{
  TemporaryDirectory temporary;
  dataset::DatasetRequest request{config(), {frame(1), frame(2)}, {}};
  const auto result = dataset::build_dataset(request, temporary.path() / "accepted", "abcdef0");
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(dataset::dataset_exit_code(result), 0);
  EXPECT_TRUE(std::filesystem::is_regular_file(result.manifest_path));
  EXPECT_TRUE(std::filesystem::is_regular_file(result.hash_path));
  ASSERT_TRUE(result.image_list_path.has_value());
  ASSERT_TRUE(result.calibration_input_path.has_value());
  EXPECT_EQ(result.manifest_sha256, dataset::sha256_file(result.manifest_path));

  const auto manifest = YAML::LoadFile(result.manifest_path.string());
  EXPECT_EQ(manifest["status"].as<std::string>(), "accepted");
  EXPECT_EQ(manifest["profile"].as<std::string>(), "evidence_only");
  EXPECT_FALSE(manifest["production_ready"].as<bool>());
  EXPECT_EQ(manifest["source"]["camera_sdk_status"].as<std::string>(), "not_used");
  EXPECT_FALSE(manifest["records"][0]["camera_info"]["present"].as<bool>());
  EXPECT_TRUE(manifest["records"][0]["camera_info"]["timestamp_matches"].IsDefined());
  EXPECT_EQ(manifest["records"][0]["sha256"].as<std::string>().size(), 64U);

  const auto handoff = YAML::LoadFile(result.calibration_input_path->string());
  EXPECT_EQ(
    handoff["metadata"]["dataset_manifest_sha256"].as<std::string>(),
    result.manifest_sha256);
  EXPECT_EQ(handoff["metadata"]["dataset_manifest"].as<std::string>(), "dataset_manifest.yaml");
}

TEST(CalibrationDataset, AnyBadRecordRejectsWholeDatasetWithoutHandoff)
{
  TemporaryDirectory temporary;
  auto request = dataset::DatasetRequest{config(), {frame(1), frame(2), frame(3)}, {}};
  request.frames[1].width = 5;
  request.frames[1].step = 15;
  request.frames[1].declared_data_size = 45;
  request.frames[1].rgb8.resize(45);
  request.frames[2].encoding = "bgr8";
  request.frames[2].width = 0;
  request.frames[2].height = 0;
  request.frames[2].step = 0;
  request.frames[2].declared_data_size = 0;
  request.frames[2].rgb8.clear();
  const auto result = dataset::build_dataset(request, temporary.path() / "bad", "abcdef0");
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(has_reason(result, "mixed_resolution"));
  EXPECT_TRUE(has_reason(result, "encoding_must_be_rgb8"));
  EXPECT_TRUE(has_reason(result, "zero_image_dimensions"));
  EXPECT_TRUE(has_reason(result, "declared_data_size_must_equal_step_times_height"));
  EXPECT_FALSE(result.image_list_path.has_value());
  EXPECT_FALSE(result.calibration_input_path.has_value());
  EXPECT_TRUE(std::filesystem::is_regular_file(result.manifest_path));
  EXPECT_EQ(
    YAML::LoadFile(result.manifest_path.string())["status"].as<std::string>(),
    "rejected");
}

TEST(CalibrationDataset, DuplicateHashMissingCameraInfoAndTimestampRollbackFailClosed)
{
  TemporaryDirectory temporary;
  auto request = dataset::DatasetRequest{
    config(true), {frame(1, true), frame(2, false), frame(3, true)}, {}};
  request.frames[1].rgb8 = request.frames[0].rgb8;
  request.frames[1].stamp.sec = request.frames[0].stamp.sec - 1;
  request.frames[2].camera_info->stamp.nanosec += 1;
  const auto result = dataset::build_dataset(
    request, temporary.path() / "rejected", "abcdef0");
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(has_reason(result, "duplicate_frame_hash"));
  EXPECT_TRUE(has_reason(result, "camera_info_missing"));
  EXPECT_TRUE(has_reason(result, "image_timestamp_rollback"));
  EXPECT_TRUE(has_reason(result, "camera_info_timestamp_mismatch"));
}

TEST(CalibrationDataset, EmptyInputAndInvalidBoardProduceRejectedEvidence)
{
  TemporaryDirectory temporary;
  auto invalid = config();
  invalid.board.type = "charuco";
  const auto result = dataset::build_dataset(
    dataset::DatasetRequest{invalid, {}, {}}, temporary.path() / "empty", "abcdef0");
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(has_reason(result, "input_unavailable"));
  EXPECT_TRUE(has_reason(result, "unsupported_board_type"));
  const auto manifest = YAML::LoadFile(result.manifest_path.string());
  EXPECT_EQ(manifest["source"]["input_status"].as<std::string>(), "input_unavailable");
  EXPECT_FALSE(manifest["production_ready"].as<bool>());
}

TEST(CalibrationDataset, UnreadableFixtureRecordIsPreservedAndRejected)
{
  TemporaryDirectory temporary;
  const auto fixture = temporary.path() / "fixture.yaml";
  std::ofstream output(fixture);
  output <<
    "schema_version: 1\n"
    "source: {mode: offline_fixture, timestamp_source: fixture_declared_ros_header}\n"
    "camera_info_required: false\n"
    "board: {type: chessboard, inner_corners_cols: 9, inner_corners_rows: 6, "
    "square_size_m: 0.024}\n"
    "acceptance: {min_views: 1, max_global_rms_reprojection_error_px: 0.5}\n"
    "metadata: {dataset_id: test, report_id: test, camera_serial: unknown, "
    "lens_identifier: unknown, acquisition_date: '2026-08-27', operator: test}\n"
    "records:\n"
    "  - image: missing.png\n"
    "    stamp: {sec: 1, nanosec: 2, source: fixture:/image_raw.header.stamp}\n"
    "    width: 4\n"
    "    height: 3\n"
    "    encoding: rgb8\n"
    "    step: 12\n"
    "    data_size: 36\n"
    "    frame_id: camera_optical_frame\n";
  output.close();
  const auto request = dataset::load_offline_fixture(fixture);
  ASSERT_EQ(request.frames.size(), 1U);
  ASSERT_FALSE(request.frames[0].input_errors.empty());
  const auto result = dataset::build_dataset(
    request, temporary.path() / "unreadable", "abcdef0");
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(has_reason(result, "image_unreadable"));
  EXPECT_EQ(
    YAML::LoadFile(result.manifest_path.string())["records"][0]["status"].as<std::string>(),
    "rejected");
}

TEST(CalibrationDataset, ManifestHashDetectsMutationAndOutputCannotRestart)
{
  TemporaryDirectory temporary;
  dataset::DatasetRequest request{config(), {frame(1), frame(2)}, {}};
  const auto output = temporary.path() / "accepted";
  const auto result = dataset::build_dataset(request, output, "abcdef0");
  std::ofstream mutate(result.manifest_path, std::ios::app);
  mutate << "# mutated\n";
  mutate.close();
  EXPECT_NE(result.manifest_sha256, dataset::sha256_file(result.manifest_path));
  EXPECT_THROW(dataset::build_dataset(request, output, "abcdef0"), std::invalid_argument);
}

TEST(CalibrationDataset, MalformedInputStillHasRejectedManifestEntryPoint)
{
  TemporaryDirectory temporary;
  const auto result = dataset::write_input_failure_manifest(
    temporary.path() / "failure", "offline_fixture", "abcdef0", {"malformed_fixture"});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(dataset::dataset_exit_code(result), 2);
  const auto manifest = YAML::LoadFile(result.manifest_path.string());
  EXPECT_EQ(manifest["status"].as<std::string>(), "rejected");
  EXPECT_EQ(manifest["rejection_reasons"][0].as<std::string>(), "malformed_fixture");
}
