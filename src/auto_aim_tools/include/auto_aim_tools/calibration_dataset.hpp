#ifndef AUTO_AIM_TOOLS__CALIBRATION_DATASET_HPP_
#define AUTO_AIM_TOOLS__CALIBRATION_DATASET_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace auto_aim_tools::calibration_dataset
{

inline constexpr int kSchemaVersion = 1;
inline constexpr char kToolVersion[] = "calibration_dataset_v1";
inline constexpr char kExpectedEncoding[] = "rgb8";
inline constexpr char kChessboardType[] = "chessboard";

struct HeaderStamp
{
  std::int64_t sec{0};
  std::uint32_t nanosec{0};
  std::string source;
};

struct CameraInfoEvidence
{
  HeaderStamp stamp;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string frame_id;
};

struct FrameInput
{
  std::string source_image;
  HeaderStamp stamp;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::uint32_t step{0};
  std::size_t declared_data_size{0};
  std::string frame_id;
  std::vector<std::uint8_t> rgb8;
  std::optional<CameraInfoEvidence> camera_info;
  std::vector<std::string> input_errors;
};

struct BoardSpecification
{
  std::string type;
  int inner_corners_cols{0};
  int inner_corners_rows{0};
  double square_size_m{0.0};
};

struct DatasetMetadata
{
  std::string dataset_id;
  std::string report_id;
  std::string camera_serial;
  std::string lens_identifier;
  std::string acquisition_date;
  std::string operator_identifier;
};

struct DatasetConfig
{
  int schema_version{0};
  std::string source_mode;
  std::string timestamp_source;
  bool camera_info_required{false};
  int min_views{0};
  double max_global_rms_reprojection_error_px{0.0};
  BoardSpecification board;
  DatasetMetadata metadata;
};

struct DatasetRequest
{
  DatasetConfig config;
  std::vector<FrameInput> frames;
  std::vector<std::string> input_errors;
};

struct DatasetResult
{
  bool accepted{false};
  std::filesystem::path manifest_path;
  std::filesystem::path hash_path;
  std::optional<std::filesystem::path> image_list_path;
  std::optional<std::filesystem::path> calibration_input_path;
  std::string manifest_sha256;
  std::vector<std::string> rejection_reasons;
};

DatasetRequest load_offline_fixture(const std::filesystem::path & fixture_path);
DatasetConfig load_ros_config(const std::filesystem::path & config_path);
DatasetResult build_dataset(
  const DatasetRequest & request, const std::filesystem::path & output_directory,
  const std::string & git_commit);
DatasetResult write_input_failure_manifest(
  const std::filesystem::path & output_directory, const std::string & source_mode,
  const std::string & git_commit, const std::vector<std::string> & reasons);

std::string sha256_file(const std::filesystem::path & path);
bool canonical_nonzero_stamp(const HeaderStamp & stamp) noexcept;
int dataset_exit_code(const DatasetResult & result) noexcept;

}  // namespace auto_aim_tools::calibration_dataset

#endif  // AUTO_AIM_TOOLS__CALIBRATION_DATASET_HPP_
