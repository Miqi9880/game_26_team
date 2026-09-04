#include "auto_aim_tools/calibration_dataset.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <openssl/evp.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace auto_aim_tools::calibration_dataset
{
namespace
{

struct RecordEvidence
{
  const FrameInput * input{nullptr};
  std::string archived_path;
  std::string archived_sha256;
  std::vector<std::string> errors;
  std::vector<std::string> warnings;
  bool camera_info_stamp_matches{false};
  bool camera_info_dimensions_match{false};
  bool camera_info_frame_id_matches{false};
};

YAML::Node required_node(
  const YAML::Node & parent, const std::string & key, const std::string & location)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(location + ": missing required field '" + key + "'");
  }
  return node;
}

YAML::Node required_map(
  const YAML::Node & parent, const std::string & key, const std::string & location)
{
  const auto node = required_node(parent, key, location);
  if (!node.IsMap()) {
    throw std::runtime_error(location + "." + key + " must be a map");
  }
  return node;
}

std::string required_string(
  const YAML::Node & parent, const std::string & key, const std::string & location)
{
  try {
    const auto value = required_node(parent, key, location).as<std::string>();
    if (value.empty()) {
      throw std::runtime_error(location + "." + key + " must not be empty");
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(location + "." + key + " must be a string");
  }
}

template<typename Value>
Value required_value(
  const YAML::Node & parent, const std::string & key, const std::string & location)
{
  try {
    return required_node(parent, key, location).as<Value>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(location + "." + key + " has an invalid type or value");
  }
}

HeaderStamp parse_stamp(const YAML::Node & node, const std::string & location)
{
  if (!node || !node.IsMap()) {
    throw std::runtime_error(location + " must be a map");
  }
  HeaderStamp stamp;
  stamp.sec = required_value<std::int64_t>(node, "sec", location);
  stamp.nanosec = required_value<std::uint32_t>(node, "nanosec", location);
  stamp.source = required_string(node, "source", location);
  return stamp;
}

DatasetConfig parse_config(const YAML::Node & root)
{
  if (!root.IsMap()) {
    throw std::runtime_error("fixture root must be a map");
  }
  DatasetConfig config;
  config.schema_version = required_value<int>(root, "schema_version", "root");
  const auto source = required_map(root, "source", "root");
  config.source_mode = required_string(source, "mode", "source");
  config.timestamp_source = required_string(source, "timestamp_source", "source");
  config.camera_info_required = required_value<bool>(root, "camera_info_required", "root");

  const auto board = required_map(root, "board", "root");
  config.board.type = required_string(board, "type", "board");
  config.board.inner_corners_cols = required_value<int>(board, "inner_corners_cols", "board");
  config.board.inner_corners_rows = required_value<int>(board, "inner_corners_rows", "board");
  config.board.square_size_m = required_value<double>(board, "square_size_m", "board");

  const auto acceptance = required_map(root, "acceptance", "root");
  config.min_views = required_value<int>(acceptance, "min_views", "acceptance");
  config.max_global_rms_reprojection_error_px =
    required_value<double>(acceptance, "max_global_rms_reprojection_error_px", "acceptance");

  const auto metadata = required_map(root, "metadata", "root");
  config.metadata.dataset_id = required_string(metadata, "dataset_id", "metadata");
  config.metadata.report_id = required_string(metadata, "report_id", "metadata");
  config.metadata.camera_serial = required_string(metadata, "camera_serial", "metadata");
  config.metadata.lens_identifier = required_string(metadata, "lens_identifier", "metadata");
  config.metadata.acquisition_date = required_string(metadata, "acquisition_date", "metadata");
  config.metadata.operator_identifier = required_string(metadata, "operator", "metadata");
  return config;
}

YAML::Node load_yaml(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("input YAML is not a regular file: " + path.string());
  }
  try {
    return YAML::LoadFile(path.string());
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse input YAML " + path.string() + ": " + error.what());
  }
}

std::string utc_now()
{
  const auto now = std::chrono::system_clock::now();
  const auto time = std::chrono::system_clock::to_time_t(now);
  std::tm utc{};
  gmtime_r(&time, &utc);
  std::ostringstream output;
  output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%SZ");
  return output.str();
}

std::optional<std::int64_t> stamp_ns(const HeaderStamp & stamp) noexcept
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U ||
    stamp.sec > std::numeric_limits<std::int64_t>::max() / 1000000000LL)
  {
    return std::nullopt;
  }
  return stamp.sec * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
}

bool valid_date(const std::string & value)
{
  static const std::regex pattern(R"(^([0-9]{4})-([0-9]{2})-([0-9]{2})$)");
  std::smatch match;
  if (!std::regex_match(value, match, pattern)) {
    return false;
  }
  const int year = std::stoi(match[1].str());
  const int month = std::stoi(match[2].str());
  const int day = std::stoi(match[3].str());
  if (year < 1970 || month < 1 || month > 12) {
    return false;
  }
  static constexpr int days_in_month[] =
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int maximum = days_in_month[month - 1];
  const bool leap = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  if (month == 2 && leap) {
    maximum = 29;
  }
  return day >= 1 && day <= maximum;
}

void append_config_errors(const DatasetConfig & config, std::vector<std::string> * errors)
{
  if (config.schema_version != kSchemaVersion) {
    errors->push_back("unsupported_schema_version");
  }
  if (config.source_mode != "offline_fixture" && config.source_mode != "ros") {
    errors->push_back("source_mode_must_be_offline_fixture_or_ros");
  }
  if (config.source_mode == "ros" && !config.camera_info_required) {
    errors->push_back("ros_mode_requires_camera_info");
  }
  if (config.source_mode == "ros" && config.timestamp_source != "ros_header") {
    errors->push_back("ros_timestamp_source_must_be_ros_header");
  }
  if (config.timestamp_source.empty()) {
    errors->push_back("timestamp_source_missing");
  }
  if (config.board.type != kChessboardType) {
    errors->push_back("unsupported_board_type:" + config.board.type);
  }
  if (config.board.inner_corners_cols < 2 || config.board.inner_corners_rows < 2) {
    errors->push_back("invalid_chessboard_inner_corner_count");
  }
  if (!std::isfinite(config.board.square_size_m) || config.board.square_size_m <= 0.0) {
    errors->push_back("invalid_chessboard_square_size_m");
  }
  if (config.min_views <= 0) {
    errors->push_back("min_views_must_be_positive");
  }
  if (!std::isfinite(config.max_global_rms_reprojection_error_px) ||
    config.max_global_rms_reprojection_error_px <= 0.0)
  {
    errors->push_back("max_global_rms_reprojection_error_px_must_be_positive");
  }
  if (config.metadata.dataset_id.empty() || config.metadata.report_id.empty() ||
    config.metadata.camera_serial.empty() || config.metadata.lens_identifier.empty() ||
    config.metadata.operator_identifier.empty())
  {
    errors->push_back("dataset_traceability_metadata_missing");
  }
  if (!valid_date(config.metadata.acquisition_date)) {
    errors->push_back("acquisition_date_must_be_yyyy_mm_dd");
  }
}

YAML::Node string_sequence(const std::vector<std::string> & values)
{
  YAML::Node result(YAML::NodeType::Sequence);
  for (const auto & value : values) {
    result.push_back(value);
  }
  return result;
}

void write_yaml(const YAML::Node & root, const std::filesystem::path & path)
{
  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    throw std::runtime_error("failed to encode YAML: " + path.string());
  }
  std::ofstream output(path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open output file: " + path.string());
  }
  output << emitter.c_str() << '\n';
  if (!output) {
    throw std::runtime_error("failed to write output file: " + path.string());
  }
}

void create_new_output_directory(const std::filesystem::path & path)
{
  if (path.empty() || path.filename().empty()) {
    throw std::invalid_argument("output directory must name a specific directory");
  }
  if (std::filesystem::exists(path)) {
    throw std::invalid_argument("output directory already exists; refusing to overwrite: " + path.string());
  }
  if (!path.parent_path().empty()) {
    std::filesystem::create_directories(path.parent_path());
  }
  if (!std::filesystem::create_directory(path)) {
    throw std::runtime_error("failed to create output directory: " + path.string());
  }
}

YAML::Node base_manifest(
  const std::string & source_mode, const std::string & git_commit, const std::string & input_status)
{
  YAML::Node root(YAML::NodeType::Map);
  root["schema_version"] = kSchemaVersion;
  root["manifest_type"] = "calibration_dataset_evidence";
  root["profile"] = "evidence_only";
  root["production_ready"] = false;
  root["created_at_utc"] = utc_now();
  root["tool"]["version"] = kToolVersion;
  root["tool"]["git_commit"] = git_commit.empty() ? "unknown" : git_commit;
  root["source"]["mode"] = source_mode.empty() ? "unknown" : source_mode;
  root["source"]["input_status"] = input_status;
  root["source"]["camera_sdk_status"] = "not_used";
  root["traceability"]["imu_time"] = "unknown";
  root["traceability"]["mcu_time"] = "unknown";
  root["traceability"]["hardware_synchronization"] = "unknown";
  root["safety"]["serial_enabled"] = false;
  root["safety"]["dry_run"] = true;
  root["safety"]["allow_fire"] = false;
  root["safety"]["fire_command"] = 0;
  root["safety"]["yaw_velocity"] = 0;
  root["safety"]["yaw_acceleration"] = 0;
  root["safety"]["pitch_velocity"] = 0;
  root["safety"]["pitch_acceleration"] = 0;
  return root;
}

void write_hash_sidecar(DatasetResult * result)
{
  result->manifest_sha256 = sha256_file(result->manifest_path);
  result->hash_path = result->manifest_path.parent_path() / "dataset_manifest.sha256";
  std::ofstream output(result->hash_path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot write manifest hash sidecar");
  }
  output << result->manifest_sha256 << "  " << result->manifest_path.filename().string() << '\n';
  if (!output) {
    throw std::runtime_error("failed to write manifest hash sidecar");
  }
}

void write_calibration_handoff(
  const DatasetRequest & request, const std::filesystem::path & output_directory,
  const std::string & git_commit, DatasetResult * result)
{
  result->image_list_path = output_directory / "images.txt";
  std::ofstream images(*result->image_list_path, std::ios::out | std::ios::trunc);
  if (!images) {
    throw std::runtime_error("cannot write images.txt");
  }
  for (std::size_t index = 0; index < request.frames.size(); ++index) {
    images << "images/frame_" << std::setfill('0') << std::setw(6) << index + 1 << ".png\n";
  }
  if (!images) {
    throw std::runtime_error("failed to write images.txt");
  }

  const auto & first = request.frames.front();
  YAML::Node config(YAML::NodeType::Map);
  config["schema_version"] = 1;
  config["profile"] = "evidence_only";
  config["board"]["type"] = request.config.board.type;
  config["board"]["inner_corners_cols"] = request.config.board.inner_corners_cols;
  config["board"]["inner_corners_rows"] = request.config.board.inner_corners_rows;
  config["board"]["square_size_m"] = request.config.board.square_size_m;
  config["image"]["width_px"] = first.width;
  config["image"]["height_px"] = first.height;
  config["image"]["pipeline"] = "raw_image_no_resize_no_crop_no_rectify";
  config["acceptance"]["min_accepted_views"] = request.config.min_views;
  config["acceptance"]["max_global_rms_reprojection_error_px"] =
    request.config.max_global_rms_reprojection_error_px;
  config["metadata"]["report_id"] = request.config.metadata.report_id;
  config["metadata"]["camera_serial"] = request.config.metadata.camera_serial;
  config["metadata"]["lens_identifier"] = request.config.metadata.lens_identifier;
  config["metadata"]["acquisition_date"] = request.config.metadata.acquisition_date;
  config["metadata"]["operator"] = request.config.metadata.operator_identifier;
  config["metadata"]["code_commit"] =
    git_commit.size() >= 7U ? git_commit : "0000000";
  config["metadata"]["dataset_id"] = request.config.metadata.dataset_id;
  config["metadata"]["dataset_manifest"] = "dataset_manifest.yaml";
  config["metadata"]["dataset_manifest_sha256"] = result->manifest_sha256;
  result->calibration_input_path = output_directory / "calibration_input.yaml";
  write_yaml(config, *result->calibration_input_path);
}

}  // namespace

bool canonical_nonzero_stamp(const HeaderStamp & stamp) noexcept
{
  const auto value = stamp_ns(stamp);
  return value.has_value() && *value != 0 && !stamp.source.empty();
}

std::string sha256_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot hash file: " + path.string());
  }
  EVP_MD_CTX * raw_context = EVP_MD_CTX_new();
  if (raw_context == nullptr) {
    throw std::runtime_error("cannot allocate SHA-256 context");
  }
  const std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
    raw_context, &EVP_MD_CTX_free);
  if (EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 16384> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 &&
      EVP_DigestUpdate(context.get(), buffer.data(), static_cast<std::size_t>(count)) != 1)
    {
      throw std::runtime_error("SHA-256 update failed");
    }
  }
  if (input.bad()) {
    throw std::runtime_error("failed while reading file for SHA-256: " + path.string());
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 ||
    digest_size != 32U)
  {
    throw std::runtime_error("SHA-256 finalization failed");
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

DatasetRequest load_offline_fixture(const std::filesystem::path & fixture_path)
{
  const auto root = load_yaml(fixture_path);
  DatasetRequest request;
  request.config = parse_config(root);
  if (request.config.source_mode != "offline_fixture") {
    request.input_errors.push_back("offline_cli_requires_source_mode_offline_fixture");
  }
  const auto records = required_node(root, "records", "root");
  if (!records.IsSequence()) {
    throw std::runtime_error("root.records must be a sequence");
  }
  const auto base = fixture_path.parent_path();
  for (std::size_t index = 0; index < records.size(); ++index) {
    FrameInput frame;
    const auto record = records[index];
    const std::string location = "records[" + std::to_string(index) + "]";
    try {
      if (!record.IsMap()) {
        throw std::runtime_error(location + " must be a map");
      }
      frame.source_image = required_string(record, "image", location);
      frame.stamp = parse_stamp(required_node(record, "stamp", location), location + ".stamp");
      frame.width = required_value<std::uint32_t>(record, "width", location);
      frame.height = required_value<std::uint32_t>(record, "height", location);
      frame.encoding = required_string(record, "encoding", location);
      frame.step = required_value<std::uint32_t>(record, "step", location);
      frame.declared_data_size = required_value<std::size_t>(record, "data_size", location);
      frame.frame_id = required_string(record, "frame_id", location);

      if (const auto camera = record["camera_info"]; camera && !camera.IsNull()) {
        if (!camera.IsMap()) {
          throw std::runtime_error(location + ".camera_info must be a map");
        }
        CameraInfoEvidence info;
        info.stamp = parse_stamp(
          required_node(camera, "stamp", location + ".camera_info"),
          location + ".camera_info.stamp");
        info.width = required_value<std::uint32_t>(camera, "width", location + ".camera_info");
        info.height = required_value<std::uint32_t>(camera, "height", location + ".camera_info");
        info.frame_id = required_string(camera, "frame_id", location + ".camera_info");
        frame.camera_info = std::move(info);
      }

      const auto source_path =
        (std::filesystem::path(frame.source_image).is_absolute() ?
        std::filesystem::path(frame.source_image) : base / frame.source_image).lexically_normal();
      const cv::Mat image = cv::imread(source_path.string(), cv::IMREAD_UNCHANGED);
      if (image.empty()) {
        frame.input_errors.push_back("image_unreadable");
      } else if (image.type() != CV_8UC3) {
        frame.input_errors.push_back("image_must_decode_as_8uc3");
      } else {
        if (image.cols != static_cast<int>(frame.width) ||
          image.rows != static_cast<int>(frame.height))
        {
          frame.input_errors.push_back("decoded_image_dimensions_mismatch");
        }
        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        frame.rgb8.assign(rgb.datastart, rgb.dataend);
      }
    } catch (const std::exception & error) {
      frame.input_errors.push_back(error.what());
    }
    request.frames.push_back(std::move(frame));
  }
  return request;
}

DatasetConfig load_ros_config(const std::filesystem::path & config_path)
{
  auto config = parse_config(load_yaml(config_path));
  if (config.source_mode != "ros") {
    throw std::runtime_error("ROS recorder requires source.mode: ros");
  }
  if (!config.camera_info_required) {
    throw std::runtime_error("ROS recorder requires camera_info_required: true");
  }
  if (config.timestamp_source != "ros_header") {
    throw std::runtime_error("ROS recorder requires source.timestamp_source: ros_header");
  }
  return config;
}

DatasetResult build_dataset(
  const DatasetRequest & request, const std::filesystem::path & output_directory,
  const std::string & git_commit)
{
  create_new_output_directory(output_directory);
  std::filesystem::create_directory(output_directory / "images");

  DatasetResult result;
  result.manifest_path = output_directory / "dataset_manifest.yaml";
  result.rejection_reasons = request.input_errors;
  append_config_errors(request.config, &result.rejection_reasons);
  if (request.frames.empty()) {
    result.rejection_reasons.push_back("input_unavailable:no_frames");
  }

  std::vector<RecordEvidence> evidence;
  evidence.reserve(request.frames.size());
  std::optional<std::pair<std::uint32_t, std::uint32_t>> expected_dimensions;
  std::optional<std::int64_t> previous_stamp;
  std::map<std::string, std::size_t> hashes;
  std::size_t archived_image_count = 0U;

  for (std::size_t index = 0; index < request.frames.size(); ++index) {
    const auto & frame = request.frames[index];
    RecordEvidence record;
    record.input = &frame;
    record.errors = frame.input_errors;
    if (frame.width == 0U || frame.height == 0U) {
      record.errors.push_back("zero_image_dimensions");
    }
    if (frame.encoding != kExpectedEncoding) {
      record.errors.push_back("encoding_must_be_rgb8");
    }
    if (frame.frame_id.empty()) {
      record.errors.push_back("image_frame_id_missing");
    }
    const std::uint64_t expected_step = static_cast<std::uint64_t>(frame.width) * 3U;
    if (expected_step > std::numeric_limits<std::uint32_t>::max() ||
      frame.step != expected_step)
    {
      record.errors.push_back("step_must_equal_width_times_3");
    }
    const std::uint64_t expected_size = static_cast<std::uint64_t>(frame.step) * frame.height;
    if (expected_size == 0U || frame.declared_data_size != expected_size) {
      record.errors.push_back("declared_data_size_must_equal_step_times_height");
    }
    if (frame.rgb8.size() != expected_size) {
      record.errors.push_back("actual_rgb8_data_size_mismatch");
    }
    const auto current_stamp = stamp_ns(frame.stamp);
    if (!canonical_nonzero_stamp(frame.stamp)) {
      record.errors.push_back("image_timestamp_must_be_canonical_nonzero_with_source");
    } else if (previous_stamp.has_value() && *current_stamp < *previous_stamp) {
      record.errors.push_back("image_timestamp_rollback");
    } else if (previous_stamp.has_value() && *current_stamp == *previous_stamp) {
      record.warnings.push_back("duplicate_image_timestamp");
    }
    if (current_stamp.has_value()) {
      previous_stamp = current_stamp;
    }

    if (!expected_dimensions.has_value() && frame.width > 0U && frame.height > 0U) {
      expected_dimensions = std::make_pair(frame.width, frame.height);
    } else if (expected_dimensions.has_value() &&
      (frame.width != expected_dimensions->first || frame.height != expected_dimensions->second))
    {
      record.errors.push_back("mixed_resolution");
    }

    if (!frame.camera_info.has_value()) {
      if (request.config.camera_info_required) {
        record.errors.push_back("camera_info_missing");
      } else {
        record.warnings.push_back("camera_info_missing_not_required");
      }
    } else {
      const auto & camera = *frame.camera_info;
      record.camera_info_stamp_matches =
        canonical_nonzero_stamp(camera.stamp) && stamp_ns(camera.stamp) == current_stamp;
      record.camera_info_dimensions_match =
        camera.width == frame.width && camera.height == frame.height &&
        camera.width > 0U && camera.height > 0U;
      record.camera_info_frame_id_matches =
        !camera.frame_id.empty() && camera.frame_id == frame.frame_id;
      const auto add_camera_finding = [&](bool valid, const char * reason) {
          if (!valid) {
            (request.config.camera_info_required ? record.errors : record.warnings).push_back(reason);
          }
        };
      add_camera_finding(record.camera_info_stamp_matches, "camera_info_timestamp_mismatch");
      add_camera_finding(record.camera_info_dimensions_match, "camera_info_dimensions_mismatch");
      add_camera_finding(record.camera_info_frame_id_matches, "camera_info_frame_id_mismatch");
    }

    if (frame.rgb8.size() == expected_size && frame.width > 0U && frame.height > 0U &&
      frame.step == expected_step)
    {
      cv::Mat rgb(
        static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3,
        const_cast<std::uint8_t *>(frame.rgb8.data()), frame.step);
      cv::Mat bgr;
      cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
      std::ostringstream filename;
      filename << "frame_" << std::setfill('0') << std::setw(6) << index + 1 << ".png";
      const auto path = output_directory / "images" / filename.str();
      if (!cv::imwrite(path.string(), bgr)) {
        record.errors.push_back("png_archive_write_failed");
      } else {
        record.archived_path = "images/" + filename.str();
        record.archived_sha256 = sha256_file(path);
        ++archived_image_count;
        const auto duplicate = hashes.find(record.archived_sha256);
        if (duplicate != hashes.end()) {
          record.errors.push_back(
            "duplicate_frame_hash:first_record=" + std::to_string(duplicate->second));
        } else {
          hashes.emplace(record.archived_sha256, index);
        }
      }
    }
    for (const auto & error : record.errors) {
      result.rejection_reasons.push_back(
        "record[" + std::to_string(index) + "]:" + error);
    }
    evidence.push_back(std::move(record));
  }

  if (static_cast<int>(request.frames.size()) < request.config.min_views) {
    result.rejection_reasons.push_back(
      "insufficient_views:got=" + std::to_string(request.frames.size()) +
      ",required=" + std::to_string(request.config.min_views));
  }
  result.accepted = result.rejection_reasons.empty();

  auto root = base_manifest(
    request.config.source_mode, git_commit,
    request.frames.empty() ? "input_unavailable" : "available");
  root["status"] = result.accepted ? "accepted" : "rejected";
  root["quality_gate_passed"] = result.accepted;
  root["source"]["timestamp_source"] = request.config.timestamp_source;
  root["camera_info_required"] = request.config.camera_info_required;
  root["board"]["type"] = request.config.board.type;
  root["board"]["inner_corners_cols"] = request.config.board.inner_corners_cols;
  root["board"]["inner_corners_rows"] = request.config.board.inner_corners_rows;
  root["board"]["square_size_m"] = request.config.board.square_size_m;
  root["acceptance"]["min_views"] = request.config.min_views;
  root["acceptance"]["max_global_rms_reprojection_error_px"] =
    request.config.max_global_rms_reprojection_error_px;
  root["metadata"]["dataset_id"] = request.config.metadata.dataset_id;
  root["metadata"]["report_id"] = request.config.metadata.report_id;
  root["metadata"]["camera_serial"] = request.config.metadata.camera_serial;
  root["metadata"]["lens_identifier"] = request.config.metadata.lens_identifier;
  root["metadata"]["acquisition_date"] = request.config.metadata.acquisition_date;
  root["metadata"]["operator"] = request.config.metadata.operator_identifier;
  root["summary"]["record_count"] = request.frames.size();
  root["summary"]["archived_image_count"] = archived_image_count;
  root["summary"]["received_image_count"] = request.received_image_count;
  root["summary"]["received_camera_info_count"] = request.received_camera_info_count;
  root["summary"]["surplus_camera_info_count"] = request.surplus_camera_info_count;
  root["summary"]["peak_unpaired_image_count"] = request.peak_unpaired_image_count;
  root["summary"]["peak_unpaired_image_bytes"] = request.peak_unpaired_image_bytes;
  root["summary"]["buffered_image_bytes"] = request.buffered_image_bytes;
  root["limits"]["image_count"] = request.image_count_limit;
  root["limits"]["image_bytes"] = request.image_bytes_limit;
  root["warnings"] = string_sequence(request.input_warnings);
  root["rejection_reasons"] = string_sequence(result.rejection_reasons);

  YAML::Node records(YAML::NodeType::Sequence);
  for (const auto & item : evidence) {
    const auto & frame = *item.input;
    YAML::Node record(YAML::NodeType::Map);
    record["source_image"] = frame.source_image;
    record["image_path"] =
      item.archived_path.empty() ? YAML::Node(YAML::NodeType::Null) : YAML::Node(item.archived_path);
    record["sha256"] = item.archived_sha256.empty() ?
      YAML::Node(YAML::NodeType::Null) : YAML::Node(item.archived_sha256);
    record["stamp"]["sec"] = frame.stamp.sec;
    record["stamp"]["nanosec"] = frame.stamp.nanosec;
    record["stamp"]["source"] = frame.stamp.source;
    record["width"] = frame.width;
    record["height"] = frame.height;
    record["encoding"] = frame.encoding;
    record["step"] = frame.step;
    record["data_size"] = frame.declared_data_size;
    record["frame_id"] = frame.frame_id;
    record["camera_info"]["present"] = frame.camera_info.has_value();
    record["camera_info"]["required"] = request.config.camera_info_required;
    record["camera_info"]["timestamp_matches"] =
      frame.camera_info.has_value() && item.camera_info_stamp_matches;
    record["camera_info"]["dimensions_match"] =
      frame.camera_info.has_value() && item.camera_info_dimensions_match;
    record["camera_info"]["frame_id_matches"] =
      frame.camera_info.has_value() && item.camera_info_frame_id_matches;
    if (frame.camera_info.has_value()) {
      record["camera_info"]["stamp"]["sec"] = frame.camera_info->stamp.sec;
      record["camera_info"]["stamp"]["nanosec"] = frame.camera_info->stamp.nanosec;
      record["camera_info"]["stamp"]["source"] = frame.camera_info->stamp.source;
      record["camera_info"]["width"] = frame.camera_info->width;
      record["camera_info"]["height"] = frame.camera_info->height;
      record["camera_info"]["frame_id"] = frame.camera_info->frame_id;
    }
    record["status"] = item.errors.empty() ? "accepted" : "rejected";
    record["errors"] = string_sequence(item.errors);
    record["warnings"] = string_sequence(item.warnings);
    records.push_back(record);
  }
  root["records"] = records;
  write_yaml(root, result.manifest_path);
  write_hash_sidecar(&result);
  if (result.accepted) {
    write_calibration_handoff(request, output_directory, git_commit, &result);
  }
  return result;
}

DatasetResult write_input_failure_manifest(
  const std::filesystem::path & output_directory, const std::string & source_mode,
  const std::string & git_commit, const std::vector<std::string> & reasons)
{
  create_new_output_directory(output_directory);
  DatasetResult result;
  result.accepted = false;
  result.rejection_reasons = reasons.empty() ?
    std::vector<std::string>{"input_unavailable"} : reasons;
  result.manifest_path = output_directory / "dataset_manifest.yaml";
  auto root = base_manifest(source_mode, git_commit, "input_unavailable");
  root["status"] = "rejected";
  root["quality_gate_passed"] = false;
  root["camera_info_required"] = source_mode == "ros";
  root["summary"]["record_count"] = 0;
  root["summary"]["archived_image_count"] = 0;
  root["rejection_reasons"] = string_sequence(result.rejection_reasons);
  root["records"] = YAML::Node(YAML::NodeType::Sequence);
  write_yaml(root, result.manifest_path);
  write_hash_sidecar(&result);
  return result;
}

int dataset_exit_code(const DatasetResult & result) noexcept
{
  return result.accepted ? 0 : 2;
}

}  // namespace auto_aim_tools::calibration_dataset
