#include "auto_aim_ros2/camera_calibration.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace rm_auto_aim::camera_calibration
{
namespace
{

constexpr int kSchemaVersion = 1;
constexpr char kReportType[] = "camera_intrinsic_calibration_evidence";
constexpr char kEvidenceReportVersion[] = "camera_intrinsic_evidence_v1";

std::string trim_copy(std::string value)
{
  const auto is_space = [](unsigned char character) { return std::isspace(character) != 0; };
  const auto first = std::find_if_not(value.begin(), value.end(), is_space);
  const auto last = std::find_if_not(value.rbegin(), value.rend(), is_space).base();
  if (first >= last) {
    return {};
  }
  return std::string(first, last);
}

bool finite(double value) noexcept
{
  return std::isfinite(value);
}

bool finite_point(const cv::Point2f & point) noexcept
{
  return finite(point.x) && finite(point.y);
}

bool valid_date(const std::string & value)
{
  static const std::regex kDatePattern(R"(^([0-9]{4})-([0-9]{2})-([0-9]{2})$)");
  std::smatch match;
  if (!std::regex_match(value, match, kDatePattern)) {
    return false;
  }

  const int year = std::stoi(match[1].str());
  const int month = std::stoi(match[2].str());
  const int day = std::stoi(match[3].str());
  if (year < 1970 || month < 1 || month > 12) {
    return false;
  }
  static constexpr int kDaysInMonth[] =
    {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  int max_day = kDaysInMonth[month - 1];
  const bool leap_year = (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
  if (month == 2 && leap_year) {
    max_day = 29;
  }
  return day >= 1 && day <= max_day;
}

bool valid_commit_sha(const std::string & value)
{
  static const std::regex kCommitPattern(R"(^[0-9a-fA-F]{7,64}$)");
  return std::regex_match(value, kCommitPattern);
}

YAML::Node required_node(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(path + ": missing required field '" + key + "'");
  }
  return node;
}

std::string required_string(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const auto value = trim_copy(required_node(parent, key, path).as<std::string>());
    if (value.empty()) {
      throw std::runtime_error(path + ": field '" + key + "' must not be empty");
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must be a non-empty string");
  }
}

int required_int(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    return required_node(parent, key, path).as<int>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must be an integer");
  }
}

double required_double(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const double value = required_node(parent, key, path).as<double>();
    if (!finite(value)) {
      throw std::runtime_error(path + ": field '" + key + "' must be finite");
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must be a finite number");
  }
}

YAML::Node required_map(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  const auto node = required_node(parent, key, path);
  if (!node.IsMap()) {
    throw std::runtime_error(path + ": field '" + key + "' must be a map");
  }
  return node;
}

std::vector<cv::Point3f> object_points(const ChessboardSpecification & board)
{
  std::vector<cv::Point3f> result;
  result.reserve(static_cast<std::size_t>(board.inner_corners_cols * board.inner_corners_rows));
  for (int row = 0; row < board.inner_corners_rows; ++row) {
    for (int col = 0; col < board.inner_corners_cols; ++col) {
      result.emplace_back(
        static_cast<float>(static_cast<double>(col) * board.square_size_m),
        static_cast<float>(static_cast<double>(row) * board.square_size_m),
        0.0F);
    }
  }
  return result;
}

bool valid_calibration_candidate(
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients)
{
  if (camera_matrix.rows != 3 || camera_matrix.cols != 3 || camera_matrix.type() != CV_64F ||
    distortion_coefficients.empty())
  {
    return false;
  }
  if (!finite(camera_matrix.at<double>(0, 0)) || !finite(camera_matrix.at<double>(1, 1)) ||
    camera_matrix.at<double>(0, 0) <= 0.0 || camera_matrix.at<double>(1, 1) <= 0.0)
  {
    return false;
  }
  for (int row = 0; row < camera_matrix.rows; ++row) {
    for (int col = 0; col < camera_matrix.cols; ++col) {
      if (!finite(camera_matrix.at<double>(row, col))) {
        return false;
      }
    }
  }

  cv::Mat distortion64;
  distortion_coefficients.reshape(1, 1).convertTo(distortion64, CV_64F);
  for (int index = 0; index < distortion64.cols; ++index) {
    if (!finite(distortion64.at<double>(0, index))) {
      return false;
    }
  }
  return true;
}

void store_calibration_candidate(
  CalibrationResult * result,
  const cv::Mat & camera_matrix,
  const cv::Mat & distortion_coefficients,
  double global_rms)
{
  if (result == nullptr || !finite(global_rms) ||
    !valid_calibration_candidate(camera_matrix, distortion_coefficients))
  {
    return;
  }

  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result->camera_matrix(row, col) = camera_matrix.at<double>(row, col);
    }
  }
  cv::Mat distortion64;
  distortion_coefficients.reshape(1, 1).convertTo(distortion64, CV_64F);
  result->distortion_coefficients.clear();
  result->distortion_coefficients.reserve(static_cast<std::size_t>(distortion64.cols));
  for (int index = 0; index < distortion64.cols; ++index) {
    result->distortion_coefficients.push_back(distortion64.at<double>(0, index));
  }
  result->global_rms_reprojection_error_px = global_rms;
  result->calibration_succeeded = true;
}

void finish_calibration(
  CalibrationResult * result,
  const std::vector<DetectedChessboardView> & accepted_views,
  const std::vector<std::size_t> & evidence_indices)
{
  if (result == nullptr || accepted_views.size() != evidence_indices.size()) {
    throw std::logic_error("internal calibration evidence bookkeeping mismatch");
  }

  if (!accepted_views.empty()) {
    try {
      const auto board_points = object_points(result->input.board);
      std::vector<std::vector<cv::Point3f>> all_object_points(
        accepted_views.size(), board_points);
      std::vector<std::vector<cv::Point2f>> all_image_points;
      all_image_points.reserve(accepted_views.size());
      for (const auto & view : accepted_views) {
        all_image_points.push_back(view.corners);
      }

      cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
      cv::Mat distortion_coefficients;
      std::vector<cv::Mat> rotation_vectors;
      std::vector<cv::Mat> translation_vectors;
      const double global_rms = cv::calibrateCamera(
        all_object_points, all_image_points,
        cv::Size(result->input.image.width_px, result->input.image.height_px),
        camera_matrix, distortion_coefficients, rotation_vectors, translation_vectors);
      store_calibration_candidate(result, camera_matrix, distortion_coefficients, global_rms);
      if (!result->calibration_succeeded || rotation_vectors.size() != accepted_views.size() ||
        translation_vectors.size() != accepted_views.size())
      {
        result->calibration_succeeded = false;
        result->global_rms_reprojection_error_px.reset();
        result->distortion_coefficients.clear();
        result->rejection_reasons.push_back("invalid_calibration_result");
      } else {
        const cv::Mat candidate_matrix(result->camera_matrix);
        const cv::Mat candidate_distortion(result->distortion_coefficients);
        for (std::size_t index = 0; index < accepted_views.size(); ++index) {
          std::vector<cv::Point2f> reprojected;
          cv::projectPoints(
            board_points, rotation_vectors[index], translation_vectors[index], candidate_matrix,
            candidate_distortion, reprojected);
          if (reprojected.size() != accepted_views[index].corners.size()) {
            throw std::runtime_error("projectPoints returned an unexpected corner count");
          }
          double squared_error = 0.0;
          for (std::size_t corner = 0; corner < reprojected.size(); ++corner) {
            const double dx = static_cast<double>(reprojected[corner].x) -
              static_cast<double>(accepted_views[index].corners[corner].x);
            const double dy = static_cast<double>(reprojected[corner].y) -
              static_cast<double>(accepted_views[index].corners[corner].y);
            squared_error += dx * dx + dy * dy;
          }
          const double view_rms = std::sqrt(squared_error / static_cast<double>(reprojected.size()));
          if (!finite(view_rms)) {
            throw std::runtime_error("non-finite per-image reprojection error");
          }
          result->image_evidence[evidence_indices[index]].reprojection_error_px = view_rms;
        }
      }
    } catch (const cv::Exception & error) {
      result->calibration_succeeded = false;
      result->global_rms_reprojection_error_px.reset();
      result->distortion_coefficients.clear();
      result->rejection_reasons.push_back("calibrate_camera_failed: " + std::string(error.what()));
    } catch (const std::exception & error) {
      result->calibration_succeeded = false;
      result->global_rms_reprojection_error_px.reset();
      result->distortion_coefficients.clear();
      result->rejection_reasons.push_back("calibrate_camera_failed: " + std::string(error.what()));
    }
  }

  if (static_cast<int>(accepted_views.size()) < result->input.acceptance.min_accepted_views) {
    std::ostringstream reason;
    reason << "insufficient_accepted_views: got " << accepted_views.size() << ", require " <<
      result->input.acceptance.min_accepted_views;
    result->rejection_reasons.push_back(reason.str());
  }
  if (!result->calibration_succeeded && accepted_views.empty()) {
    result->rejection_reasons.push_back("no_accepted_chessboard_views");
  } else if (!result->calibration_succeeded && result->rejection_reasons.empty()) {
    result->rejection_reasons.push_back("invalid_calibration_result");
  }
  if (result->calibration_succeeded && result->global_rms_reprojection_error_px.has_value() &&
    *result->global_rms_reprojection_error_px >
    result->input.acceptance.max_global_rms_reprojection_error_px)
  {
    std::ostringstream reason;
    reason << "global_rms_exceeds_threshold: " << *result->global_rms_reprojection_error_px << " > " <<
      result->input.acceptance.max_global_rms_reprojection_error_px;
    result->rejection_reasons.push_back(reason.str());
  }
  result->quality_accepted = result->calibration_succeeded && result->rejection_reasons.empty();
}

YAML::Node null_node()
{
  return YAML::Node(YAML::NodeType::Null);
}

YAML::Node matrix_node(const cv::Matx33d & matrix)
{
  YAML::Node result(YAML::NodeType::Sequence);
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result.push_back(matrix(row, col));
    }
  }
  return result;
}

YAML::Node vector_node(const std::vector<double> & values)
{
  YAML::Node result(YAML::NodeType::Sequence);
  for (const double value : values) {
    result.push_back(value);
  }
  return result;
}

void set_candidate_or_null(
  YAML::Node & parent,
  const CalibrationResult & result,
  const std::string & matrix_key,
  const std::string & distortion_key)
{
  if (result.quality_accepted) {
    parent[matrix_key] = matrix_node(result.camera_matrix);
    parent[distortion_key] = vector_node(result.distortion_coefficients);
  } else {
    parent[matrix_key] = null_node();
    parent[distortion_key] = null_node();
  }
}

bool protected_camera_info_path(const std::string & path)
{
  return std::filesystem::path(path).filename() == "camera_info.yaml";
}

}  // namespace

std::optional<std::string> CalibrationInput::validate() const
{
  if (schema_version != kSchemaVersion) {
    return "unsupported schema_version; expected 1";
  }
  if (profile != kEvidenceOnlyProfile) {
    return "profile must be evidence_only";
  }
  if (board.type != kChessboardType) {
    return "board.type must be chessboard";
  }
  if (board.inner_corners_cols < 2 || board.inner_corners_rows < 2) {
    return "board inner_corners_cols and inner_corners_rows must both be at least 2";
  }
  if (!finite(board.square_size_m) || board.square_size_m <= 0.0) {
    return "board.square_size_m must be finite and positive";
  }
  if (image.width_px <= 0 || image.height_px <= 0) {
    return "image.width_px and image.height_px must be positive";
  }
  if (image.pipeline != kRawImagePipeline) {
    return "image.pipeline must be raw_image_no_resize_no_crop_no_rectify";
  }
  if (acceptance.min_accepted_views <= 0) {
    return "acceptance.min_accepted_views must be positive";
  }
  if (!finite(acceptance.max_global_rms_reprojection_error_px) ||
    acceptance.max_global_rms_reprojection_error_px <= 0.0)
  {
    return "acceptance.max_global_rms_reprojection_error_px must be finite and positive";
  }
  if (metadata.report_id.empty() || metadata.camera_serial.empty() || metadata.lens_identifier.empty() ||
    metadata.acquisition_date.empty() || metadata.operator_identifier.empty() ||
    metadata.code_commit.empty() || metadata.dataset_id.empty())
  {
    return "all metadata fields are required";
  }
  if (!valid_date(metadata.acquisition_date)) {
    return "metadata.acquisition_date must be a valid YYYY-MM-DD date";
  }
  if (!valid_commit_sha(metadata.code_commit)) {
    return "metadata.code_commit must be a 7 to 64 character hexadecimal commit SHA";
  }
  return std::nullopt;
}

const char * image_failure_reason_name(ImageFailureReason reason) noexcept
{
  switch (reason) {
    case ImageFailureReason::None:
      return "none";
    case ImageFailureReason::ImageUnreadable:
      return "image_unreadable";
    case ImageFailureReason::ImageDimensionsMismatch:
      return "image_dimensions_mismatch";
    case ImageFailureReason::ChessboardNotFound:
      return "chessboard_not_found";
    case ImageFailureReason::CornerRefinementFailed:
      return "corner_refinement_failed";
    case ImageFailureReason::InvalidDetectedCorners:
      return "invalid_detected_corners";
  }
  return "unknown";
}

CalibrationInput load_calibration_input(const std::string & yaml_path)
{
  if (yaml_path.empty() || !std::filesystem::is_regular_file(yaml_path)) {
    throw std::runtime_error("calibration input YAML does not exist or is not a regular file: " + yaml_path);
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse calibration input " + yaml_path + ": " + error.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("calibration input root must be a map: " + yaml_path);
  }

  CalibrationInput result{};
  result.schema_version = required_int(root, "schema_version", "root");
  result.profile = required_string(root, "profile", "root");

  const auto board = required_map(root, "board", "root");
  result.board.type = required_string(board, "type", "board");
  result.board.inner_corners_cols = required_int(board, "inner_corners_cols", "board");
  result.board.inner_corners_rows = required_int(board, "inner_corners_rows", "board");
  result.board.square_size_m = required_double(board, "square_size_m", "board");

  const auto image = required_map(root, "image", "root");
  result.image.width_px = required_int(image, "width_px", "image");
  result.image.height_px = required_int(image, "height_px", "image");
  result.image.pipeline = required_string(image, "pipeline", "image");

  const auto acceptance = required_map(root, "acceptance", "root");
  result.acceptance.min_accepted_views = required_int(
    acceptance, "min_accepted_views", "acceptance");
  result.acceptance.max_global_rms_reprojection_error_px = required_double(
    acceptance, "max_global_rms_reprojection_error_px", "acceptance");

  const auto metadata = required_map(root, "metadata", "root");
  result.metadata.report_id = required_string(metadata, "report_id", "metadata");
  result.metadata.camera_serial = required_string(metadata, "camera_serial", "metadata");
  result.metadata.lens_identifier = required_string(metadata, "lens_identifier", "metadata");
  result.metadata.acquisition_date = required_string(metadata, "acquisition_date", "metadata");
  result.metadata.operator_identifier = required_string(metadata, "operator", "metadata");
  result.metadata.code_commit = required_string(metadata, "code_commit", "metadata");
  result.metadata.dataset_id = required_string(metadata, "dataset_id", "metadata");

  if (const auto error = result.validate(); error.has_value()) {
    throw std::runtime_error("invalid calibration input " + yaml_path + ": " + *error);
  }
  return result;
}

std::vector<std::string> load_image_manifest(const std::string & image_list_path)
{
  if (image_list_path.empty() || !std::filesystem::is_regular_file(image_list_path)) {
    throw std::runtime_error("image list does not exist or is not a regular file: " + image_list_path);
  }
  std::ifstream input(image_list_path);
  if (!input) {
    throw std::runtime_error("cannot read image list: " + image_list_path);
  }

  std::vector<std::string> result;
  std::string line;
  while (std::getline(input, line)) {
    const auto path = trim_copy(std::move(line));
    if (!path.empty()) {
      result.push_back(path);
    }
  }
  if (input.bad()) {
    throw std::runtime_error("failed while reading image list: " + image_list_path);
  }
  return result;
}

CalibrationResult calibrate_detected_views(
  const CalibrationInput & input,
  const std::vector<DetectedChessboardView> & views)
{
  if (const auto error = input.validate(); error.has_value()) {
    throw std::invalid_argument("invalid calibration input: " + *error);
  }

  CalibrationResult result{};
  result.input = input;
  const auto expected_corner_count = static_cast<std::size_t>(
    input.board.inner_corners_cols * input.board.inner_corners_rows);
  std::vector<DetectedChessboardView> accepted_views;
  std::vector<std::size_t> evidence_indices;

  for (const auto & view : views) {
    ImageEvidence evidence{};
    evidence.input_file = view.input_file;
    evidence.observed_width_px = view.image_width_px;
    evidence.observed_height_px = view.image_height_px;
    if (view.image_width_px != input.image.width_px || view.image_height_px != input.image.height_px) {
      evidence.failure_reason = ImageFailureReason::ImageDimensionsMismatch;
      evidence.failure_detail = "observed " + std::to_string(view.image_width_px) + "x" +
        std::to_string(view.image_height_px) + ", expected " + std::to_string(input.image.width_px) +
        "x" + std::to_string(input.image.height_px);
    } else if (view.corners.size() != expected_corner_count ||
      !std::all_of(view.corners.begin(), view.corners.end(), finite_point))
    {
      evidence.failure_reason = ImageFailureReason::InvalidDetectedCorners;
      evidence.failure_detail = "expected " + std::to_string(expected_corner_count) +
        " finite chessboard inner corners";
    } else {
      evidence.accepted = true;
      accepted_views.push_back(view);
      evidence_indices.push_back(result.image_evidence.size());
    }
    result.image_evidence.push_back(std::move(evidence));
  }

  finish_calibration(&result, accepted_views, evidence_indices);
  return result;
}

CalibrationResult run_image_calibration(
  const CalibrationInput & input,
  const std::vector<std::string> & image_paths)
{
  if (const auto error = input.validate(); error.has_value()) {
    throw std::invalid_argument("invalid calibration input: " + *error);
  }

  CalibrationResult result{};
  result.input = input;
  const auto expected_corner_count = static_cast<std::size_t>(
    input.board.inner_corners_cols * input.board.inner_corners_rows);
  std::vector<DetectedChessboardView> accepted_views;
  std::vector<std::size_t> evidence_indices;

  for (const auto & image_path : image_paths) {
    ImageEvidence evidence{};
    evidence.input_file = image_path;
    const cv::Mat bgr_image = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr_image.empty()) {
      evidence.failure_reason = ImageFailureReason::ImageUnreadable;
      evidence.failure_detail = "cv::imread returned an empty image";
      result.image_evidence.push_back(std::move(evidence));
      continue;
    }

    evidence.observed_width_px = bgr_image.cols;
    evidence.observed_height_px = bgr_image.rows;
    if (bgr_image.cols != input.image.width_px || bgr_image.rows != input.image.height_px) {
      evidence.failure_reason = ImageFailureReason::ImageDimensionsMismatch;
      evidence.failure_detail = "observed " + std::to_string(bgr_image.cols) + "x" +
        std::to_string(bgr_image.rows) + ", expected " + std::to_string(input.image.width_px) +
        "x" + std::to_string(input.image.height_px);
      result.image_evidence.push_back(std::move(evidence));
      continue;
    }

    try {
      cv::Mat grayscale;
      cv::cvtColor(bgr_image, grayscale, cv::COLOR_BGR2GRAY);
      std::vector<cv::Point2f> corners;
      const bool found = cv::findChessboardCorners(
        grayscale,
        cv::Size(input.board.inner_corners_cols, input.board.inner_corners_rows),
        corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
      if (!found || corners.size() != expected_corner_count) {
        evidence.failure_reason = ImageFailureReason::ChessboardNotFound;
        evidence.failure_detail = "findChessboardCorners did not find the configured inner-corner pattern";
        result.image_evidence.push_back(std::move(evidence));
        continue;
      }
      cv::cornerSubPix(
        grayscale, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
      if (!std::all_of(corners.begin(), corners.end(), finite_point)) {
        evidence.failure_reason = ImageFailureReason::CornerRefinementFailed;
        evidence.failure_detail = "cornerSubPix produced non-finite coordinates";
        result.image_evidence.push_back(std::move(evidence));
        continue;
      }

      evidence.accepted = true;
      accepted_views.push_back(DetectedChessboardView{
        image_path, bgr_image.cols, bgr_image.rows, std::move(corners)});
      evidence_indices.push_back(result.image_evidence.size());
      result.image_evidence.push_back(std::move(evidence));
    } catch (const cv::Exception & error) {
      evidence.failure_reason = ImageFailureReason::CornerRefinementFailed;
      evidence.failure_detail = error.what();
      result.image_evidence.push_back(std::move(evidence));
    }
  }

  finish_calibration(&result, accepted_views, evidence_indices);
  return result;
}

int quality_exit_code(const CalibrationResult & result) noexcept
{
  return result.quality_accepted ? 0 : 2;
}

void write_evidence_report(const CalibrationResult & result, const std::string & report_path)
{
  if (report_path.empty()) {
    throw std::invalid_argument("report path must not be empty");
  }
  if (protected_camera_info_path(report_path)) {
    throw std::invalid_argument("refusing to overwrite any camera_info.yaml with an evidence report");
  }
  if (const auto error = result.input.validate(); error.has_value()) {
    throw std::invalid_argument("cannot write report for invalid calibration input: " + *error);
  }

  const auto accepted_count = static_cast<std::size_t>(std::count_if(
      result.image_evidence.begin(), result.image_evidence.end(),
      [](const ImageEvidence & evidence) { return evidence.accepted; }));

  YAML::Node root(YAML::NodeType::Map);
  root["report_schema_version"] = kSchemaVersion;
  root["report_type"] = kReportType;
  root["profile"] = kEvidenceOnlyProfile;
  root["production_ready"] = false;
  root["evidence_status"] = result.quality_accepted ? "accepted" : "rejected";
  root["quality_gate_passed"] = result.quality_accepted;

  auto board = root["input"]["board"];
  board["type"] = result.input.board.type;
  board["inner_corners_cols"] = result.input.board.inner_corners_cols;
  board["inner_corners_rows"] = result.input.board.inner_corners_rows;
  board["square_size_m"] = result.input.board.square_size_m;
  auto image = root["input"]["image"];
  image["width_px"] = result.input.image.width_px;
  image["height_px"] = result.input.image.height_px;
  image["pipeline"] = result.input.image.pipeline;
  auto acceptance = root["input"]["acceptance"];
  acceptance["min_accepted_views"] = result.input.acceptance.min_accepted_views;
  acceptance["max_global_rms_reprojection_error_px"] =
    result.input.acceptance.max_global_rms_reprojection_error_px;
  auto metadata = root["input"]["metadata"];
  metadata["report_id"] = result.input.metadata.report_id;
  metadata["camera_serial"] = result.input.metadata.camera_serial;
  metadata["lens_identifier"] = result.input.metadata.lens_identifier;
  metadata["acquisition_date"] = result.input.metadata.acquisition_date;
  metadata["operator"] = result.input.metadata.operator_identifier;
  metadata["code_commit"] = result.input.metadata.code_commit;
  metadata["dataset_id"] = result.input.metadata.dataset_id;

  auto summary = root["summary"];
  summary["input_file_count"] = result.image_evidence.size();
  summary["accepted_view_count"] = accepted_count;
  summary["rejected_view_count"] = result.image_evidence.size() - accepted_count;
  summary["global_rms_reprojection_error_px"] =
    result.global_rms_reprojection_error_px.has_value() ?
    YAML::Node(*result.global_rms_reprojection_error_px) : null_node();
  summary["calibration_succeeded"] = result.calibration_succeeded;

  auto calibration = root["calibration_result"];
  calibration["candidate_available"] = result.quality_accepted;
  set_candidate_or_null(calibration, result, "camera_matrix", "distortion_coefficients");

  YAML::Node reasons(YAML::NodeType::Sequence);
  for (const auto & reason : result.rejection_reasons) {
    reasons.push_back(reason);
  }
  root["rejection_reasons"] = reasons;

  YAML::Node images(YAML::NodeType::Sequence);
  for (const auto & evidence : result.image_evidence) {
    YAML::Node record(YAML::NodeType::Map);
    record["input_filename"] = evidence.input_file;
    record["observed_width_px"] = evidence.observed_width_px > 0 ?
      YAML::Node(evidence.observed_width_px) : null_node();
    record["observed_height_px"] = evidence.observed_height_px > 0 ?
      YAML::Node(evidence.observed_height_px) : null_node();
    record["status"] = evidence.accepted ? "accepted" : "rejected";
    record["failure_reason"] = image_failure_reason_name(evidence.failure_reason);
    record["failure_detail"] = evidence.failure_detail;
    record["reprojection_error_px"] = evidence.reprojection_error_px.has_value() ?
      YAML::Node(*evidence.reprojection_error_px) : null_node();
    images.push_back(record);
  }
  root["images"] = images;

  // This nested object is deliberately only a manual-review handoff.  It is
  // not named or shaped as the PnP root "camera" block and has no geometry,
  // extrinsic, aiming, serial, or firing fields.
  auto manual_review = root["pnp_camera_fields_for_manual_review"];
  manual_review["image_width"] = result.input.image.width_px;
  manual_review["image_height"] = result.input.image.height_px;
  set_candidate_or_null(
    manual_review, result, "camera_matrix", "distortion_coefficients");
  manual_review["source"] = "camera_intrinsic_evidence:" + result.input.metadata.report_id;
  manual_review["version"] = std::string(kEvidenceReportVersion) + ":" +
    result.input.metadata.code_commit;
  manual_review["coordinate_frame"] = kCameraOpticalFrame;

  YAML::Emitter emitter;
  emitter << root;
  if (!emitter.good()) {
    throw std::runtime_error("failed to encode evidence YAML report");
  }

  std::ofstream output(report_path, std::ios::out | std::ios::trunc);
  if (!output) {
    throw std::runtime_error("cannot open evidence report path: " + report_path);
  }
  output << emitter.c_str() << '\n';
  if (!output) {
    throw std::runtime_error("failed to write evidence report: " + report_path);
  }
}

}  // namespace rm_auto_aim::camera_calibration
