#ifndef AUTO_AIM_ROS2__CAMERA_CALIBRATION_HPP_
#define AUTO_AIM_ROS2__CAMERA_CALIBRATION_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm_auto_aim::camera_calibration
{

inline constexpr char kEvidenceOnlyProfile[] = "evidence_only";
inline constexpr char kChessboardType[] = "chessboard";
inline constexpr char kRawImagePipeline[] = "raw_image_no_resize_no_crop_no_rectify";
inline constexpr char kCameraOpticalFrame[] = "opencv_camera_optical";

// Both dimensions below are counts of *inner* chessboard corners, not counts
// of squares.  For example, a 9 x 6 board has 10 x 7 alternating squares.
struct ChessboardSpecification
{
  std::string type{kChessboardType};
  int inner_corners_cols{0};
  int inner_corners_rows{0};
  double square_size_m{0.0};
};

struct ImageSpecification
{
  int width_px{0};
  int height_px{0};
  std::string pipeline{kRawImagePipeline};
};

struct AcceptanceCriteria
{
  int min_accepted_views{0};
  double max_global_rms_reprojection_error_px{0.0};
};

struct AcquisitionMetadata
{
  std::string report_id;
  std::string camera_serial;
  std::string lens_identifier;
  std::string acquisition_date;
  std::string operator_identifier;
  std::string code_commit;
  std::string dataset_id;
  std::optional<std::string> dataset_manifest;
  std::optional<std::string> dataset_manifest_sha256;
};

struct CalibrationInput
{
  int schema_version{0};
  std::string profile;
  ChessboardSpecification board;
  ImageSpecification image;
  AcceptanceCriteria acceptance;
  AcquisitionMetadata metadata;

  // This is deliberately an input schema for an evidence-only tool.  It is
  // not a PnP configuration and contains no geometry or extrinsic fields.
  std::optional<std::string> validate() const;
};

enum class ImageFailureReason : std::uint8_t
{
  None = 0,
  ImageUnreadable,
  ImageDimensionsMismatch,
  ChessboardNotFound,
  CornerRefinementFailed,
  InvalidDetectedCorners,
};

const char * image_failure_reason_name(ImageFailureReason reason) noexcept;

struct ImageEvidence
{
  std::string input_file;
  int observed_width_px{0};
  int observed_height_px{0};
  bool accepted{false};
  ImageFailureReason failure_reason{ImageFailureReason::None};
  std::string failure_detail;
  std::optional<double> reprojection_error_px;
};

// A corner-only view is intentionally public so the calibration math can be
// tested from cv::projectPoints observations without a camera, ROS node, or
// image acquisition device.  The CLI obtains the same type only after
// findChessboardCorners and cornerSubPix accept a local image file.
struct DetectedChessboardView
{
  std::string input_file;
  int image_width_px{0};
  int image_height_px{0};
  std::vector<cv::Point2f> corners;
};

struct CalibrationResult
{
  CalibrationInput input;
  std::vector<ImageEvidence> image_evidence;
  bool calibration_succeeded{false};
  bool quality_accepted{false};
  std::vector<std::string> rejection_reasons;
  cv::Matx33d camera_matrix{};
  std::vector<double> distortion_coefficients;
  std::optional<double> global_rms_reprojection_error_px;
};

// Strictly parse the versioned evidence-only input YAML.  Invalid input is
// rejected with std::runtime_error; no inferred image size, board, or camera
// parameters are ever used.
CalibrationInput load_calibration_input(const std::string & yaml_path);

// A linked dataset is optional for backward-compatible schema v1 inputs. If
// the metadata declares one, the CLI requires the actual manifest and verifies
// SHA-256 over its exact archived bytes before reading calibration images.
void verify_dataset_manifest(
  const CalibrationInput & input, const std::string & manifest_path);

// For linked inputs, the accepted records in dataset_manifest.yaml are the
// authority. The declared manifest path, image-list entries, archive paths,
// and every PNG SHA-256 are verified before any image is decoded. Legacy
// unlinked schema-v1 inputs retain the image-list-only behavior.
std::vector<std::string> load_verified_calibration_images(
  const CalibrationInput & input, const std::string & calibration_input_path,
  const std::string & manifest_path, const std::string & image_list_path);

// SHA-256 over exact file bytes, shared by manifest verification and tests.
std::string sha256_file(const std::string & path);

// Reads one image path per non-empty line.  An empty manifest is returned as
// an empty vector so the caller can still write a rejected evidence report.
std::vector<std::string> load_image_manifest(const std::string & image_list_path);

// Calibrates already-detected chessboard observations.  This is offline math
// only and makes no hardware, ROS, serial, or control calls.
CalibrationResult calibrate_detected_views(
  const CalibrationInput & input,
  const std::vector<DetectedChessboardView> & views);

// Reads local images with imread, detects chessboards, refines corners, then
// invokes the same calibrateCamera path as calibrate_detected_views().
CalibrationResult run_image_calibration(
  const CalibrationInput & input,
  const std::vector<std::string> & image_paths);

// The CLI returns this status after it has successfully written the evidence
// report.  A failed quality gate is deliberately non-zero even if a numerical
// candidate was computed, so a caller cannot mistake evidence for acceptance.
int quality_exit_code(const CalibrationResult & result) noexcept;

// Writes a self-contained evidence report.  Its root keys are deliberately
// incompatible with the project's PnP configuration schema; production_ready
// is always false regardless of the quality gate result.
void write_evidence_report(const CalibrationResult & result, const std::string & report_path);

}  // namespace rm_auto_aim::camera_calibration

#endif  // AUTO_AIM_ROS2__CAMERA_CALIBRATION_HPP_
