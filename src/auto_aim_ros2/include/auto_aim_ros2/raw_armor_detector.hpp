#ifndef AUTO_AIM_ROS2__RAW_ARMOR_DETECTOR_HPP_
#define AUTO_AIM_ROS2__RAW_ARMOR_DETECTOR_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "auto_aim_ros2/auto_aim_core.hpp"

namespace rm_auto_aim::detector
{

// A detector output is deliberately limited to image-space evidence.  It does
// not contain yaw/pitch, target state, or any fire command.  PnP and the later
// tracking stages must consume this boundary only after camera conventions are
// confirmed.
struct RawArmorDetection
{
  enum class ArmorTypeHint : std::uint8_t { Unknown = 0, Small = 1, Large = 2 };

  int class_id{-1};
  int color_id{-1};
  ArmorTypeHint armor_type{ArmorTypeHint::Unknown};
  float confidence{0.0F};
  cv::Rect2f bbox{};
  std::array<cv::Point2f, 4> keypoints{};  // top-left, top-right, bottom-right, bottom-left
};

// Construct a raw detection with all safety checks at the detector boundary.
// class_count/color_count may be -1 when the model has no known upper bound.
std::optional<RawArmorDetection> make_raw_armor_detection(
  int class_id,
  int color_id,
  float confidence,
  const cv::Rect2f & bbox,
  const std::vector<cv::Point2f> & keypoints,
  int class_count = -1,
  int color_count = -1);

struct DetectorConfig
{
  std::string model_path;

  // A production profile binds the runtime artifact to the exact path that
  // was reviewed.  Test-only profiles may use an external:// identifier and
  // explicitly override it for offline fixtures; production profiles may not.
  bool require_model_path_match{false};
  std::string reviewed_model_path;
  std::string device{"CPU"};

  // Optional semantic mapping from the reviewed model profile.  An empty map
  // preserves the legacy smoke path; a non-empty map is applied to every
  // emitted RawArmorDetection and unknown class ids fail closed.
  std::map<int, RawArmorDetection::ArmorTypeHint> class_to_armor_type;

  // When non-empty, the model's declared element types are checked against
  // the versioned model profile.  Empty input type keeps the legacy smoke
  // path compatible; output remains FP32 by default because the parser below
  // consumes float logits.
  std::string expected_input_element_type;
  std::string expected_output_element_type{"f32"};

  // The first adapter targets the supplied YOLOv5 IR model.  These values are
  // explicit configuration, not a claim that a future competition model has
  // the same contract.
  int input_width{640};
  int input_height{640};
  std::size_t expected_output_rows{25200};
  std::size_t expected_output_columns{22};
  std::size_t color_class_count{4};
  std::size_t armor_class_count{9};

  // Fixed row contract for the reference YOLO export.  These offsets are
  // explicit configuration so a future model cannot silently reuse a
  // different output layout.
  std::size_t objectness_index{8};
  std::size_t color_logits_offset{9};
  std::size_t armor_logits_offset{13};

  float objectness_threshold{0.7F};
  float nms_threshold{0.3F};

  // The reference YOLOv5 export places the resized image at the top-left of
  // the square tensor.  Center padding is opt-in for a different model.
  bool center_padding{false};

  // Raw coordinate pair index -> output point index.  The reference model's
  // postprocess order is [0, 3, 2, 1].
  std::array<int, 4> keypoint_order{{0, 3, 2, 1}};
};

// A model profile is a versioned, reviewable description of a detector
// artifact's tensor and semantic contract.  It is intentionally separate
// from the physical PnP/calibration YAML.  No production profile is shipped
// until the competition model has been measured and its class semantics have
// been confirmed.
struct ModelProfile
{
  int schema_version{0};
  bool test_only{false};
  std::string model_id;
  // For production this must be an absolute local artifact path and is bound
  // to the runtime path.  Test-only profiles may use an external:// identifier
  // because their model is intentionally supplied outside the repository.
  std::string model_path;
  std::string source;
  std::string version;

  std::array<std::size_t, 4> input_shape{};  // N,C,H,W
  std::string input_layout;
  std::string input_element_type;
  std::string source_color_order;
  std::string model_color_order;
  std::string normalization;
  std::string resize_mode;

  std::array<std::size_t, 3> output_shape{};  // N,rows,columns
  std::string output_layout;
  std::string output_element_type;
  std::size_t keypoint_count{0};
  std::size_t objectness_index{0};
  std::size_t color_logits_offset{0};
  std::size_t color_class_count{0};
  std::size_t armor_logits_offset{0};
  std::size_t armor_class_count{0};

  float objectness_threshold{0.0F};
  float nms_threshold{0.0F};
  std::array<int, 4> keypoint_order{};
  std::vector<std::string> color_names;
  std::vector<std::string> armor_names;
  std::map<int, RawArmorDetection::ArmorTypeHint> class_to_armor_type;

  std::optional<std::string> validate() const;
};

struct ModelProfileLoadOptions
{
  // A test-only profile is rejected by default.  Only offline tools/tests may
  // opt in explicitly; runtime/control paths should leave this false.
  bool allow_test_only{false};
};

// Parse and validate a model profile without loading or connecting to a model
// device.  The path is never inferred from old repositories or local caches.
ModelProfile load_model_profile(
  const std::string & yaml_path,
  ModelProfileLoadOptions options = {});

// Convert a validated profile into the detector's runtime contract.  The
// caller still supplies the actual model artifact path and OpenVINO device.
DetectorConfig detector_config_from_model_profile(
  const ModelProfile & profile,
  std::string model_path,
  std::string device = "CPU");

struct ModelInfo
{
  std::vector<std::size_t> input_shape;
  std::vector<std::size_t> output_shape;
  std::string input_element_type;
  std::string output_element_type;
};

// Returns an error description when the model output is not the configured
// [1, rows, columns] raw-detection contract.
std::optional<std::string> validate_output_shape(
  const std::vector<std::size_t> & shape,
  const DetectorConfig & config);

// Numerically stable objectness conversion.  Non-finite model logits are
// rejected instead of allowing +/-Inf to become a seemingly valid confidence.
std::optional<float> sigmoid_probability(float value) noexcept;

// Returns an error description when the model input is not the configured
// static NCHW [1, 3, input_height, input_width] contract.
std::optional<std::string> validate_input_shape(
  const std::vector<std::size_t> & shape,
  const DetectorConfig & config);

struct LetterboxResult
{
  cv::Mat image;
  float scale{0.0F};
  int pad_x{0};
  int pad_y{0};
};

// Converts a BGR CV_8UC3 image to the configured tensor canvas.  Empty or
// unsupported images return nullopt instead of throwing.
std::optional<LetterboxResult> make_letterbox(
  const cv::Mat & bgr_image,
  const DetectorConfig & config);

// Convert one point from model-canvas pixels back to the original image
// pixels.  This is the only inverse of the letterbox transform used by the
// detector; it removes padding first and then divides by the resize scale.
// Points are not clamped, because the caller must decide how to handle model
// evidence outside the original image bounds.
std::optional<cv::Point2f> model_point_to_image(
  const cv::Point2f & model_point,
  const LetterboxResult & letterbox) noexcept;

class OpenVinoYoloDetector final
{
public:
  explicit OpenVinoYoloDetector(DetectorConfig config);
  ~OpenVinoYoloDetector();

  OpenVinoYoloDetector(OpenVinoYoloDetector &&) noexcept;
  OpenVinoYoloDetector & operator=(OpenVinoYoloDetector &&) noexcept;
  OpenVinoYoloDetector(const OpenVinoYoloDetector &) = delete;
  OpenVinoYoloDetector & operator=(const OpenVinoYoloDetector &) = delete;

  // Empty ImageFrame or non-BGR pixels produce an empty result.  No later
  // pipeline stage is called from this detector.
  std::vector<RawArmorDetection> detect(const pipeline::ImageFrame & frame) const;

  const ModelInfo & model_info() const;

  static cv::Mat annotate(
    const cv::Mat & bgr_image,
    const std::vector<RawArmorDetection> & detections);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rm_auto_aim::detector

#endif  // AUTO_AIM_ROS2__RAW_ARMOR_DETECTOR_HPP_
