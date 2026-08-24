#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef AUTO_AIM_HAS_OPENVINO
#include <openvino/openvino.hpp>
#endif

namespace rm_auto_aim::detector
{
namespace
{
bool finite_point(const cv::Point2f & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finite_rect(const cv::Rect2f & rect)
{
  return std::isfinite(rect.x) && std::isfinite(rect.y) && std::isfinite(rect.width) &&
         std::isfinite(rect.height);
}

float sigmoid(float value)
{
  if (value >= 0.0F) {
    const float z = std::exp(-value);
    return 1.0F / (1.0F + z);
  }
  const float z = std::exp(value);
  return z / (1.0F + z);
}

int argmax(const float * values, std::size_t count)
{
  if (values == nullptr || count == 0) {
    return -1;
  }
  std::size_t best = 0;
  for (std::size_t index = 1; index < count; ++index) {
    if (values[index] > values[best]) {
      best = index;
    }
  }
  return static_cast<int>(best);
}

std::string shape_string(const std::vector<std::size_t> & shape)
{
  std::ostringstream stream;
  stream << '[';
  for (std::size_t index = 0; index < shape.size(); ++index) {
    if (index != 0) {
      stream << ',';
    }
    stream << shape[index];
  }
  stream << ']';
  return stream.str();
}

cv::Rect to_nms_rect(const cv::Rect2f & rect)
{
  const int x = static_cast<int>(std::floor(rect.x));
  const int y = static_cast<int>(std::floor(rect.y));
  const int width = std::max(1, static_cast<int>(std::ceil(rect.width)));
  const int height = std::max(1, static_cast<int>(std::ceil(rect.height)));
  return {x, y, width, height};
}

#ifdef AUTO_AIM_HAS_OPENVINO
std::vector<std::size_t> static_shape(const ov::Output<const ov::Node> & port, const char * label)
{
  const auto partial_shape = port.get_partial_shape();
  if (!partial_shape.is_static()) {
    throw std::runtime_error(std::string(label) + " shape is dynamic: " + partial_shape.to_string());
  }
  const auto shape = partial_shape.to_shape();
  return {shape.begin(), shape.end()};
}
#endif
}  // namespace

std::optional<RawArmorDetection> make_raw_armor_detection(
  int class_id,
  int color_id,
  float confidence,
  const cv::Rect2f & bbox,
  const std::vector<cv::Point2f> & keypoints,
  int class_count,
  int color_count)
{
  if (class_id < 0 || (class_count >= 0 && class_id >= class_count)) {
    return std::nullopt;
  }
  if (color_id < -1 || (color_count >= 0 && color_id >= color_count)) {
    return std::nullopt;
  }
  if (!std::isfinite(confidence) || confidence < 0.0F || confidence > 1.0F) {
    return std::nullopt;
  }
  if (!finite_rect(bbox) || bbox.width <= 0.0F || bbox.height <= 0.0F) {
    return std::nullopt;
  }
  if (keypoints.size() != 4) {
    return std::nullopt;
  }

  RawArmorDetection result{};
  result.class_id = class_id;
  result.color_id = color_id;
  result.confidence = confidence;
  result.bbox = bbox;
  for (std::size_t index = 0; index < result.keypoints.size(); ++index) {
    if (!finite_point(keypoints[index])) {
      return std::nullopt;
    }
    result.keypoints[index] = keypoints[index];
  }
  return result;
}

std::optional<std::string> validate_output_shape(
  const std::vector<std::size_t> & shape,
  const DetectorConfig & config)
{
  if (shape.size() != 3 || shape[0] != 1) {
    return "expected output shape [1, rows, columns], got " + shape_string(shape);
  }
  if (config.expected_output_rows != 0 && shape[1] != config.expected_output_rows) {
    return "expected output rows " + std::to_string(config.expected_output_rows) + ", got " +
           std::to_string(shape[1]);
  }
  if (shape[2] != config.expected_output_columns) {
    return "expected output columns " + std::to_string(config.expected_output_columns) + ", got " +
           std::to_string(shape[2]);
  }
  const auto minimum_columns = 8 + 1 + config.color_class_count + config.armor_class_count;
  if (shape[2] < minimum_columns) {
    return "output columns are too few for the configured keypoint/color/class contract: " +
           std::to_string(shape[2]) + " < " + std::to_string(minimum_columns);
  }
  return std::nullopt;
}

std::optional<std::string> validate_input_shape(
  const std::vector<std::size_t> & shape,
  const DetectorConfig & config)
{
  const std::vector<std::size_t> expected{
    1, 3, static_cast<std::size_t>(config.input_height),
    static_cast<std::size_t>(config.input_width)};
  if (shape != expected) {
    return "expected static NCHW input shape " + shape_string(expected) + ", got " +
           shape_string(shape);
  }
  return std::nullopt;
}

std::optional<LetterboxResult> make_letterbox(
  const cv::Mat & bgr_image,
  const DetectorConfig & config)
{
  if (bgr_image.empty() || bgr_image.type() != CV_8UC3 || config.input_width <= 0 ||
    config.input_height <= 0)
  {
    return std::nullopt;
  }

  const auto scale = std::min(
    static_cast<float>(config.input_width) / static_cast<float>(bgr_image.cols),
    static_cast<float>(config.input_height) / static_cast<float>(bgr_image.rows));
  if (!std::isfinite(scale) || scale <= 0.0F) {
    return std::nullopt;
  }

  const int resized_width = std::max(1, static_cast<int>(bgr_image.cols * scale));
  const int resized_height = std::max(1, static_cast<int>(bgr_image.rows * scale));
  const int available_width = config.input_width - resized_width;
  const int available_height = config.input_height - resized_height;
  const int pad_x = config.center_padding ? available_width / 2 : 0;
  const int pad_y = config.center_padding ? available_height / 2 : 0;
  if (pad_x < 0 || pad_y < 0 || pad_x + resized_width > config.input_width ||
    pad_y + resized_height > config.input_height)
  {
    return std::nullopt;
  }

  LetterboxResult result{};
  result.image = cv::Mat(
    config.input_height, config.input_width, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::Mat resized;
  cv::resize(bgr_image, resized, cv::Size(resized_width, resized_height), 0.0, 0.0, cv::INTER_LINEAR);
  resized.copyTo(result.image(cv::Rect(pad_x, pad_y, resized_width, resized_height)));
  result.scale = scale;
  result.pad_x = pad_x;
  result.pad_y = pad_y;
  return result;
}

std::optional<cv::Point2f> model_point_to_image(
  const cv::Point2f & model_point,
  const LetterboxResult & letterbox) noexcept
{
  if (!finite_point(model_point) || !std::isfinite(letterbox.scale) || letterbox.scale <= 0.0F) {
    return std::nullopt;
  }
  const cv::Point2f image_point{
    (model_point.x - static_cast<float>(letterbox.pad_x)) / letterbox.scale,
    (model_point.y - static_cast<float>(letterbox.pad_y)) / letterbox.scale};
  if (!finite_point(image_point)) {
    return std::nullopt;
  }
  return image_point;
}

struct OpenVinoYoloDetector::Impl
{
  explicit Impl(DetectorConfig detector_config) : config(std::move(detector_config)) {}

  DetectorConfig config;
  ModelInfo info;
#ifdef AUTO_AIM_HAS_OPENVINO
  ov::Core core;
  ov::CompiledModel compiled_model;
#endif
};

OpenVinoYoloDetector::OpenVinoYoloDetector(DetectorConfig config)
: impl_(std::make_unique<Impl>(std::move(config)))
{
  if (impl_->config.model_path.empty()) {
    throw std::invalid_argument("OpenVINO model_path must not be empty");
  }
  if (!std::filesystem::exists(impl_->config.model_path)) {
    throw std::runtime_error("OpenVINO model file does not exist: " + impl_->config.model_path);
  }
#ifndef AUTO_AIM_HAS_OPENVINO
  throw std::runtime_error(
    "OpenVINO support is not built; configure OpenVINO_DIR before building auto_aim_ros2");
#else
  try {
    const auto model = impl_->core.read_model(impl_->config.model_path);
    if (model->inputs().size() != 1) {
      throw std::runtime_error(
        "expected one model input, got " + std::to_string(model->inputs().size()));
    }
    if (model->outputs().size() != 1) {
      throw std::runtime_error(
        "expected one model output, got " + std::to_string(model->outputs().size()));
    }

    const auto input_shape = static_shape(model->input(), "model input");
    if (const auto error = validate_input_shape(input_shape, impl_->config); error.has_value()) {
      throw std::runtime_error(*error);
    }
    const auto output_shape = static_shape(model->output(), "model output");
    if (const auto error = validate_output_shape(output_shape, impl_->config); error.has_value()) {
      throw std::runtime_error(*error);
    }
    if (model->output().get_element_type() != ov::element::f32) {
      throw std::runtime_error(
        "expected FP32 model output, got " + model->output().get_element_type().to_string());
    }

    impl_->info.input_shape = input_shape;
    impl_->info.output_shape = output_shape;
    impl_->info.input_element_type = model->input().get_element_type().to_string();
    impl_->info.output_element_type = model->output().get_element_type().to_string();

    ov::preprocess::PrePostProcessor preprocessor(model);
    auto & input = preprocessor.input();
    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, static_cast<std::int64_t>(impl_->config.input_height),
                  static_cast<std::int64_t>(impl_->config.input_width), 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);
    input.model().set_layout("NCHW");
    input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      // OpenVINO's scale operation divides by the supplied value.
      .scale(255.0);

    impl_->compiled_model = impl_->core.compile_model(
      preprocessor.build(), impl_->config.device,
      ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
  } catch (const std::exception & error) {
    throw std::runtime_error(
      "OpenVINO model initialization failed for " + impl_->config.model_path + ": " +
      error.what());
  }
#endif
}

OpenVinoYoloDetector::~OpenVinoYoloDetector() = default;

OpenVinoYoloDetector::OpenVinoYoloDetector(OpenVinoYoloDetector &&) noexcept = default;

OpenVinoYoloDetector & OpenVinoYoloDetector::operator=(OpenVinoYoloDetector &&) noexcept = default;

std::vector<RawArmorDetection> OpenVinoYoloDetector::detect(const pipeline::ImageFrame & frame) const
{
  if (!frame.has_pixels()) {
    return {};
  }
  const auto letterbox = make_letterbox(frame.bgr_image, impl_->config);
  if (!letterbox.has_value()) {
    return {};
  }
#ifndef AUTO_AIM_HAS_OPENVINO
  return {};
#else
  auto request = impl_->compiled_model.create_infer_request();
  ov::Tensor input_tensor(
    ov::element::u8,
    {1, static_cast<std::size_t>(impl_->config.input_height),
     static_cast<std::size_t>(impl_->config.input_width), 3},
    letterbox->image.data);
  request.set_input_tensor(input_tensor);
  request.infer();

  const auto output_tensor = request.get_output_tensor();
  const auto output_shape = output_tensor.get_shape();
  if (const auto error = validate_output_shape(output_shape, impl_->config); error.has_value()) {
    throw std::runtime_error("OpenVINO output changed at inference time: " + *error);
  }
  if (output_tensor.get_element_type() != ov::element::f32) {
    throw std::runtime_error("OpenVINO inference output is not FP32");
  }

  const auto * values = output_tensor.data<const float>();
  const std::size_t rows = output_shape[1];
  const std::size_t columns = output_shape[2];
  const std::size_t color_offset = 9;
  const std::size_t class_offset = color_offset + impl_->config.color_class_count;
  std::vector<RawArmorDetection> candidates;
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  candidates.reserve(rows);
  boxes.reserve(rows);
  confidences.reserve(rows);

  for (std::size_t row = 0; row < rows; ++row) {
    const float * current = values + row * columns;
    const float confidence = sigmoid(current[8]);
    if (!std::isfinite(confidence) || confidence < impl_->config.objectness_threshold) {
      continue;
    }
    const int color_id = argmax(current + color_offset, impl_->config.color_class_count);
    const int class_id = argmax(current + class_offset, impl_->config.armor_class_count);
    if (color_id < 0 || class_id < 0) {
      continue;
    }

    std::vector<cv::Point2f> keypoints;
    keypoints.reserve(4);
    bool finite_coordinates = true;
    for (const int raw_index : impl_->config.keypoint_order) {
      if (raw_index < 0 || raw_index >= 4) {
        finite_coordinates = false;
        break;
      }
      const std::size_t raw_offset = static_cast<std::size_t>(raw_index) * 2;
      const auto image_point = model_point_to_image(
        {current[raw_offset], current[raw_offset + 1]}, *letterbox);
      if (!image_point.has_value()) {
        finite_coordinates = false;
        break;
      }
      keypoints.push_back(*image_point);
    }
    if (!finite_coordinates || keypoints.size() != 4) {
      continue;
    }

    float min_x = keypoints.front().x;
    float max_x = keypoints.front().x;
    float min_y = keypoints.front().y;
    float max_y = keypoints.front().y;
    for (const auto & point : keypoints) {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }
    const cv::Rect2f bbox(min_x, min_y, max_x - min_x, max_y - min_y);
    const auto detection = make_raw_armor_detection(
      class_id, color_id, confidence, bbox, keypoints,
      static_cast<int>(impl_->config.armor_class_count),
      static_cast<int>(impl_->config.color_class_count));
    if (!detection.has_value()) {
      continue;
    }
    candidates.push_back(*detection);
    boxes.push_back(to_nms_rect(bbox));
    confidences.push_back(confidence);
  }

  std::vector<int> kept;
  cv::dnn::NMSBoxes(
    boxes, confidences, impl_->config.objectness_threshold, impl_->config.nms_threshold, kept);
  std::vector<RawArmorDetection> result;
  result.reserve(kept.size());
  for (const int index : kept) {
    if (index >= 0 && static_cast<std::size_t>(index) < candidates.size()) {
      result.push_back(candidates[static_cast<std::size_t>(index)]);
    }
  }
  return result;
#endif
}

const ModelInfo & OpenVinoYoloDetector::model_info() const
{
  return impl_->info;
}

cv::Mat OpenVinoYoloDetector::annotate(
  const cv::Mat & bgr_image,
  const std::vector<RawArmorDetection> & detections)
{
  if (bgr_image.empty()) {
    return {};
  }
  cv::Mat result = bgr_image.clone();
  for (const auto & detection : detections) {
    cv::rectangle(result, detection.bbox, cv::Scalar(0, 255, 0), 2);
    for (std::size_t index = 0; index < detection.keypoints.size(); ++index) {
      cv::circle(result, detection.keypoints[index], 3, cv::Scalar(0, 0, 255), -1);
      if (index > 0) {
        cv::line(
          result, detection.keypoints[index - 1], detection.keypoints[index],
          cv::Scalar(255, 0, 0), 1);
      }
    }
    cv::line(
      result, detection.keypoints.back(), detection.keypoints.front(), cv::Scalar(255, 0, 0), 1);
    std::ostringstream label;
    label << "class=" << detection.class_id << " color=" << detection.color_id << " conf="
          << std::fixed << std::setprecision(2) << detection.confidence;
    cv::putText(
      result, label.str(), cv::Point(static_cast<int>(detection.bbox.x),
                                     std::max(0, static_cast<int>(detection.bbox.y) - 4)),
      cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
  }
  return result;
}

}  // namespace rm_auto_aim::detector
