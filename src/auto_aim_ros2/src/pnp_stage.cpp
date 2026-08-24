#include "auto_aim_ros2/pnp_stage.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace rm_auto_aim::pnp
{
namespace
{
constexpr double kFiniteEpsilon = 1e-9;
constexpr double kRotationTolerance = 1e-6;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finite_point(const cv::Point2f & point)
{
  return finite(point.x) && finite(point.y);
}

bool finite_point(const cv::Point3d & point)
{
  return finite(point.x) && finite(point.y) && finite(point.z);
}

bool finite_vector(const cv::Vec3d & vector)
{
  return finite(vector[0]) && finite(vector[1]) && finite(vector[2]);
}

cv::Vec3d vector_between(const cv::Point3d & from, const cv::Point3d & to)
{
  return {to.x - from.x, to.y - from.y, to.z - from.z};
}

bool finite_matrix(const cv::Matx33d & matrix)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      if (!finite(matrix(row, col))) {
        return false;
      }
    }
  }
  return true;
}

double signed_area(const std::array<cv::Point2f, 4> & points)
{
  double twice_area = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & a = points[index];
    const auto & b = points[(index + 1U) % points.size()];
    twice_area += static_cast<double>(a.x) * b.y - static_cast<double>(a.y) * b.x;
  }
  return twice_area * 0.5;
}

bool is_ordered_convex_quad(const std::array<cv::Point2f, 4> & points)
{
  // Detector contract is top-left, top-right, bottom-right, bottom-left in
  // image coordinates (x right/y down), which has positive winding.  This
  // rejects crossed, duplicate, and mirrored point order; a cyclic shift
  // remains model-semantic and therefore is intentionally not guessed here.
  if (signed_area(points) <= kFiniteEpsilon) {
    return false;
  }
  double last_cross = 0.0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & a = points[index];
    const auto & b = points[(index + 1U) % points.size()];
    const auto & c = points[(index + 2U) % points.size()];
    const cv::Point2f ab = b - a;
    const cv::Point2f bc = c - b;
    const double cross = static_cast<double>(ab.x) * bc.y -
      static_cast<double>(ab.y) * bc.x;
    if (!finite(cross) || std::abs(cross) <= kFiniteEpsilon) {
      return false;
    }
    if (index > 0 && cross * last_cross <= 0.0) {
      return false;
    }
    last_cross = cross;
  }
  return true;
}

cv::Matx33d matx_from_values(const std::vector<double> & values)
{
  return cv::Matx33d(
    values[0], values[1], values[2],
    values[3], values[4], values[5],
    values[6], values[7], values[8]);
}

std::string yaml_context(const std::string & path, const std::string & field)
{
  return path + ": missing or invalid required field '" + field + "'";
}

YAML::Node required_node(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(yaml_context(path, key));
  }
  return node;
}

std::string required_string(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const auto value = required_node(parent, key, path).as<std::string>();
    if (value.empty()) {
      throw std::runtime_error(yaml_context(path, key));
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(yaml_context(path, key));
  }
}

int required_int(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    return required_node(parent, key, path).as<int>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(yaml_context(path, key));
  }
}

double required_double(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const double value = required_node(parent, key, path).as<double>();
    if (!finite(value)) {
      throw std::runtime_error(yaml_context(path, key));
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(yaml_context(path, key));
  }
}

bool required_bool(const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    return required_node(parent, key, path).as<bool>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(yaml_context(path, key));
  }
}

std::vector<double> required_double_sequence(
  const YAML::Node & parent,
  const std::string & key,
  std::size_t expected_count,
  const std::string & path)
{
  const auto node = required_node(parent, key, path);
  if (!node.IsSequence() || node.size() != expected_count) {
    throw std::runtime_error(
      path + ": field '" + key + "' must contain exactly " + std::to_string(expected_count) +
      " finite numbers");
  }
  std::vector<double> result;
  result.reserve(expected_count);
  try {
    for (const auto & value : node) {
      const double number = value.as<double>();
      if (!finite(number)) {
        throw std::runtime_error(path + ": field '" + key + "' contains non-finite value");
      }
      result.push_back(number);
    }
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must contain finite numbers");
  }
  return result;
}

std::vector<double> required_distortion_sequence(
  const YAML::Node & parent,
  const std::string & key,
  const std::string & path)
{
  const auto node = required_node(parent, key, path);
  if (!node.IsSequence() ||
    !(node.size() == 4 || node.size() == 5 || node.size() == 8 || node.size() == 12 || node.size() == 14))
  {
    throw std::runtime_error(
      path + ": field '" + key + "' must contain 4, 5, 8, 12, or 14 finite numbers");
  }
  std::vector<double> result;
  result.reserve(node.size());
  try {
    for (const auto & value : node) {
      const double number = value.as<double>();
      if (!finite(number)) {
        throw std::runtime_error(path + ": field '" + key + "' contains non-finite value");
      }
      result.push_back(number);
    }
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must contain finite numbers");
  }
  return result;
}

std::array<cv::Point3d, 4> required_object_points(
  const YAML::Node & parent,
  const std::string & path)
{
  const auto node = required_node(parent, "object_points_m", path);
  if (!node.IsSequence() || node.size() != 4) {
    throw std::runtime_error(path + ": object_points_m must contain exactly four [x,y,z] points");
  }
  std::array<cv::Point3d, 4> result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    const auto point = node[index];
    if (!point.IsSequence() || point.size() != 3) {
      throw std::runtime_error(path + ": every object_points_m entry must contain [x,y,z]");
    }
    try {
      result[index] = {point[0].as<double>(), point[1].as<double>(), point[2].as<double>()};
    } catch (const YAML::Exception &) {
      throw std::runtime_error(path + ": object_points_m must contain finite numbers");
    }
    if (!finite_point(result[index])) {
      throw std::runtime_error(path + ": object_points_m contains non-finite value");
    }
  }
  return result;
}

ArmorSize parse_armor_size(const std::string & value, const std::string & context)
{
  if (value == "small") {
    return ArmorSize::Small;
  }
  if (value == "large") {
    return ArmorSize::Large;
  }
  throw std::runtime_error(context + ": armor size must be 'small' or 'large'");
}

ArmorGeometry parse_geometry(
  const YAML::Node & geometry_node,
  const std::string & name,
  ArmorSize size,
  bool test_only)
{
  const std::string path = "armor_geometry." + name;
  const auto node = required_node(geometry_node, name, "armor_geometry");
  if (!node.IsMap()) {
    throw std::runtime_error(path + " must be a map");
  }
  ArmorGeometry result{};
  result.size = size;
  result.width_m = required_double(node, "width_m", path);
  result.height_m = required_double(node, "height_m", path);
  result.object_points_m = required_object_points(node, path);
  result.source = required_string(node, "source", path);
  result.version = required_string(node, "version", path);
  result.object_frame = required_string(node, "object_frame", path);
  result.test_only = test_only;
  if (const auto error = result.validate(); error.has_value()) {
    throw std::runtime_error(path + ": " + *error);
  }
  return result;
}

PoseObservation invalid_observation(
  const detector::RawArmorDetection & detection,
  PoseFailure failure)
{
  PoseObservation result{};
  result.raw_detection = detection;
  result.failure = failure;
  return result;
}

bool keypoints_within_image(
  const std::array<cv::Point2f, 4> & points,
  const CameraCalibration & camera)
{
  for (const auto & point : points) {
    if (!finite_point(point) || point.x < 0.0F || point.y < 0.0F ||
      point.x >= static_cast<float>(camera.image_width) ||
      point.y >= static_cast<float>(camera.image_height))
    {
      return false;
    }
  }
  return true;
}

bool bbox_within_image(const cv::Rect2f & bbox, const CameraCalibration & camera)
{
  return finite(bbox.x) && finite(bbox.y) && finite(bbox.width) && finite(bbox.height) &&
         bbox.x >= 0.0F && bbox.y >= 0.0F && bbox.width > 0.0F && bbox.height > 0.0F &&
         bbox.x + bbox.width <= static_cast<float>(camera.image_width) &&
         bbox.y + bbox.height <= static_cast<float>(camera.image_height);
}

bool valid_raw_detection(const detector::RawArmorDetection & detection)
{
  if (detection.class_id < 0 || detection.color_id < -1 ||
    !finite(detection.confidence) || detection.confidence < 0.0F || detection.confidence > 1.0F ||
    !finite(detection.bbox.x) || !finite(detection.bbox.y) ||
    !finite(detection.bbox.width) || !finite(detection.bbox.height) ||
    detection.bbox.width <= 0.0F || detection.bbox.height <= 0.0F)
  {
    return false;
  }
  return std::all_of(
    detection.keypoints.begin(), detection.keypoints.end(),
    [](const cv::Point2f & point) { return finite_point(point); });
}

double reprojection_rms(
  const std::array<cv::Point2f, 4> & observed,
  const std::vector<cv::Point2d> & reprojected)
{
  if (reprojected.size() != observed.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double squared_sum = 0.0;
  for (std::size_t index = 0; index < observed.size(); ++index) {
    const double dx = static_cast<double>(observed[index].x) - reprojected[index].x;
    const double dy = static_cast<double>(observed[index].y) - reprojected[index].y;
    squared_sum += dx * dx + dy * dy;
  }
  return std::sqrt(squared_sum / static_cast<double>(observed.size()));
}

bool all_object_points_in_front(
  const std::array<cv::Point3d, 4> & object_points,
  const cv::Matx33d & rotation_camera_from_armor,
  const cv::Vec3d & translation_camera_m)
{
  for (const auto & point : object_points) {
    const cv::Vec3d point_in_armor{point.x, point.y, point.z};
    const cv::Vec3d point_in_camera =
      rotation_camera_from_armor * point_in_armor + translation_camera_m;
    if (!finite_vector(point_in_camera) || point_in_camera[2] <= kFiniteEpsilon) {
      return false;
    }
  }
  return true;
}
}  // namespace

const char * armor_size_name(ArmorSize size) noexcept
{
  return size == ArmorSize::Small ? "small" : "large";
}

std::optional<std::string> CameraCalibration::validate() const
{
  if (image_width <= 0 || image_height <= 0) {
    return "image_width and image_height must be positive";
  }
  if (!finite_matrix(camera_matrix) || camera_matrix(0, 0) <= 0.0 || camera_matrix(1, 1) <= 0.0 ||
    std::abs(camera_matrix(2, 0)) > kFiniteEpsilon ||
    std::abs(camera_matrix(2, 1)) > kFiniteEpsilon ||
    std::abs(camera_matrix(2, 2) - 1.0) > kFiniteEpsilon)
  {
    return "camera_matrix must be a finite standard pinhole K with positive fx/fy and bottom row [0,0,1]";
  }
  if (distortion_coefficients.empty() ||
    !std::all_of(distortion_coefficients.begin(), distortion_coefficients.end(), finite))
  {
    return "distortion_coefficients must be a non-empty finite OpenCV-supported vector";
  }
  if (source.empty() || version.empty()) {
    return "source and version are required";
  }
  if (coordinate_frame != kCameraOpticalFrame) {
    return std::string("coordinate_frame must be '") + kCameraOpticalFrame + "'";
  }
  return std::nullopt;
}

std::optional<std::string> ArmorGeometry::validate() const
{
  if (!finite(width_m) || !finite(height_m) || width_m <= 0.0 || height_m <= 0.0) {
    return "width_m and height_m must be positive finite metres";
  }
  if (source.empty() || version.empty() || object_frame.empty()) {
    return "source, version, and object_frame are required";
  }
  if (!std::all_of(
      object_points_m.begin(), object_points_m.end(),
      [](const cv::Point3d & point) { return finite_point(point); }))
  {
    return "object_points_m must be finite";
  }

  const cv::Vec3d horizontal_top = vector_between(object_points_m[0], object_points_m[1]);
  const cv::Vec3d horizontal_bottom = vector_between(object_points_m[3], object_points_m[2]);
  const cv::Vec3d vertical_left = vector_between(object_points_m[0], object_points_m[3]);
  const cv::Vec3d vertical_right = vector_between(object_points_m[1], object_points_m[2]);
  const double observed_width = (cv::norm(horizontal_top) + cv::norm(horizontal_bottom)) * 0.5;
  const double observed_height = (cv::norm(vertical_left) + cv::norm(vertical_right)) * 0.5;
  if (observed_width <= kFiniteEpsilon || observed_height <= kFiniteEpsilon) {
    return "object_points_m must describe a non-degenerate quadrilateral";
  }
  const double width_tolerance = std::max(1e-9, width_m * 0.01);
  const double height_tolerance = std::max(1e-9, height_m * 0.01);
  if (std::abs(observed_width - width_m) > width_tolerance ||
    std::abs(observed_height - height_m) > height_tolerance)
  {
    return "width_m/height_m do not match the explicit object_points_m geometry";
  }
  const double horizontal_top_norm = cv::norm(horizontal_top);
  const double horizontal_bottom_norm = cv::norm(horizontal_bottom);
  const double vertical_left_norm = cv::norm(vertical_left);
  const double vertical_right_norm = cv::norm(vertical_right);
  const cv::Vec3d normal = horizontal_top.cross(vertical_left);
  if (cv::norm(normal) <= kFiniteEpsilon ||
    std::abs(normal.dot(vector_between(object_points_m[0], object_points_m[2]))) >
    std::max(1e-9, cv::norm(normal) * 1e-6))
  {
    return "object_points_m must be planar";
  }
  if (horizontal_top.dot(horizontal_bottom) <= 0.0 ||
    vertical_left.dot(vertical_right) <= 0.0 ||
    cv::norm(horizontal_top.cross(horizontal_bottom)) >
      horizontal_top_norm * horizontal_bottom_norm * 1e-6 ||
    cv::norm(vertical_left.cross(vertical_right)) >
      vertical_left_norm * vertical_right_norm * 1e-6 ||
    std::abs(horizontal_top.dot(vertical_left)) >
      horizontal_top_norm * vertical_left_norm * 1e-6)
  {
    return "object_points_m must be an ordered rectangular top-left/top-right/bottom-right/bottom-left geometry";
  }
  return std::nullopt;
}

std::optional<std::string> CameraToGimbalExtrinsic::validate() const
{
  if (!configured) {
    return std::nullopt;
  }
  if (source.empty() || version.empty()) {
    return "configured extrinsic requires source and version";
  }
  if (source_frame != kCameraOpticalFrame || target_frame != kGimbalFrame) {
    return std::string("configured extrinsic must map '") + kCameraOpticalFrame + "' to '" +
           kGimbalFrame + "'";
  }
  if (!finite_matrix(rotation_gimbal_from_camera) || !finite_vector(translation_gimbal_from_camera_m)) {
    return "extrinsic rotation and translation must be finite";
  }
  const cv::Matx33d should_be_identity =
    rotation_gimbal_from_camera * rotation_gimbal_from_camera.t();
  const double orthogonality_error = cv::norm(should_be_identity - cv::Matx33d::eye());
  const double determinant = cv::determinant(rotation_gimbal_from_camera);
  if (orthogonality_error > kRotationTolerance || std::abs(determinant - 1.0) > kRotationTolerance) {
    return "rotation_gimbal_from_camera must be a proper orthonormal rotation";
  }
  return std::nullopt;
}

const ArmorGeometry * PnpConfiguration::geometry_for(
  const detector::RawArmorDetection & detection) const noexcept
{
  if (detection.armor_type == detector::RawArmorDetection::ArmorTypeHint::Small) {
    return &small_armor;
  }
  if (detection.armor_type == detector::RawArmorDetection::ArmorTypeHint::Large) {
    return &large_armor;
  }
  const auto iterator = class_to_armor_size.find(detection.class_id);
  if (iterator == class_to_armor_size.end()) {
    return nullptr;
  }
  return iterator->second == ArmorSize::Small ? &small_armor : &large_armor;
}

std::optional<std::string> PnpConfiguration::validate() const
{
  if (schema_version != 1) {
    return "unsupported schema_version; expected 1";
  }
  if (!test_only && !camera_to_gimbal.configured) {
    return "production PnP configuration requires a verified camera_to_gimbal extrinsic";
  }
  if (const auto error = camera.validate(); error.has_value()) {
    return "camera: " + *error;
  }
  if (const auto error = small_armor.validate(); error.has_value()) {
    return "small_armor: " + *error;
  }
  if (const auto error = large_armor.validate(); error.has_value()) {
    return "large_armor: " + *error;
  }
  if (!finite(max_reprojection_error_px) || max_reprojection_error_px <= 0.0) {
    return "max_reprojection_error_px must be a positive finite pixel value";
  }
  if (solvepnp_flag != cv::SOLVEPNP_ITERATIVE) {
    return "only explicit ITERATIVE solvePnP is supported by schema version 1";
  }
  for (const auto & [class_id, size] : class_to_armor_size) {
    if (class_id < 0 || (size != ArmorSize::Small && size != ArmorSize::Large)) {
      return "class_to_armor_type contains invalid entry";
    }
  }
  if (const auto error = camera_to_gimbal.validate(); error.has_value()) {
    return "camera_to_gimbal: " + *error;
  }
  return std::nullopt;
}

PnpConfiguration load_pnp_configuration(const std::string & yaml_path, ConfigLoadOptions options)
{
  if (yaml_path.empty() || !std::filesystem::is_regular_file(yaml_path)) {
    throw std::runtime_error("PnP configuration file does not exist: " + yaml_path);
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse PnP configuration " + yaml_path + ": " + error.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("PnP configuration root must be a map: " + yaml_path);
  }

  PnpConfiguration result{};
  result.schema_version = required_int(root, "schema_version", "root");
  const auto profile = required_string(root, "profile", "root");
  if (profile == "test_only") {
    result.test_only = true;
  } else if (profile == "production") {
    result.test_only = false;
  } else {
    throw std::runtime_error("root.profile must be 'test_only' or 'production'");
  }
  if (result.test_only && !options.allow_test_only) {
    throw std::runtime_error(
      "refusing test_only PnP configuration without explicit allow_test_only=true: " + yaml_path);
  }

  const auto camera_node = required_node(root, "camera", "root");
  if (!camera_node.IsMap()) {
    throw std::runtime_error("camera must be a map");
  }
  result.camera.image_width = required_int(camera_node, "image_width", "camera");
  result.camera.image_height = required_int(camera_node, "image_height", "camera");
  result.camera.camera_matrix = matx_from_values(
    required_double_sequence(camera_node, "camera_matrix", 9, "camera"));
  result.camera.distortion_coefficients =
    required_distortion_sequence(camera_node, "distortion_coefficients", "camera");
  result.camera.source = required_string(camera_node, "source", "camera");
  result.camera.version = required_string(camera_node, "version", "camera");
  result.camera.coordinate_frame = required_string(camera_node, "coordinate_frame", "camera");
  result.camera.test_only = result.test_only;

  const auto geometry_node = required_node(root, "armor_geometry", "root");
  if (!geometry_node.IsMap()) {
    throw std::runtime_error("armor_geometry must be a map");
  }
  const auto corner_order = required_node(geometry_node, "corner_order", "armor_geometry");
  const std::array<std::string, 4> expected_order{{
    "top_left", "top_right", "bottom_right", "bottom_left"}};
  if (!corner_order.IsSequence() || corner_order.size() != expected_order.size()) {
    throw std::runtime_error("armor_geometry.corner_order must list four explicit ordered corners");
  }
  for (std::size_t index = 0; index < expected_order.size(); ++index) {
    try {
      if (corner_order[index].as<std::string>() != expected_order[index]) {
        throw std::runtime_error(
          "armor_geometry.corner_order must be top_left, top_right, bottom_right, bottom_left");
      }
    } catch (const YAML::Exception &) {
      throw std::runtime_error("armor_geometry.corner_order must contain strings");
    }
  }
  result.small_armor = parse_geometry(geometry_node, "small", ArmorSize::Small, result.test_only);
  result.large_armor = parse_geometry(geometry_node, "large", ArmorSize::Large, result.test_only);

  const auto mapping_node = root["class_to_armor_type"];
  if (mapping_node) {
    if (!mapping_node.IsMap()) {
      throw std::runtime_error("class_to_armor_type must be a map when present");
    }
    for (const auto & item : mapping_node) {
      try {
        const int class_id = item.first.as<int>();
        const auto [ignored, inserted] = result.class_to_armor_size.emplace(
          class_id, parse_armor_size(item.second.as<std::string>(), "class_to_armor_type"));
        if (!inserted) {
          throw std::runtime_error("class_to_armor_type contains a duplicate class id");
        }
      } catch (const YAML::Exception &) {
        throw std::runtime_error("class_to_armor_type must map integer class ids to small/large");
      }
    }
  }

  const auto pnp_node = required_node(root, "pnp", "root");
  if (!pnp_node.IsMap()) {
    throw std::runtime_error("pnp must be a map");
  }
  const auto method = required_string(pnp_node, "method", "pnp");
  if (method != "ITERATIVE") {
    throw std::runtime_error("pnp.method must be the explicit supported value 'ITERATIVE'");
  }
  result.solvepnp_flag = cv::SOLVEPNP_ITERATIVE;
  result.max_reprojection_error_px = required_double(pnp_node, "max_reprojection_error_px", "pnp");

  const auto extrinsic_node = required_node(root, "camera_to_gimbal", "root");
  if (!extrinsic_node.IsMap()) {
    throw std::runtime_error("camera_to_gimbal must be a map");
  }
  result.camera_to_gimbal.configured =
    required_bool(extrinsic_node, "configured", "camera_to_gimbal");
  result.camera_to_gimbal.test_only = result.test_only;
  if (result.camera_to_gimbal.configured) {
    result.camera_to_gimbal.rotation_gimbal_from_camera = matx_from_values(
      required_double_sequence(
        extrinsic_node, "rotation_gimbal_from_camera", 9, "camera_to_gimbal"));
    const auto translation = required_double_sequence(
      extrinsic_node, "translation_gimbal_from_camera_m", 3, "camera_to_gimbal");
    result.camera_to_gimbal.translation_gimbal_from_camera_m = {
      translation[0], translation[1], translation[2]};
    result.camera_to_gimbal.source =
      required_string(extrinsic_node, "source", "camera_to_gimbal");
    result.camera_to_gimbal.version =
      required_string(extrinsic_node, "version", "camera_to_gimbal");
    result.camera_to_gimbal.source_frame =
      required_string(extrinsic_node, "source_frame", "camera_to_gimbal");
    result.camera_to_gimbal.target_frame =
      required_string(extrinsic_node, "target_frame", "camera_to_gimbal");
  }

  if (const auto error = result.validate(); error.has_value()) {
    throw std::runtime_error("invalid PnP configuration " + yaml_path + ": " + *error);
  }
  return result;
}

const char * pose_failure_name(PoseFailure failure) noexcept
{
  switch (failure) {
    case PoseFailure::None:
      return "none";
    case PoseFailure::InvalidRawDetection:
      return "invalid_raw_detection";
    case PoseFailure::GeometryNotConfigured:
      return "geometry_not_configured";
    case PoseFailure::GeometrySemanticConflict:
      return "geometry_semantic_conflict";
    case PoseFailure::InvalidConfiguration:
      return "invalid_configuration";
    case PoseFailure::ImageDimensionsMismatch:
      return "image_dimensions_mismatch";
    case PoseFailure::KeypointOrderRejected:
      return "keypoint_order_rejected";
    case PoseFailure::SolvePnpFailed:
      return "solvepnp_failed";
    case PoseFailure::NonPositiveCameraDepth:
      return "nonpositive_camera_depth";
    case PoseFailure::ReprojectionErrorTooLarge:
      return "reprojection_error_too_large";
    case PoseFailure::RelativeAngleUndefined:
      return "relative_angle_undefined";
  }
  return "unknown";
}

PnpStage::PnpStage(PnpConfiguration config) : config_(std::move(config))
{
  if (const auto error = config_.validate(); error.has_value()) {
    throw std::invalid_argument("invalid PnP stage configuration: " + *error);
  }
}

PoseObservation PnpStage::solve(const detector::RawArmorDetection & detection) const noexcept
{
  return solve(detection, config_.camera.image_width, config_.camera.image_height);
}

PoseObservation PnpStage::solve(
  const detector::RawArmorDetection & detection,
  int image_width,
  int image_height) const noexcept
{
  try {
    if (image_width != config_.camera.image_width || image_height != config_.camera.image_height) {
      return invalid_observation(detection, PoseFailure::ImageDimensionsMismatch);
    }
    if (!valid_raw_detection(detection)) {
      return invalid_observation(detection, PoseFailure::InvalidRawDetection);
    }
    const auto * geometry = config_.geometry_for(detection);
    if (geometry == nullptr) {
      return invalid_observation(detection, PoseFailure::GeometryNotConfigured);
    }
    // A reviewed model profile may carry an armor-size hint while the PnP
    // profile may also map class_id to a physical geometry.  If both are
    // present they must agree; silently preferring one would allow a stale or
    // mismatched model/calibration pair to solve with the wrong dimensions.
    const auto class_mapping = config_.class_to_armor_size.find(detection.class_id);
    if (class_mapping != config_.class_to_armor_size.end() &&
      detection.armor_type != detector::RawArmorDetection::ArmorTypeHint::Unknown)
    {
      const auto hinted_size = detection.armor_type ==
        detector::RawArmorDetection::ArmorTypeHint::Small ? ArmorSize::Small : ArmorSize::Large;
      if (class_mapping->second != hinted_size) {
        return invalid_observation(detection, PoseFailure::GeometrySemanticConflict);
      }
    }
    if (!bbox_within_image(detection.bbox, config_.camera) ||
      !is_ordered_convex_quad(detection.keypoints) ||
      !keypoints_within_image(detection.keypoints, config_.camera))
    {
      return invalid_observation(detection, PoseFailure::KeypointOrderRejected);
    }

    const std::vector<cv::Point3d> object_points(
      geometry->object_points_m.begin(), geometry->object_points_m.end());
    const std::vector<cv::Point2f> image_points(
      detection.keypoints.begin(), detection.keypoints.end());
    const cv::Mat distortion(config_.camera.distortion_coefficients);
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    const bool solved = cv::solvePnP(
      object_points, image_points, config_.camera.camera_matrix, distortion, rvec, tvec, false,
      config_.solvepnp_flag);
    if (!solved || !finite_vector(rvec) || !finite_vector(tvec)) {
      return invalid_observation(detection, PoseFailure::SolvePnpFailed);
    }
    cv::Mat rotation_mat;
    cv::Rodrigues(rvec, rotation_mat);
    cv::Mat rotation_double;
    rotation_mat.convertTo(rotation_double, CV_64F);
    cv::Matx33d rotation_camera_from_armor{};
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        rotation_camera_from_armor(row, col) = rotation_double.at<double>(row, col);
      }
    }
    if (!finite_matrix(rotation_camera_from_armor)) {
      return invalid_observation(detection, PoseFailure::SolvePnpFailed);
    }
    if (!all_object_points_in_front(
        geometry->object_points_m, rotation_camera_from_armor, tvec))
    {
      return invalid_observation(detection, PoseFailure::NonPositiveCameraDepth);
    }

    std::vector<cv::Point2d> reprojected;
    cv::projectPoints(
      object_points, rvec, tvec, config_.camera.camera_matrix, distortion, reprojected);
    const double error_px = reprojection_rms(detection.keypoints, reprojected);
    if (!finite(error_px)) {
      return invalid_observation(detection, PoseFailure::SolvePnpFailed);
    }
    if (error_px > config_.max_reprojection_error_px) {
      auto result = invalid_observation(detection, PoseFailure::ReprojectionErrorTooLarge);
      result.reprojection_error_px = error_px;
      return result;
    }

    PoseObservation result{};
    result.raw_detection = detection;
    result.armor_size = geometry->size;
    result.valid = true;
    result.failure = PoseFailure::None;
    result.translation_in_camera_m = tvec;
    result.rotation_camera_from_armor = rotation_camera_from_armor;
    result.reprojection_error_px = error_px;

    if (config_.camera_to_gimbal.configured) {
      const auto & extrinsic = config_.camera_to_gimbal;
      const cv::Vec3d translation_gimbal =
        extrinsic.rotation_gimbal_from_camera * tvec + extrinsic.translation_gimbal_from_camera_m;
      if (!finite_vector(translation_gimbal)) {
        return invalid_observation(detection, PoseFailure::InvalidConfiguration);
      }
      result.translation_in_gimbal_m = translation_gimbal;
      result.rotation_gimbal_from_armor =
        extrinsic.rotation_gimbal_from_camera * rotation_camera_from_armor;
      result.relative_angles_in_gimbal = relative_angles_from_gimbal_translation(translation_gimbal);
    }
    return result;
  } catch (const cv::Exception &) {
    return invalid_observation(detection, PoseFailure::SolvePnpFailed);
  } catch (const std::exception &) {
    return invalid_observation(detection, PoseFailure::SolvePnpFailed);
  }
}

const PnpConfiguration & PnpStage::config() const noexcept
{
  return config_;
}

std::optional<RelativeAngles> PnpStage::relative_angles_from_gimbal_translation(
  const cv::Vec3d & translation_gimbal_m) noexcept
{
  if (!finite_vector(translation_gimbal_m)) {
    return std::nullopt;
  }
  const double horizontal = std::hypot(translation_gimbal_m[0], translation_gimbal_m[1]);
  const double distance = std::hypot(horizontal, translation_gimbal_m[2]);
  if (!finite(horizontal) || !finite(distance) || distance <= kFiniteEpsilon) {
    return std::nullopt;
  }
  return RelativeAngles{
    std::atan2(translation_gimbal_m[1], translation_gimbal_m[0]),
    std::atan2(translation_gimbal_m[2], horizontal)};
}

cv::Mat PnpStage::annotate(
  const cv::Mat & bgr_image,
  const std::vector<PoseObservation> & observations)
{
  if (bgr_image.empty()) {
    return {};
  }
  cv::Mat result = bgr_image.clone();
  for (const auto & observation : observations) {
    const auto & box = observation.raw_detection.bbox;
    const auto color = observation.valid ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(result, box, color, 1);
    std::array<cv::Point, 4> corners{};
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const auto & keypoint = observation.raw_detection.keypoints[index];
      corners[index] = {
        static_cast<int>(std::lround(keypoint.x)),
        static_cast<int>(std::lround(keypoint.y))};
    }
    for (std::size_t index = 0; index < corners.size(); ++index) {
      const auto next = (index + 1U) % corners.size();
      cv::line(result, corners[index], corners[next], color, 1, cv::LINE_AA);
      cv::circle(result, corners[index], 3, color, cv::FILLED, cv::LINE_AA);
      cv::putText(
        result, std::to_string(index), corners[index] + cv::Point(3, -3),
        cv::FONT_HERSHEY_SIMPLEX, 0.33, color, 1, cv::LINE_AA);
    }
    std::ostringstream label;
    if (observation.valid) {
      label << "pnp " << armor_size_name(observation.armor_size) << " xyz[m]=" << std::fixed <<
        std::setprecision(2) << observation.translation_in_camera_m[0] << ',' <<
        observation.translation_in_camera_m[1] << ',' << observation.translation_in_camera_m[2] <<
        " err=" << observation.reprojection_error_px << "px";
      if (observation.relative_angles_in_gimbal.has_value()) {
        label << " rel[rad]=" << observation.relative_angles_in_gimbal->relative_yaw_rad << ',' <<
          observation.relative_angles_in_gimbal->relative_pitch_rad;
      }
    } else {
      label << "pnp invalid: " << pose_failure_name(observation.failure);
    }
    cv::putText(
      result, label.str(), cv::Point(static_cast<int>(box.x), std::max(12, static_cast<int>(box.y) - 7)),
      cv::FONT_HERSHEY_SIMPLEX, 0.38, color, 1, cv::LINE_AA);
  }
  return result;
}

}  // namespace rm_auto_aim::pnp
