#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <opencv2/dnn/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#ifdef AUTO_AIM_HAS_OPENVINO
#include <openvino/openvino.hpp>
#endif

#ifdef AUTO_AIM_HAS_OPENSSL
#include <openssl/evp.h>
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

bool same_model_path(const std::string & left, const std::string & right)
{
  if (left.empty() || right.empty()) {
    return false;
  }
  std::error_code left_error;
  std::error_code right_error;
  const auto left_path = std::filesystem::weakly_canonical(
    std::filesystem::absolute(left), left_error);
  const auto right_path = std::filesystem::weakly_canonical(
    std::filesystem::absolute(right), right_error);
  if (!left_error && !right_error) {
    return left_path == right_path;
  }
  return std::filesystem::absolute(left).lexically_normal() ==
    std::filesystem::absolute(right).lexically_normal();
}

bool valid_sha256_digest(const std::string & value)
{
  return value.size() == 64 && std::all_of(
    value.begin(), value.end(), [](const unsigned char character) {
      return std::isxdigit(character) != 0;
    });
}

std::optional<std::string> validate_artifact_sha256(
  const std::string & artifact_path,
  const std::string & expected_digest,
  const std::string & artifact_label)
{
  if (artifact_path.empty()) {
    return artifact_label + " path must not be empty for SHA-256 verification";
  }
  if (!valid_sha256_digest(expected_digest)) {
    return "reviewed " + artifact_label + " SHA-256 must contain exactly 64 hexadecimal characters";
  }
#ifndef AUTO_AIM_HAS_OPENSSL
  return "SHA-256 verification is unavailable in this build";
#else
  std::ifstream artifact(artifact_path, std::ios::binary);
  if (!artifact) {
    return "cannot read " + artifact_label + " for SHA-256 verification";
  }
  std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)> context(
    EVP_MD_CTX_new(), EVP_MD_CTX_free);
  if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1) {
    return "SHA-256 verifier initialization failed";
  }
  std::array<char, 8192> buffer{};
  while (artifact) {
    artifact.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto read_count = artifact.gcount();
    if (read_count > 0 && EVP_DigestUpdate(
        context.get(), buffer.data(), static_cast<std::size_t>(read_count)) != 1)
    {
      return "SHA-256 verifier update failed";
    }
  }
  if (!artifact.eof()) {
    return "cannot read " + artifact_label + " for SHA-256 verification";
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0;
  if (EVP_DigestFinal_ex(context.get(), digest.data(), &digest_size) != 1 || digest_size != 32U) {
    return "SHA-256 verifier finalization failed";
  }
  std::ostringstream actual;
  actual << std::hex << std::setfill('0');
  for (unsigned int index = 0; index < digest_size; ++index) {
    actual << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  std::string expected = expected_digest;
  std::transform(expected.begin(), expected.end(), expected.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  if (actual.str() != expected) {
    return artifact_label + " SHA-256 does not match the reviewed profile declaration";
  }
  return std::nullopt;
#endif
}

std::optional<std::string> validate_model_manifest_sha256(const DetectorConfig & config)
{
  if (!config.require_model_hash_match) {
    return std::nullopt;
  }
  if (const auto error = validate_artifact_sha256(
      config.model_path, config.reviewed_model_sha256, "model XML artifact");
    error.has_value())
  {
    return error;
  }
  if (const auto error = validate_artifact_sha256(
      config.model_bin_path, config.reviewed_model_bin_sha256, "model BIN artifact");
    error.has_value())
  {
    return error;
  }
  return std::nullopt;
}

int argmax(const float * values, std::size_t count)
{
  if (values == nullptr || count == 0) {
    return -1;
  }
  std::size_t best = 0;
  if (!std::isfinite(values[best])) {
    return -1;
  }
  for (std::size_t index = 1; index < count; ++index) {
    if (!std::isfinite(values[index])) {
      return -1;
    }
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

std::string canonical_layout_string(const std::string & value)
{
  std::string result;
  result.reserve(value.size());
  for (const unsigned char character : value) {
    if (std::isspace(character) != 0 || character == '[' || character == ']' ||
      character == ',')
    {
      continue;
    }
    result.push_back(static_cast<char>(std::toupper(character)));
  }
  return result;
}

std::string profile_context(const std::string & path, const std::string & field)
{
  return path + ": missing or invalid required field '" + field + "'";
}

YAML::Node required_profile_node(
  const YAML::Node & parent, const std::string & key, const std::string & path)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    throw std::runtime_error(profile_context(path, key));
  }
  return node;
}

std::string required_profile_string(
  const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const auto value = required_profile_node(parent, key, path).as<std::string>();
    if (value.empty()) {
      throw std::runtime_error(profile_context(path, key));
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(profile_context(path, key));
  }
}

std::optional<std::string> optional_profile_string(
  const YAML::Node & parent, const std::string & key, const std::string & path)
{
  const auto node = parent[key];
  if (!node || node.IsNull()) {
    return std::nullopt;
  }
  try {
    return node.as<std::string>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(profile_context(path, key));
  }
}

std::int64_t required_profile_integer(
  const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    return required_profile_node(parent, key, path).as<std::int64_t>();
  } catch (const YAML::Exception &) {
    throw std::runtime_error(profile_context(path, key));
  }
}

double required_profile_double(
  const YAML::Node & parent, const std::string & key, const std::string & path)
{
  try {
    const auto value = required_profile_node(parent, key, path).as<double>();
    if (!std::isfinite(value)) {
      throw std::runtime_error(profile_context(path, key));
    }
    return value;
  } catch (const YAML::Exception &) {
    throw std::runtime_error(profile_context(path, key));
  }
}

std::vector<std::int64_t> required_profile_integer_sequence(
  const YAML::Node & parent,
  const std::string & key,
  std::size_t expected_count,
  const std::string & path)
{
  const auto node = required_profile_node(parent, key, path);
  if (!node.IsSequence() || node.size() != expected_count) {
    throw std::runtime_error(
      path + ": field '" + key + "' must contain exactly " +
      std::to_string(expected_count) + " integers");
  }
  std::vector<std::int64_t> values;
  values.reserve(expected_count);
  try {
    for (const auto & item : node) {
      values.push_back(item.as<std::int64_t>());
    }
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must contain integers");
  }
  return values;
}

std::vector<std::string> required_profile_strings(
  const YAML::Node & parent,
  const std::string & key,
  std::size_t expected_count,
  const std::string & path)
{
  const auto node = required_profile_node(parent, key, path);
  if (!node.IsSequence() || node.size() != expected_count) {
    throw std::runtime_error(
      path + ": field '" + key + "' must contain exactly " +
      std::to_string(expected_count) + " names");
  }
  std::vector<std::string> values;
  values.reserve(expected_count);
  try {
    for (const auto & item : node) {
      const auto value = item.as<std::string>();
      if (value.empty()) {
        throw std::runtime_error(path + ": field '" + key + "' contains an empty name");
      }
      values.push_back(value);
    }
  } catch (const YAML::Exception &) {
    throw std::runtime_error(path + ": field '" + key + "' must contain strings");
  }
  return values;
}

RawArmorDetection::ArmorTypeHint parse_profile_armor_type(
  const std::string & value, const std::string & context)
{
  if (value == "small") {
    return RawArmorDetection::ArmorTypeHint::Small;
  }
  if (value == "large") {
    return RawArmorDetection::ArmorTypeHint::Large;
  }
  throw std::runtime_error(context + ": armor type must be 'small' or 'large'");
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

std::optional<float> sigmoid_probability(float value) noexcept
{
  if (!std::isfinite(value)) {
    return std::nullopt;
  }
  if (value >= 0.0F) {
    const float z = std::exp(-value);
    return 1.0F / (1.0F + z);
  }
  const float z = std::exp(value);
  return z / (1.0F + z);
}

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

std::optional<std::string> ModelProfile::validate() const
{
  if (schema_version != 1 && schema_version != 2) {
    return "unsupported schema_version; expected 1 (legacy test_only) or 2";
  }
  if (model_id.empty() || model_artifacts.xml.path.empty() || source.empty() || version.empty()) {
    return "model_id, model.artifacts.xml.path, source, and version are required";
  }
  if (schema_version == 2 && model_format != "openvino_ir") {
    return "schema_version 2 model.format must be openvino_ir";
  }
  if (schema_version == 2 && model_artifacts.bin.path.empty()) {
    return "schema_version 2 model.artifacts.bin.path is required";
  }
  if (schema_version == 1 && !test_only) {
    return "production model profiles require schema_version 2 XML/BIN artifact manifests";
  }
  if (!test_only) {
    if (model_format != "openvino_ir") {
      return "production model.format must be openvino_ir";
    }
    const std::filesystem::path xml_path(model_artifacts.xml.path);
    const std::filesystem::path bin_path(model_artifacts.bin.path);
    if (!xml_path.is_absolute() || model_artifacts.xml.path.find("://") != std::string::npos) {
      return "production model.artifacts.xml.path must be an absolute local artifact path";
    }
    if (!bin_path.is_absolute() || model_artifacts.bin.path.find("://") != std::string::npos) {
      return "production model.artifacts.bin.path must be an absolute local artifact path";
    }
    if (same_model_path(model_artifacts.xml.path, model_artifacts.bin.path)) {
      return "production XML and BIN artifact paths must be distinct";
    }
    if (!model_artifacts.xml.sha256.has_value()) {
      return "production model.artifacts.xml.sha256 must be declared";
    }
    if (!model_artifacts.bin.sha256.has_value()) {
      return "production model.artifacts.bin.sha256 must be declared";
    }
  }
  if (model_artifacts.xml.sha256.has_value() &&
    !valid_sha256_digest(*model_artifacts.xml.sha256))
  {
    return "model.artifacts.xml.sha256 must contain exactly 64 hexadecimal characters";
  }
  if (model_artifacts.bin.sha256.has_value() &&
    !valid_sha256_digest(*model_artifacts.bin.sha256))
  {
    return "model.artifacts.bin.sha256 must contain exactly 64 hexadecimal characters";
  }
  if (input_shape[0] != 1 || input_shape[1] != 3 || input_shape[2] == 0 || input_shape[3] == 0 ||
    input_shape[2] > static_cast<std::size_t>(std::numeric_limits<int>::max()) ||
    input_shape[3] > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    return "input_shape must be [1,3,height,width] with positive height/width";
  }
  if (input_layout != "NCHW") {
    return "input_layout must be NCHW";
  }
  if (input_element_type != "f32" || source_color_order != "BGR" || model_color_order != "RGB") {
    return "input contract must declare FP32 BGR source to RGB model order";
  }
  if (normalization != "divide_255") {
    return "normalization must be divide_255";
  }
  if (resize_mode != "top_left" && resize_mode != "center") {
    return "resize_mode must be top_left or center";
  }
  if (output_shape[0] != 1 || output_shape[1] == 0 || output_shape[2] == 0) {
    return "output_shape must be [1,rows,columns] with positive rows/columns";
  }
  if (output_layout != "NRC") {
    return "output_layout must be NRC";
  }
  if (output_element_type != "f32") {
    return "output_element_type must be f32 for the current decoder";
  }
  if (keypoint_count != 4) {
    return "keypoint_count must be exactly 4";
  }
  if (objectness_index != 8 || color_logits_offset != 9 || armor_logits_offset != 13) {
    return "current decoder requires objectness_index=8, color_logits_offset=9, "
           "armor_logits_offset=13";
  }
  const auto range_fits = [](std::size_t offset, std::size_t count, std::size_t limit) {
      return offset <= limit && count <= limit - offset;
    };
  if (color_class_count == 0 || armor_class_count == 0 ||
    !range_fits(color_logits_offset, color_class_count, output_shape[2]) ||
    !range_fits(armor_logits_offset, armor_class_count, output_shape[2]))
  {
    return "class logit ranges must be non-empty and fit inside output_shape columns";
  }
  if (!std::isfinite(objectness_threshold) || objectness_threshold < 0.0F ||
    objectness_threshold > 1.0F || !std::isfinite(nms_threshold) || nms_threshold < 0.0F ||
    nms_threshold > 1.0F)
  {
    return "objectness_threshold and nms_threshold must be finite values in [0,1]";
  }
  std::set<int> keypoint_indices;
  for (const int index : keypoint_order) {
    if (index < 0 || index >= static_cast<int>(keypoint_count) ||
      !keypoint_indices.insert(index).second)
    {
      return "keypoint_order must be a permutation of [0,1,2,3]";
    }
  }
  if (color_names.size() != color_class_count || armor_names.size() != armor_class_count) {
    return "color_names/armor_names length must match class counts";
  }
  std::set<std::string> names;
  for (const auto & name : color_names) {
    if (name.empty() || !names.insert(name).second) {
      return "color_names must be non-empty and unique";
    }
  }
  names.clear();
  for (const auto & name : armor_names) {
    if (name.empty() || !names.insert(name).second) {
      return "armor_names must be non-empty and unique";
    }
  }
  if (class_to_armor_type.size() != armor_class_count) {
    return "class_to_armor_type must explicitly map every supported class exactly once";
  }
  for (const auto & [class_id, type] : class_to_armor_type) {
    if (class_id < 0 || class_id >= static_cast<int>(armor_class_count) ||
      (type != RawArmorDetection::ArmorTypeHint::Small &&
      type != RawArmorDetection::ArmorTypeHint::Large))
    {
      return "class_to_armor_type contains an invalid class id or armor type";
    }
  }
  for (std::size_t class_id = 0; class_id < armor_class_count; ++class_id) {
    if (class_to_armor_type.find(static_cast<int>(class_id)) == class_to_armor_type.end()) {
      return "class_to_armor_type must contain every class id from zero to armor_class_count-1";
    }
  }
  return std::nullopt;
}

ModelProfile load_model_profile(const std::string & yaml_path, ModelProfileLoadOptions options)
{
  if (yaml_path.empty() || !std::filesystem::is_regular_file(yaml_path)) {
    throw std::runtime_error("model profile does not exist: " + yaml_path);
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception & error) {
    throw std::runtime_error("cannot parse model profile " + yaml_path + ": " + error.what());
  }
  if (!root.IsMap()) {
    throw std::runtime_error("model profile root must be a map: " + yaml_path);
  }

  ModelProfile result{};
  const auto schema_version = required_profile_integer(root, "schema_version", "root");
  if (schema_version < 0 || schema_version > std::numeric_limits<int>::max()) {
    throw std::runtime_error("root.schema_version must be a non-negative integer");
  }
  result.schema_version = static_cast<int>(schema_version);
  const auto profile = required_profile_string(root, "profile", "root");
  if (profile == "test_only") {
    result.test_only = true;
  } else if (profile == "production") {
    result.test_only = false;
  } else {
    throw std::runtime_error("root.profile must be 'test_only' or 'production'");
  }
  if (result.test_only && !options.allow_test_only) {
    throw std::runtime_error(
      "refusing test_only model profile without explicit allow_test_only=true: " + yaml_path);
  }

  const auto model_node = required_profile_node(root, "model", "root");
  if (!model_node.IsMap()) {
    throw std::runtime_error("model must be a map");
  }
  result.model_id = required_profile_string(model_node, "id", "model");
  if (result.schema_version == 1) {
    // Schema v1 is retained strictly for checked-in test-only fixtures.  Its
    // single historical path is represented as the XML graph member, with no
    // inferred sibling BIN artifact and no route to production admission.
    result.model_format = "legacy_single_path";
    result.model_artifacts.xml.path = required_profile_string(model_node, "path", "model");
    result.model_artifacts.xml.sha256 = optional_profile_string(model_node, "sha256", "model");
  } else if (result.schema_version == 2) {
    result.model_format = required_profile_string(model_node, "format", "model");
    const auto artifacts_node = required_profile_node(model_node, "artifacts", "model");
    if (!artifacts_node.IsMap()) {
      throw std::runtime_error("model.artifacts must be a map");
    }
    const auto load_artifact = [&artifacts_node](
      const std::string & name, ModelArtifact & artifact) {
        const auto artifact_node = required_profile_node(
          artifacts_node, name, "model.artifacts");
        if (!artifact_node.IsMap()) {
          throw std::runtime_error("model.artifacts." + name + " must be a map");
        }
        artifact.path = required_profile_string(
          artifact_node, "path", "model.artifacts." + name);
        artifact.sha256 = optional_profile_string(
          artifact_node, "sha256", "model.artifacts." + name);
      };
    load_artifact("xml", result.model_artifacts.xml);
    load_artifact("bin", result.model_artifacts.bin);
  }
  result.source = required_profile_string(model_node, "source", "model");
  result.version = required_profile_string(model_node, "version", "model");

  const auto input_node = required_profile_node(root, "input", "root");
  if (!input_node.IsMap()) {
    throw std::runtime_error("input must be a map");
  }
  const auto input_shape = required_profile_integer_sequence(input_node, "shape", 4, "input");
  for (std::size_t index = 0; index < result.input_shape.size(); ++index) {
    if (input_shape[index] < 0) {
      throw std::runtime_error("input.shape must contain non-negative integers");
    }
    result.input_shape[index] = static_cast<std::size_t>(input_shape[index]);
  }
  result.input_layout = required_profile_string(input_node, "layout", "input");
  result.input_element_type = required_profile_string(input_node, "element_type", "input");
  result.source_color_order = required_profile_string(input_node, "source_color_order", "input");
  result.model_color_order = required_profile_string(input_node, "model_color_order", "input");
  result.normalization = required_profile_string(input_node, "normalization", "input");
  result.resize_mode = required_profile_string(input_node, "resize_mode", "input");

  const auto output_node = required_profile_node(root, "output", "root");
  if (!output_node.IsMap()) {
    throw std::runtime_error("output must be a map");
  }
  const auto output_shape = required_profile_integer_sequence(output_node, "shape", 3, "output");
  for (std::size_t index = 0; index < result.output_shape.size(); ++index) {
    if (output_shape[index] < 0) {
      throw std::runtime_error("output.shape must contain non-negative integers");
    }
    result.output_shape[index] = static_cast<std::size_t>(output_shape[index]);
  }
  result.output_layout = required_profile_string(output_node, "layout", "output");
  result.output_element_type = required_profile_string(output_node, "element_type", "output");
  const auto keypoint_count = required_profile_integer(output_node, "keypoint_count", "output");
  if (keypoint_count < 0) {
    throw std::runtime_error("output.keypoint_count must be non-negative");
  }
  result.keypoint_count = static_cast<std::size_t>(keypoint_count);
  const auto objectness_index = required_profile_integer(output_node, "objectness_index", "output");
  const auto color_logits_offset = required_profile_integer(
    output_node, "color_logits_offset", "output");
  const auto color_class_count = required_profile_integer(
    output_node, "color_class_count", "output");
  const auto armor_logits_offset = required_profile_integer(
    output_node, "armor_logits_offset", "output");
  const auto armor_class_count = required_profile_integer(
    output_node, "armor_class_count", "output");
  if (objectness_index < 0 || color_logits_offset < 0 || color_class_count < 0 ||
    armor_logits_offset < 0 || armor_class_count < 0)
  {
    throw std::runtime_error("output offsets and class counts must be non-negative");
  }
  result.objectness_index = static_cast<std::size_t>(objectness_index);
  result.color_logits_offset = static_cast<std::size_t>(color_logits_offset);
  result.color_class_count = static_cast<std::size_t>(color_class_count);
  result.armor_logits_offset = static_cast<std::size_t>(armor_logits_offset);
  result.armor_class_count = static_cast<std::size_t>(armor_class_count);

  const auto postprocess_node = required_profile_node(root, "postprocess", "root");
  if (!postprocess_node.IsMap()) {
    throw std::runtime_error("postprocess must be a map");
  }
  result.objectness_threshold = static_cast<float>(
    required_profile_double(postprocess_node, "objectness_threshold", "postprocess"));
  result.nms_threshold = static_cast<float>(
    required_profile_double(postprocess_node, "nms_threshold", "postprocess"));
  const auto keypoint_order = required_profile_integer_sequence(
    postprocess_node, "keypoint_order", 4, "postprocess");
  for (std::size_t index = 0; index < result.keypoint_order.size(); ++index) {
    if (keypoint_order[index] < std::numeric_limits<int>::min() ||
      keypoint_order[index] > std::numeric_limits<int>::max())
    {
      throw std::runtime_error("postprocess.keypoint_order contains an out-of-range integer");
    }
    result.keypoint_order[index] = static_cast<int>(keypoint_order[index]);
  }

  const auto semantics_node = required_profile_node(root, "semantics", "root");
  if (!semantics_node.IsMap()) {
    throw std::runtime_error("semantics must be a map");
  }
  result.color_names = required_profile_strings(
    semantics_node, "color_id_to_name", result.color_class_count, "semantics");
  result.armor_names = required_profile_strings(
    semantics_node, "armor_class_names", result.armor_class_count, "semantics");
  const auto mapping_node = required_profile_node(semantics_node, "class_to_armor_type", "semantics");
  if (!mapping_node.IsMap()) {
    throw std::runtime_error("semantics.class_to_armor_type must be a map");
  }
  for (const auto & item : mapping_node) {
    try {
      const auto class_id = item.first.as<int>();
      const auto armor_type = parse_profile_armor_type(
        item.second.as<std::string>(), "semantics.class_to_armor_type");
      const auto [ignored, inserted] = result.class_to_armor_type.emplace(class_id, armor_type);
      if (!inserted) {
        throw std::runtime_error("semantics.class_to_armor_type contains a duplicate class id");
      }
    } catch (const YAML::Exception &) {
      throw std::runtime_error(
        "semantics.class_to_armor_type must map integer class ids to small/large");
    }
  }

  if (const auto error = result.validate(); error.has_value()) {
    throw std::runtime_error("invalid model profile " + yaml_path + ": " + *error);
  }
  return result;
}

DetectorConfig detector_config_from_model_profile(
  const ModelProfile & profile,
  std::string model_path,
  std::string device,
  std::string model_bin_path)
{
  if (const auto error = profile.validate(); error.has_value()) {
    throw std::invalid_argument("invalid model profile: " + *error);
  }
  if (model_path.empty()) {
    model_path = profile.model_artifacts.xml.path;
  }
  if (model_bin_path.empty()) {
    model_bin_path = profile.model_artifacts.bin.path;
  }
  if (!profile.test_only && !same_model_path(model_path, profile.model_artifacts.xml.path)) {
    throw std::invalid_argument(
            "production model XML artifact path does not match model profile path: profile=" +
            profile.model_artifacts.xml.path + " runtime=" + model_path);
  }
  if (!profile.test_only && !same_model_path(model_bin_path, profile.model_artifacts.bin.path)) {
    throw std::invalid_argument(
            "production model BIN artifact path does not match model profile path: profile=" +
            profile.model_artifacts.bin.path + " runtime=" + model_bin_path);
  }
  DetectorConfig result{};
  result.model_path = std::move(model_path);
  result.model_bin_path = std::move(model_bin_path);
  result.require_model_path_match = !profile.test_only;
  result.reviewed_model_path = profile.model_artifacts.xml.path;
  result.require_model_bin_path_match = !profile.test_only;
  result.reviewed_model_bin_path = profile.model_artifacts.bin.path;
  result.require_model_hash_match = !profile.test_only;
  result.reviewed_model_sha256 = profile.model_artifacts.xml.sha256.value_or("");
  result.reviewed_model_bin_sha256 = profile.model_artifacts.bin.sha256.value_or("");
  result.device = std::move(device);
  result.expected_input_element_type = profile.input_element_type;
  result.expected_output_element_type = profile.output_element_type;
  result.expected_input_layout = profile.input_layout;
  result.expected_output_layout = profile.output_layout;
  result.input_width = static_cast<int>(profile.input_shape[3]);
  result.input_height = static_cast<int>(profile.input_shape[2]);
  result.expected_output_rows = profile.output_shape[1];
  result.expected_output_columns = profile.output_shape[2];
  result.color_class_count = profile.color_class_count;
  result.armor_class_count = profile.armor_class_count;
  result.objectness_index = profile.objectness_index;
  result.color_logits_offset = profile.color_logits_offset;
  result.armor_logits_offset = profile.armor_logits_offset;
  result.objectness_threshold = profile.objectness_threshold;
  result.nms_threshold = profile.nms_threshold;
  result.center_padding = profile.resize_mode == "center";
  result.keypoint_order = profile.keypoint_order;
  result.class_to_armor_type = profile.class_to_armor_type;
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
  const auto minimum_columns = std::max(
    {config.objectness_index + 1,
      config.color_logits_offset + config.color_class_count,
      config.armor_logits_offset + config.armor_class_count});
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

std::optional<std::string> validate_runtime_layout(
  const std::string & actual_layout,
  const std::string & expected_layout,
  const std::string & port_name)
{
  if (expected_layout.empty()) {
    return std::nullopt;
  }
  const auto expected = canonical_layout_string(expected_layout);
  if (expected.empty()) {
    return "expected " + port_name + " layout must not be empty";
  }
  const auto actual = canonical_layout_string(actual_layout);
  if (actual.empty()) {
    return "model " + port_name + " layout is unavailable; refusing to infer axis order";
  }
  if (actual != expected) {
    return "model " + port_name + " layout does not match profile: expected " +
           expected + ", got " + actual;
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
  result.scale_x = static_cast<float>(resized_width) / static_cast<float>(bgr_image.cols);
  result.scale_y = static_cast<float>(resized_height) / static_cast<float>(bgr_image.rows);
  if (!std::isfinite(result.scale_x) || !std::isfinite(result.scale_y) || result.scale_x <= 0.0F ||
    result.scale_y <= 0.0F)
  {
    return std::nullopt;
  }
  result.resized_width = resized_width;
  result.resized_height = resized_height;
  result.pad_x = pad_x;
  result.pad_y = pad_y;
  return result;
}

std::optional<PreprocessedTensor> make_preprocessed_tensor(
  const cv::Mat & bgr_image,
  const DetectorConfig & config)
{
  const auto letterbox = make_letterbox(bgr_image, config);
  if (!letterbox.has_value()) {
    return std::nullopt;
  }

  const auto width = static_cast<std::size_t>(letterbox->image.cols);
  const auto height = static_cast<std::size_t>(letterbox->image.rows);
  if (width == 0 || height == 0 || width > std::numeric_limits<std::size_t>::max() / height ||
    width * height > std::numeric_limits<std::size_t>::max() / 3U)
  {
    return std::nullopt;
  }

  const std::size_t plane_size = width * height;
  PreprocessedTensor result{};
  result.letterbox = *letterbox;
  result.nchw_rgb_f32.resize(plane_size * 3U);
  constexpr float kInv255 = 1.0F / 255.0F;
  for (int row = 0; row < result.letterbox.image.rows; ++row) {
    const auto * pixels = result.letterbox.image.ptr<cv::Vec3b>(row);
    for (int column = 0; column < result.letterbox.image.cols; ++column) {
      const std::size_t index = static_cast<std::size_t>(row) * width +
        static_cast<std::size_t>(column);
      const auto & bgr = pixels[column];
      result.nchw_rgb_f32[index] = static_cast<float>(bgr[2]) * kInv255;
      result.nchw_rgb_f32[plane_size + index] = static_cast<float>(bgr[1]) * kInv255;
      result.nchw_rgb_f32[2U * plane_size + index] = static_cast<float>(bgr[0]) * kInv255;
    }
  }
  return result;
}

std::optional<cv::Point2f> model_point_to_image(
  const cv::Point2f & model_point,
  const LetterboxResult & letterbox) noexcept
{
  if (!finite_point(model_point) || !std::isfinite(letterbox.scale) || letterbox.scale <= 0.0F ||
    !std::isfinite(letterbox.scale_x) || !std::isfinite(letterbox.scale_y) ||
    letterbox.scale_x <= 0.0F || letterbox.scale_y <= 0.0F)
  {
    return std::nullopt;
  }
  const cv::Point2f image_point{
    (model_point.x - static_cast<float>(letterbox.pad_x)) / letterbox.scale_x,
    (model_point.y - static_cast<float>(letterbox.pad_y)) / letterbox.scale_y};
  if (!finite_point(image_point)) {
    return std::nullopt;
  }
  return image_point;
}

std::optional<cv::Point2f> model_point_to_image(
  const cv::Point2f & model_point,
  const LetterboxResult & letterbox,
  const cv::Size & source_image_size) noexcept
{
  if (source_image_size.width <= 0 || source_image_size.height <= 0) {
    return std::nullopt;
  }
  if (letterbox.resized_width <= 0 || letterbox.resized_height <= 0 ||
    model_point.x < static_cast<float>(letterbox.pad_x) ||
    model_point.y < static_cast<float>(letterbox.pad_y) ||
    model_point.x >= static_cast<float>(letterbox.pad_x + letterbox.resized_width) ||
    model_point.y >= static_cast<float>(letterbox.pad_y + letterbox.resized_height))
  {
    return std::nullopt;
  }
  const auto image_point = model_point_to_image(model_point, letterbox);
  if (!image_point.has_value() || image_point->x < 0.0F || image_point->y < 0.0F ||
    image_point->x >= static_cast<float>(source_image_size.width) ||
    image_point->y >= static_cast<float>(source_image_size.height))
  {
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
  if (impl_->config.require_model_path_match &&
    !same_model_path(impl_->config.model_path, impl_->config.reviewed_model_path))
  {
    throw std::invalid_argument(
            "OpenVINO model XML artifact path does not match the reviewed model profile");
  }
  if (impl_->config.require_model_bin_path_match &&
    !same_model_path(impl_->config.model_bin_path, impl_->config.reviewed_model_bin_path))
  {
    throw std::invalid_argument(
            "OpenVINO model BIN artifact path does not match the reviewed model profile");
  }
  if (!std::filesystem::exists(impl_->config.model_path)) {
    throw std::runtime_error("OpenVINO model XML file does not exist: " + impl_->config.model_path);
  }
  if (!impl_->config.model_bin_path.empty() &&
    !std::filesystem::exists(impl_->config.model_bin_path))
  {
    throw std::runtime_error(
            "OpenVINO model BIN file does not exist: " + impl_->config.model_bin_path);
  }
  if (impl_->config.require_model_bin_path_match && impl_->config.model_bin_path.empty()) {
    throw std::invalid_argument("OpenVINO model BIN path must not be empty for a reviewed manifest");
  }
  if (const auto error = validate_model_manifest_sha256(impl_->config); error.has_value()) {
    throw std::runtime_error(*error);
  }
#ifndef AUTO_AIM_HAS_OPENVINO
  throw std::runtime_error(
    "OpenVINO support is not built; configure OpenVINO_DIR before building auto_aim_ros2");
#else
  try {
    // A schema-v2 profile passes the reviewed XML and BIN explicitly.  The
    // legacy/test-only path deliberately leaves the BIN empty and retains
    // OpenVINO's compatibility auto-discovery behavior only for that case.
    const auto model = impl_->config.model_bin_path.empty() ?
      impl_->core.read_model(impl_->config.model_path) :
      impl_->core.read_model(impl_->config.model_path, impl_->config.model_bin_path);
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
    const auto input_layout = ov::layout::get_layout(model->input());
    const auto output_layout = ov::layout::get_layout(model->output());
    const auto input_layout_text =
      input_layout.empty() ? std::string{} : input_layout.to_string();
    const auto output_layout_text =
      output_layout.empty() ? std::string{} : output_layout.to_string();
    if (const auto error = validate_runtime_layout(
        input_layout_text, impl_->config.expected_input_layout, "input");
      error.has_value())
    {
      throw std::runtime_error(*error);
    }
    if (const auto error = validate_runtime_layout(
        output_layout_text, impl_->config.expected_output_layout, "output");
      error.has_value())
    {
      throw std::runtime_error(*error);
    }
    // The detector always supplies an explicitly constructed FP32 NCHW
    // tensor.  Do not let an unprofiled/direct DetectorConfig clear its
    // string expectation and defer a type mismatch until the first frame.
    if (model->input().get_element_type() != ov::element::f32) {
      throw std::runtime_error(
        "expected FP32 model input, got " + model->input().get_element_type().to_string());
    }
    if (model->output().get_element_type() != ov::element::f32) {
      throw std::runtime_error(
        "expected FP32 model output, got " + model->output().get_element_type().to_string());
    }
    if (!impl_->config.expected_input_element_type.empty() &&
      model->input().get_element_type().to_string() != impl_->config.expected_input_element_type)
    {
      throw std::runtime_error(
        "model input element type does not match profile: expected " +
        impl_->config.expected_input_element_type + ", got " +
        model->input().get_element_type().to_string());
    }
    if (!impl_->config.expected_output_element_type.empty() &&
      model->output().get_element_type().to_string() != impl_->config.expected_output_element_type)
    {
      throw std::runtime_error(
        "model output element type does not match profile: expected " +
        impl_->config.expected_output_element_type + ", got " +
        model->output().get_element_type().to_string());
    }

    impl_->info.input_shape = input_shape;
    impl_->info.output_shape = output_shape;
    impl_->info.input_element_type = model->input().get_element_type().to_string();
    impl_->info.output_element_type = model->output().get_element_type().to_string();
    impl_->info.input_layout = input_layout_text;
    impl_->info.output_layout = output_layout_text;

    impl_->compiled_model = impl_->core.compile_model(
      model, impl_->config.device,
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
  auto preprocessed = make_preprocessed_tensor(frame.bgr_image, impl_->config);
  if (!preprocessed.has_value()) {
    return {};
  }
#ifndef AUTO_AIM_HAS_OPENVINO
  return {};
#else
  auto request = impl_->compiled_model.create_infer_request();
  ov::Tensor input_tensor(
    ov::element::f32,
    {1, 3, static_cast<std::size_t>(impl_->config.input_height),
     static_cast<std::size_t>(impl_->config.input_width)},
    preprocessed->nchw_rgb_f32.data());
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
  const std::size_t color_offset = impl_->config.color_logits_offset;
  const std::size_t class_offset = impl_->config.armor_logits_offset;
  std::vector<RawArmorDetection> candidates;
  std::vector<cv::Rect> boxes;
  std::vector<float> confidences;
  candidates.reserve(rows);
  boxes.reserve(rows);
  confidences.reserve(rows);

  for (std::size_t row = 0; row < rows; ++row) {
    const float * current = values + row * columns;
    const auto confidence = sigmoid_probability(current[impl_->config.objectness_index]);
    if (!confidence.has_value() || *confidence < impl_->config.objectness_threshold) {
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
        {current[raw_offset], current[raw_offset + 1]}, preprocessed->letterbox,
        frame.bgr_image.size());
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
      class_id, color_id, *confidence, bbox, keypoints,
      static_cast<int>(impl_->config.armor_class_count),
      static_cast<int>(impl_->config.color_class_count));
    if (!detection.has_value()) {
      continue;
    }
    auto typed_detection = *detection;
    if (!impl_->config.class_to_armor_type.empty()) {
      const auto type = impl_->config.class_to_armor_type.find(class_id);
      if (type == impl_->config.class_to_armor_type.end()) {
        // A reviewed profile must describe every supported class.  Do not
        // guess a physical armor size for an unknown semantic id.
        continue;
      }
      typed_detection.armor_type = type->second;
    }
    candidates.push_back(typed_detection);
    boxes.push_back(to_nms_rect(bbox));
    confidences.push_back(*confidence);
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
