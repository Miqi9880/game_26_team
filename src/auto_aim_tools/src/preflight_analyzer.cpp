#include "auto_aim_tools/preflight_analyzer.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace auto_aim_tools
{
namespace
{
constexpr std::size_t status_index(Status status) noexcept
{
  return static_cast<std::size_t>(status);
}

std::string number(double value)
{
  std::ostringstream stream;
  stream << std::setprecision(12) << value;
  return stream.str();
}

template<typename Integer>
std::string integer(Integer value)
{
  return std::to_string(value);
}

std::string size_value(std::size_t value)
{
  return std::to_string(value);
}

bool finite_values(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

bool finite_values(const std::vector<float> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](float value) {
      return std::isfinite(value);
    });
}

std::optional<std::int64_t> stamp_ns(const HeaderStamp & stamp)
{
  if (!stamp.readable || stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return std::nullopt;
  }
  if (stamp.sec > std::numeric_limits<std::int64_t>::max() / 1'000'000'000LL) {
    return std::nullopt;
  }
  return stamp.sec * 1'000'000'000LL + static_cast<std::int64_t>(stamp.nanosec);
}

std::string lowercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::toupper(character));
    });
  return value;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (character < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<int>(character) << std::dec;
        } else {
          stream << character;
        }
    }
  }
  return stream.str();
}

std::optional<std::array<double, 2>> pitch_limits(const std::string & profile)
{
  if (profile == "new_turtle") {
    return std::array<double, 2>{{-20.0, 19.0}};
  }
  if (profile == "dog_leg") {
    return std::array<double, 2>{{-10.0, 31.0}};
  }
  return std::nullopt;
}

bool timed_topic(const std::string & topic)
{
  return topic == kImageTopic || topic == kCameraInfoTopic || topic == kVisionTopic;
}

enum class EncodingWidthStatus : std::uint8_t
{
  Known,
  Unknown,
  Invalid,
};

struct EncodingWidth
{
  EncodingWidthStatus status{EncodingWidthStatus::Unknown};
  std::optional<std::size_t> bytes;
  std::string reason;
};

std::optional<std::size_t> parse_size(const std::string & digits)
{
  std::size_t value = 0U;
  for (const char character : digits) {
    const auto digit = static_cast<std::size_t>(character - '0');
    if (value > (std::numeric_limits<std::size_t>::max() - digit) / 10U) {
      return std::nullopt;
    }
    value = value * 10U + digit;
  }
  return value;
}

EncodingWidth encoding_width(const std::string & encoding)
{
  static const std::map<std::string, std::size_t> fixed_encodings{
    {"mono8", 1U}, {"mono16", 2U}, {"bgr8", 3U}, {"rgb8", 3U},
    {"bgra8", 4U}, {"rgba8", 4U}, {"bayer_rggb8", 1U},
    {"bayer_bggr8", 1U}, {"bayer_gbrg8", 1U}, {"bayer_grbg8", 1U},
    {"bayer_rggb16", 2U}, {"bayer_bggr16", 2U}, {"bayer_gbrg16", 2U},
    {"bayer_grbg16", 2U}, {"yuv422", 2U}, {"yuyv", 2U}, {"uyvy", 2U},
  };
  const auto fixed = fixed_encodings.find(lowercase(encoding));
  if (fixed != fixed_encodings.end()) {
    return EncodingWidth{EncodingWidthStatus::Known, fixed->second, ""};
  }

  static const std::regex typed_encoding(R"(^([0-9]+)(U|S|F)C([1-9][0-9]*)$)");
  std::smatch match;
  const auto canonical = uppercase(encoding);
  if (!std::regex_match(canonical, match, typed_encoding)) {
    return EncodingWidth{
      EncodingWidthStatus::Unknown, std::nullopt,
      "Encoding is not in the known byte-width table"};
  }

  const auto bits = parse_size(match[1].str());
  const auto channels = parse_size(match[3].str());
  if (!bits.has_value() || !channels.has_value()) {
    return EncodingWidth{
      EncodingWidthStatus::Invalid, std::nullopt,
      "Typed encoding numeric component exceeds size_t"};
  }
  if (*bits != 8U && *bits != 16U && *bits != 32U && *bits != 64U) {
    return EncodingWidth{
      EncodingWidthStatus::Invalid, std::nullopt,
      "Typed encoding bit depth must be 8, 16, 32, or 64"};
  }
  const std::size_t channel_bytes = *bits / 8U;
  if (*channels > std::numeric_limits<std::size_t>::max() / channel_bytes) {
    return EncodingWidth{
      EncodingWidthStatus::Invalid, std::nullopt,
      "Typed encoding bytes per pixel overflows size_t"};
  }
  return EncodingWidth{
    EncodingWidthStatus::Known, *channels * channel_bytes, ""};
}

}  // namespace

const char * status_name(Status status) noexcept
{
  switch (status) {
    case Status::Pass: return "PASS";
    case Status::Warn: return "WARN";
    case Status::Fail: return "FAIL";
  }
  return "FAIL";
}

std::optional<std::size_t> bytes_per_pixel(const std::string & encoding)
{
  return encoding_width(encoding).bytes;
}

PreflightAnalyzer::PreflightAnalyzer(PreflightConfig config, double start_s)
: config_(std::move(config)), start_s_(start_s)
{
  if (!std::isfinite(config_.timeout_s) || config_.timeout_s <= 0.0) {
    throw std::invalid_argument("timeout_s must be finite and positive");
  }
  if (config_.vehicle_profile != "unselected" &&
    config_.vehicle_profile != "new_turtle" && config_.vehicle_profile != "dog_leg")
  {
    throw std::invalid_argument("unknown vehicle profile");
  }
  if (!std::isfinite(config_.sync_tolerance_ms) || config_.sync_tolerance_ms < 0.0) {
    throw std::invalid_argument("sync_tolerance_ms must be finite and non-negative");
  }
  if (config_.expected_frame_id.empty()) {
    throw std::invalid_argument("expected_frame_id must not be empty");
  }
  stats_.emplace(kImageTopic, TopicStats{});
  stats_.emplace(kCameraInfoTopic, TopicStats{});
  stats_.emplace(kVisionTopic, TopicStats{});
}

const PreflightConfig & PreflightAnalyzer::config() const noexcept
{
  return config_;
}

void PreflightAnalyzer::record(
  const std::string & key, Status status, const std::string & check,
  const std::string & topic, const std::string & reason,
  std::map<std::string, std::string> details)
{
  const auto existing = findings_.find(key);
  if (existing != findings_.end() && existing->second.status > status) {
    return;
  }
  findings_[key] = Finding{status, check, topic, reason, std::move(details)};
}

void PreflightAnalyzer::observe_common(
  const std::string & topic, const HeaderStamp * stamp, double arrival_s)
{
  auto & stats = stats_.at(topic);
  ++stats.count;
  if (!stats.first_arrival_s.has_value()) {
    stats.first_arrival_s = arrival_s;
  }
  stats.last_arrival_s = arrival_s;
  if (stamp == nullptr) {
    return;
  }
  const auto current = stamp_ns(*stamp);
  if (!current.has_value()) {
    ++stats.invalid_timestamps;
    return;
  }
  if (*current == 0) {
    ++stats.unset_timestamps;
  }
  if (stats.last_stamp_ns.has_value()) {
    if (*current < *stats.last_stamp_ns) {
      ++stats.timestamp_rollbacks;
    } else if (*current == *stats.last_stamp_ns) {
      ++stats.duplicate_timestamps;
    }
  }
  stats.last_stamp_ns = current;
}

void PreflightAnalyzer::pair_frame(
  bool image, const HeaderStamp & stamp, const FrameMetadata & metadata)
{
  const auto timestamp = stamp_ns(stamp);
  if (!timestamp.has_value() || *timestamp == 0) {
    return;
  }

  auto & own = image ? unmatched_images_ : unmatched_camera_info_;
  auto & other = image ? unmatched_camera_info_ : unmatched_images_;
  const auto range = other.equal_range(*timestamp);
  if (range.first == range.second) {
    own.emplace(*timestamp, metadata);
    return;
  }

  const auto counterpart = range.first->second;
  other.erase(range.first);
  last_matched_stamp_ns_ = timestamp;
  if (image) {
    validate_pair(metadata, counterpart);
  } else {
    validate_pair(counterpart, metadata);
  }
}

void PreflightAnalyzer::validate_pair(
  const FrameMetadata & image, const FrameMetadata & camera_info)
{
  ++matched_pairs_;
  if (image.width != camera_info.width || image.height != camera_info.height) {
    ++dimension_mismatches_;
  }
  if (image.frame_id != camera_info.frame_id) {
    ++frame_id_mismatches_;
  }
}

void PreflightAnalyzer::observe_image(const ImageSample & sample, double arrival_s)
{
  observe_common(kImageTopic, &sample.stamp, arrival_s);
  const bool frame_id_valid = sample.frame_id == config_.expected_frame_id;
  record(
    "image.frame_id", frame_id_valid ? Status::Pass : Status::Fail,
    "image.frame_id", kImageTopic,
    frame_id_valid ? "Image frame_id matches the configured camera frame" :
    "Image frame_id is empty or does not match --expected-frame-id",
    {{"actual", sample.frame_id}, {"expected", config_.expected_frame_id}});
  const bool dimensions_valid = sample.width > 0U && sample.height > 0U;
  record(
    "image.dimensions", dimensions_valid ? Status::Pass : Status::Fail,
    "image.dimensions", kImageTopic,
    dimensions_valid ? "Image dimensions are positive" :
    "Image width and height must both be positive",
    {{"width", std::to_string(sample.width)}, {"height", std::to_string(sample.height)}});
  pair_frame(
    true, sample.stamp,
    FrameMetadata{sample.frame_id, sample.width, sample.height, arrival_s});

  const bool encoding_valid = sample.encoding == kExpectedImageEncoding;
  record(
    "image.encoding", encoding_valid ? Status::Pass : Status::Fail,
    "image.encoding", kImageTopic,
    encoding_valid ? "Image encoding matches the camera contract" :
    "Image encoding must be exactly rgb8 for this camera-to-detector contract",
    {{"actual", sample.encoding}, {"expected", kExpectedImageEncoding}});

  bool row_width_representable = true;
  bool step_valid = dimensions_valid && encoding_valid && sample.step > 0U;
  std::uint64_t minimum_step = 0U;
  constexpr std::size_t rgb8_bytes_per_pixel = 3U;
  if (sample.width > 0U) {
    if (rgb8_bytes_per_pixel >
      std::numeric_limits<std::uint64_t>::max() /
      static_cast<std::uint64_t>(sample.width))
    {
      row_width_representable = false;
    } else {
      minimum_step = static_cast<std::uint64_t>(sample.width) * rgb8_bytes_per_pixel;
      row_width_representable =
        minimum_step <= std::numeric_limits<std::uint32_t>::max();
    }
    step_valid = step_valid && row_width_representable && sample.step == minimum_step;
  }
  const std::string step_reason = !row_width_representable ?
    "Encoded row width cannot be represented by sensor_msgs/Image step" :
    "Image step must equal width * 3 for packed rgb8";
  record(
    "image.step", step_valid ? Status::Pass : Status::Fail, "image.step", kImageTopic,
    step_valid ? "Image step equals width * 3 for packed rgb8" :
    step_reason,
    {{"step", std::to_string(sample.step)}, {"minimum_step", integer(minimum_step)}});

  const auto expected = static_cast<std::uint64_t>(sample.step) * sample.height;
  const bool length_valid = step_valid && expected > 0U && sample.data_size == expected;
  record(
    "image.data_length", length_valid ? Status::Pass : Status::Fail,
    "image.data_length", kImageTopic,
    length_valid ? "Image data length matches step times height" :
    (!step_valid ? "Image step is invalid, so the data layout cannot pass" :
    "Image data must be non-empty and equal step times height"),
    {{"data_length", size_value(sample.data_size)}, {"expected_length", integer(expected)}});
}

void PreflightAnalyzer::observe_camera_info(
  const CameraInfoSample & sample, double arrival_s)
{
  observe_common(kCameraInfoTopic, &sample.stamp, arrival_s);
  const bool frame_id_valid = sample.frame_id == config_.expected_frame_id;
  record(
    "camera.frame_id", frame_id_valid ? Status::Pass : Status::Fail,
    "camera_info.frame_id", kCameraInfoTopic,
    frame_id_valid ? "CameraInfo frame_id matches the configured camera frame" :
    "CameraInfo frame_id is empty or does not match --expected-frame-id",
    {{"actual", sample.frame_id}, {"expected", config_.expected_frame_id}});
  const bool dimensions_valid = sample.width > 0U && sample.height > 0U;
  record(
    "camera.dimensions", dimensions_valid ? Status::Pass : Status::Fail,
    "camera_info.dimensions", kCameraInfoTopic,
    dimensions_valid ? "CameraInfo dimensions are positive" :
    "CameraInfo width and height must both be positive",
    {{"width", std::to_string(sample.width)}, {"height", std::to_string(sample.height)}});
  pair_frame(
    false, sample.stamp,
    FrameMetadata{sample.frame_id, sample.width, sample.height, arrival_s});

  const bool k_valid = sample.k.size() == 9U && finite_values(sample.k) &&
    sample.k[0] > 0.0 && sample.k[4] > 0.0 && std::abs(sample.k[8]) > 1e-12;
  record(
    "camera.k", k_valid ? Status::Pass : Status::Fail, "camera_info.K",
    kCameraInfoTopic,
    k_valid ? "K has 9 finite entries with positive fx/fy and non-zero K[8]" :
    "K must have 9 finite entries, positive fx/fy, and non-zero K[8]",
    {{"length", size_value(sample.k.size())}});

  Status d_status = Status::Warn;
  std::string d_reason;
  if (!finite_values(sample.d)) {
    d_status = Status::Fail;
    d_reason = "D contains a non-finite value";
  } else if (sample.distortion_model.empty()) {
    d_status = Status::Fail;
    d_reason = "distortion_model is empty, so D format cannot be established";
  } else {
    std::optional<std::size_t> expected_length;
    if (sample.distortion_model == "plumb_bob") {
      expected_length = 5U;
    } else if (sample.distortion_model == "rational_polynomial") {
      expected_length = 8U;
    } else if (sample.distortion_model == "equidistant") {
      expected_length = 4U;
    }
    if (!expected_length.has_value()) {
      d_status = Status::Fail;
      d_reason = "Unknown distortion_model; D format cannot satisfy the input contract";
    } else if (sample.d.size() != *expected_length) {
      d_status = Status::Fail;
      d_reason = "D length does not match the declared distortion model";
    } else {
      d_status = Status::Pass;
      d_reason = "D length and finite values match the declared distortion model";
    }
  }
  record(
    "camera.d", d_status, "camera_info.D", kCameraInfoTopic, d_reason,
    {{"distortion_model", sample.distortion_model}, {"length", size_value(sample.d.size())}});
}

void PreflightAnalyzer::observe_topic_publishers(
  const std::string & topic, const std::string & expected_type,
  const std::vector<PublisherEndpointSample> & publishers)
{
  const bool publisher_count_valid = publishers.size() == 1U;
  record(
    "graph.publisher_count." + topic,
    publisher_count_valid ? Status::Pass : Status::Fail,
    "topic.publisher_count", topic,
    publisher_count_valid ? "Exactly one publisher was discovered on the required topic" :
    "Expected exactly one publisher; check topic name, remapping, startup, and duplicate sources",
    {{"count", size_value(publishers.size())}});

  const bool type_valid = !publishers.empty() && std::all_of(
    publishers.begin(), publishers.end(), [&](const PublisherEndpointSample & publisher) {
      return publisher.topic_type == expected_type;
    });
  record(
    "graph.type." + topic, type_valid ? Status::Pass : Status::Fail,
    "topic.type", topic,
    type_valid ? "Publisher type matches the required ROS message type" :
    "Required topic has no publisher or exposes the wrong ROS message type",
    {{"expected", expected_type}});

  const bool delivery_qos_valid = !publishers.empty() && std::all_of(
    publishers.begin(), publishers.end(), [](const PublisherEndpointSample & publisher) {
      return publisher.reliability == "best_effort" &&
      publisher.durability == "volatile";
    });
  const bool queue_qos_available = !publishers.empty() && std::all_of(
    publishers.begin(), publishers.end(), [](const PublisherEndpointSample & publisher) {
      return publisher.history != "unknown" && publisher.depth != 0U;
    });
  const bool queue_qos_valid = queue_qos_available && std::all_of(
    publishers.begin(), publishers.end(), [](const PublisherEndpointSample & publisher) {
      return publisher.history == "keep_last" && publisher.depth == 5U;
    });
  const auto qos_status = !delivery_qos_valid || (queue_qos_available && !queue_qos_valid) ?
    Status::Fail : queue_qos_available ? Status::Pass : Status::Warn;
  std::map<std::string, std::string> qos_details{
    {"publishers", size_value(publishers.size())}};
  if (publishers.size() == 1U) {
    qos_details.emplace("actual_reliability", publishers.front().reliability);
    qos_details.emplace("actual_durability", publishers.front().durability);
    qos_details.emplace("actual_history", publishers.front().history);
    qos_details.emplace("actual_depth", size_value(publishers.front().depth));
  }
  record(
    "graph.qos." + topic, qos_status,
    "topic.qos", topic,
    qos_status == Status::Pass ? "Publisher QoS matches SensorDataQoS" :
    qos_status == Status::Warn ?
    "Publisher is best_effort/volatile, but this DDS graph does not expose history/depth; "
    "verify keep_last depth 5 with ros2 topic info -v" :
    "Publisher QoS must be SensorDataQoS: best_effort, volatile, keep_last, depth 5",
    std::move(qos_details));
}

void PreflightAnalyzer::observe_vision(const VisionSample & sample, double arrival_s)
{
  observe_common(kVisionTopic, &sample.stamp, arrival_s);
  const std::array<float, 6> scalars{{
    sample.yaw_degree, sample.yaw_vel_degree_s, sample.pitch_degree,
    sample.pitch_vel_degree_s, sample.roll_degree, sample.shoot_speed_m_s,
  }};
  const bool scalars_valid = std::all_of(
    scalars.begin(), scalars.end(), [](float value) {
      return std::isfinite(value);
    });
  record(
    "vision.scalars", scalars_valid ? Status::Pass : Status::Fail,
    "vision.scalar_finite", kVisionTopic,
    scalars_valid ? "Vision scalar fields are finite and retain their external units" :
    "Vision contains NaN or Inf scalar fields",
    {
      {"yaw", number(sample.yaw_degree) + " degree"},
      {"yaw_vel", number(sample.yaw_vel_degree_s) + " degree/s"},
      {"pitch", number(sample.pitch_degree) + " degree"},
      {"pitch_vel", number(sample.pitch_vel_degree_s) + " degree/s"},
      {"roll", number(sample.roll_degree) + " degree"},
      {"shoot_speed", number(sample.shoot_speed_m_s) + " m/s"},
    });

  if (!sample.yaw_acc_degree_s2.has_value() || !sample.pitch_acc_degree_s2.has_value()) {
    record(
      "vision.acceleration", Status::Warn, "vision.acceleration_finite", kVisionTopic,
      "Installed Vision interface has no acceleration field; degree/s^2 check is unavailable");
  } else {
    const bool acceleration_valid = std::isfinite(*sample.yaw_acc_degree_s2) &&
      std::isfinite(*sample.pitch_acc_degree_s2);
    record(
      "vision.acceleration", acceleration_valid ? Status::Pass : Status::Fail,
      "vision.acceleration_finite", kVisionTopic,
      acceleration_valid ? "Vision acceleration fields are finite and unchanged" :
      "Vision acceleration contains NaN or Inf",
      {
        {"yaw_acc", number(*sample.yaw_acc_degree_s2) + " degree/s^2"},
        {"pitch_acc", number(*sample.pitch_acc_degree_s2) + " degree/s^2"},
      });
  }

  if (std::isfinite(sample.yaw_degree)) {
    const bool yaw_valid = sample.yaw_degree >= -180.0F && sample.yaw_degree <= 180.0F;
    record(
      "vision.yaw_range", yaw_valid ? Status::Pass : Status::Fail,
      "vision.yaw_range", kVisionTopic,
      yaw_valid ? "yaw is within inclusive [-180 degree, 180 degree]" :
      "yaw is outside inclusive [-180 degree, 180 degree]",
      {{"yaw", number(sample.yaw_degree) + " degree"}});
  }

  const auto limits = pitch_limits(config_.vehicle_profile);
  if (!limits.has_value()) {
    record(
      "vision.pitch_range", Status::Warn, "vision.pitch_profile", kVisionTopic,
      "No vehicle profile selected: pitch range 无法判定",
      {{"vehicle_profile", config_.vehicle_profile}});
  } else if (std::isfinite(sample.pitch_degree)) {
    const bool pitch_valid = sample.pitch_degree >= (*limits)[0] &&
      sample.pitch_degree <= (*limits)[1];
    record(
      "vision.pitch_range", pitch_valid ? Status::Pass : Status::Fail,
      "vision.pitch_profile", kVisionTopic,
      pitch_valid ? "pitch is within the explicitly selected vehicle profile" :
      "pitch is outside the explicitly selected vehicle profile",
      {
        {"pitch", number(sample.pitch_degree) + " degree"},
        {"minimum", number((*limits)[0]) + " degree"},
        {"maximum", number((*limits)[1]) + " degree"},
        {"vehicle_profile", config_.vehicle_profile},
      });
  }

  const bool quaternion_valid = sample.quaternion_wxyz.size() == 4U &&
    finite_values(sample.quaternion_wxyz);
  record(
    "vision.quaternion", quaternion_valid ? Status::Pass : Status::Fail,
    "vision.quaternion_format", kVisionTopic,
    quaternion_valid ?
    "Quaternion has 4 finite wxyz entries; IMU relative to power-on origin; format only" :
    "Quaternion must have exactly 4 finite entries in declared wxyz order",
    {
      {"length", size_value(sample.quaternion_wxyz.size())},
      {"order", "wxyz"}, {"interpretation", "format_only"},
    });
}

void PreflightAnalyzer::record_callback_error(
  const std::string & topic, const std::string & reason)
{
  record(
    "callback." + topic, Status::Fail, "callback.exception", topic,
    "Message validation raised an exception: " + reason);
}

std::vector<Finding> PreflightAnalyzer::runtime_findings(double now_s) const
{
  std::vector<Finding> result;
  for (const auto & entry : stats_) {
    const auto & topic = entry.first;
    const auto & stats = entry.second;
    if (stats.count == 0U) {
      result.push_back(
        Finding{
          Status::Fail, "topic.received", topic,
          "No messages were received during the observation window", {{"count", "0"}}});
      continue;
    }
    result.push_back(
      Finding{
        Status::Pass, "topic.received", topic, "Messages were received",
        {{"count", size_value(stats.count)}}});

    if (stats.count < 2U || stats.first_arrival_s == stats.last_arrival_s) {
      result.push_back(
        Finding{
          Status::Warn, "topic.frequency", topic,
          "At least two arrivals are required to calculate frequency",
          {{"count", size_value(stats.count)}}});
    } else {
      const double span = *stats.last_arrival_s - *stats.first_arrival_s;
      const double hz = span > 0.0 ? static_cast<double>(stats.count - 1U) / span : 0.0;
      result.push_back(
        Finding{
          span > 0.0 ? Status::Pass : Status::Warn, "topic.frequency", topic,
          span > 0.0 ? "Frequency calculated from local monotonic arrival times" :
          "Receive frequency could not be calculated",
          {{"count", size_value(stats.count)}, {"hz", number(hz)}}});
    }

    if (!timed_topic(topic)) {
      continue;
    }
    const double age_s = std::max(0.0, now_s - *stats.last_arrival_s);
    const bool timed_out = age_s > config_.timeout_s;
    result.push_back(
      Finding{
        timed_out ? Status::Fail : Status::Pass, "topic.timeout", topic,
        timed_out ? "Last message exceeds the configured receive timeout" :
        "Last message is within the configured receive timeout",
        {{"age_s", number(age_s)}, {"timeout_s", number(config_.timeout_s)}}});

    Status stamp_status = Status::Pass;
    std::string stamp_reason = "Header timestamps are strictly increasing";
    if (stats.invalid_timestamps > 0U) {
      stamp_status = Status::Fail;
      stamp_reason = "One or more Header timestamps were not canonical";
    } else if (stats.timestamp_rollbacks > 0U) {
      stamp_status = Status::Fail;
      stamp_reason = "Header timestamp moved backwards";
    } else if (stats.unset_timestamps > 0U) {
      stamp_status = topic == kVisionTopic ? Status::Warn : Status::Fail;
      stamp_reason = topic == kVisionTopic ?
        "One or more Vision Header timestamps were zero/unset" :
        "Image and CameraInfo Header timestamps must be non-zero";
    } else if (stats.duplicate_timestamps > 0U) {
      stamp_status = Status::Warn;
      stamp_reason = "Duplicate Header timestamps were observed";
    } else if (stats.count < 2U) {
      stamp_status = Status::Warn;
      stamp_reason = "At least two messages are required to establish monotonicity";
    }
    result.push_back(
      Finding{
        stamp_status, "header.timestamp_monotonic", topic, stamp_reason,
        {
          {"rollbacks", size_value(stats.timestamp_rollbacks)},
          {"duplicates", size_value(stats.duplicate_timestamps)},
          {"invalid", size_value(stats.invalid_timestamps)},
          {"unset", size_value(stats.unset_timestamps)},
        }});
  }
  return result;
}

std::vector<Finding> PreflightAnalyzer::relationship_findings(double now_s) const
{
  std::vector<Finding> result;
  const auto image_count = stats_.at(kImageTopic).count;
  const auto camera_info_count = stats_.at(kCameraInfoTopic).count;
  constexpr double pairing_delivery_grace_s = 0.1;
  const auto stale_count = [&](const auto & unmatched) {
      return static_cast<std::size_t>(std::count_if(
               unmatched.begin(), unmatched.end(), [&](const auto & entry) {
                 return now_s - entry.second.arrival_s > pairing_delivery_grace_s;
               }));
    };
  const auto stale_images = stale_count(unmatched_images_);
  const auto stale_camera_info = stale_count(unmatched_camera_info_);
  const auto unmatched_count = unmatched_images_.size() + unmatched_camera_info_.size();
  const auto unmatched_is_final_tail = [&]() {
      if (unmatched_count == 0U) {
        return true;
      }
      if (unmatched_count != 1U || stale_images != 0U || stale_camera_info != 0U ||
        !last_matched_stamp_ns_.has_value())
      {
        return false;
      }
      const auto unmatched_stamp = !unmatched_images_.empty() ?
        unmatched_images_.begin()->first : unmatched_camera_info_.begin()->first;
      return unmatched_stamp > *last_matched_stamp_ns_;
    }();
  const auto & image_stats = stats_.at(kImageTopic);
  const auto & camera_stats = stats_.at(kCameraInfoTopic);
  const bool timestamps_well_formed =
    image_stats.invalid_timestamps == 0U && image_stats.unset_timestamps == 0U &&
    camera_stats.invalid_timestamps == 0U && camera_stats.unset_timestamps == 0U;
  const bool base_pairing_valid = matched_pairs_ > 0U && timestamps_well_formed;
  const auto pairing_status = !base_pairing_valid || !unmatched_is_final_tail ?
    Status::Fail : unmatched_count == 0U ? Status::Pass : Status::Warn;
  result.push_back(
    Finding{
      pairing_status,
      "image_camera.timestamp_pairing", "cross_topic",
      pairing_status == Status::Pass ?
      "Every Image has a CameraInfo with the exact same non-zero Header timestamp" :
      pairing_status == Status::Warn ?
      "One final message is still within the DDS delivery grace; repeat or extend observation" :
      "Image and CameraInfo require one-to-one exact non-zero Header timestamp pairing",
      {
        {"images", size_value(image_count)},
        {"camera_info", size_value(camera_info_count)},
        {"matched", size_value(matched_pairs_)},
        {"unmatched_images", size_value(unmatched_images_.size())},
        {"unmatched_camera_info", size_value(unmatched_camera_info_.size())},
        {"stale_unmatched_images", size_value(stale_images)},
        {"stale_unmatched_camera_info", size_value(stale_camera_info)},
        {"unmatched_is_final_tail", unmatched_is_final_tail ? "true" : "false"},
        {"delivery_grace_ms", "100"},
      }});

  const bool dimensions_match = matched_pairs_ > 0U && dimension_mismatches_ == 0U;
  result.push_back(
    Finding{
      dimensions_match ? Status::Pass : Status::Fail,
      "image_camera.dimensions", "cross_topic",
      dimensions_match ? "Paired Image and CameraInfo dimensions match" :
      "A timestamp-paired Image and CameraInfo have different dimensions or no pair exists",
      {{"matched", size_value(matched_pairs_)},
        {"mismatches", size_value(dimension_mismatches_)}}});

  const bool frame_ids_match = matched_pairs_ > 0U && frame_id_mismatches_ == 0U;
  result.push_back(
    Finding{
      frame_ids_match ? Status::Pass : Status::Fail,
      "image_camera.frame_id", "cross_topic",
      frame_ids_match ? "Paired Image and CameraInfo frame_id values match" :
      "A timestamp-paired Image and CameraInfo have different frame_id values or no pair exists",
      {{"matched", size_value(matched_pairs_)},
        {"mismatches", size_value(frame_id_mismatches_)}}});

  const auto image_stamp = stats_.at(kImageTopic).last_stamp_ns;
  const auto vision_stamp = stats_.at(kVisionTopic).last_stamp_ns;
  if (!config_.shared_clock_domain) {
    result.push_back(
      Finding{
        Status::Warn, "image_vision.clock_domain", "cross_topic",
        "Shared clock domain not declared: 时间基准未确认; timestamps were not compared",
        {{"compared", "false"}}});
  } else if (!image_stamp.has_value() || !vision_stamp.has_value()) {
    result.push_back(
      Finding{
        Status::Warn, "image_vision.clock_domain", "cross_topic",
        "Shared clock declared, but both valid timestamps are unavailable",
        {{"compared", "false"}}});
  } else {
    const double delta_ms = std::abs(static_cast<double>(*image_stamp - *vision_stamp)) / 1e6;
    const bool within = delta_ms <= config_.sync_tolerance_ms;
    result.push_back(
      Finding{
        within ? Status::Pass : Status::Warn, "image_vision.timestamp_delta", "cross_topic",
        within ? "Timestamp delta is within the diagnostic tolerance" :
        "Timestamp delta exceeds the diagnostic tolerance",
        {
          {"compared", "true"}, {"delta_ms", number(delta_ms)},
          {"tolerance_ms", number(config_.sync_tolerance_ms)},
          {"synchronization_inferred", "false"},
        }});
  }
  return result;
}

Report PreflightAnalyzer::build_report(double now_s) const
{
  Report report;
  report.config = config_;
  report.observation_duration_s = std::max(0.0, now_s - start_s_);
  report.findings.push_back(
    Finding{
      Status::Pass, "safety.read_only", "node",
      "No control publisher, serial access, camera access, or message mutation",
      {{"subscriptions", "/image_raw,/camera_info,/Vision_data"},
        {"forbidden_topic", "/Robot_ctrl_data"}}});
  for (const auto & entry : findings_) {
    report.findings.push_back(entry.second);
  }
  const auto runtime = runtime_findings(now_s);
  report.findings.insert(report.findings.end(), runtime.begin(), runtime.end());
  const auto relationships = relationship_findings(now_s);
  report.findings.insert(report.findings.end(), relationships.begin(), relationships.end());

  report.overall = Status::Pass;
  for (const auto & finding : report.findings) {
    ++report.counts[status_index(finding.status)];
    if (finding.status > report.overall) {
      report.overall = finding.status;
    }
  }
  return report;
}

std::string format_report_text(const Report & report)
{
  std::ostringstream stream;
  stream << "ROS 2 INPUT PREFLIGHT: " << status_name(report.overall) << '\n';
  stream << "configuration: timeout=" << report.config.timeout_s
         << "s, vehicle_profile=" << report.config.vehicle_profile
         << ", shared_clock_domain=" << std::boolalpha
         << report.config.shared_clock_domain
         << ", expected_frame_id=" << report.config.expected_frame_id << '\n';
  for (const auto & finding : report.findings) {
    stream << '[' << status_name(finding.status) << "] " << finding.topic << ' '
           << finding.check << ": " << finding.reason;
    if (!finding.details.empty()) {
      stream << " | ";
      bool first = true;
      for (const auto & detail : finding.details) {
        if (!first) {
          stream << ", ";
        }
        first = false;
        stream << detail.first << '=' << detail.second;
      }
    }
    stream << '\n';
  }
  stream << "summary: PASS=" << report.counts[status_index(Status::Pass)]
         << ", WARN=" << report.counts[status_index(Status::Warn)]
         << ", FAIL=" << report.counts[status_index(Status::Fail)] << '\n';
  return stream.str();
}

std::string format_report_json(const Report & report)
{
  std::ostringstream stream;
  stream << "{\n  \"schema_version\": 1,\n"
         << "  \"tool\": \"ros_input_preflight\",\n"
         << "  \"overall\": \"" << status_name(report.overall) << "\",\n"
         << "  \"observation_duration_s\": " << report.observation_duration_s << ",\n"
         << "  \"counts\": {\"PASS\": " << report.counts[0]
         << ", \"WARN\": " << report.counts[1]
         << ", \"FAIL\": " << report.counts[2] << "},\n"
         << "  \"configuration\": {\"timeout_s\": " << report.config.timeout_s
         << ", \"vehicle_profile\": \"" << json_escape(report.config.vehicle_profile)
         << "\", \"shared_clock_domain_declared\": "
         << (report.config.shared_clock_domain ? "true" : "false")
         << ", \"sync_tolerance_ms\": " << report.config.sync_tolerance_ms
         << ", \"expected_frame_id\": \""
         << json_escape(report.config.expected_frame_id) << "\"},\n"
         << "  \"findings\": [\n";
  for (std::size_t index = 0; index < report.findings.size(); ++index) {
    const auto & finding = report.findings[index];
    stream << "    {\"status\": \"" << status_name(finding.status)
           << "\", \"check\": \"" << json_escape(finding.check)
           << "\", \"topic\": \"" << json_escape(finding.topic)
           << "\", \"reason\": \"" << json_escape(finding.reason)
           << "\", \"details\": {";
    std::size_t detail_index = 0U;
    for (const auto & detail : finding.details) {
      if (detail_index++ > 0U) {
        stream << ", ";
      }
      stream << '"' << json_escape(detail.first) << "\": \""
             << json_escape(detail.second) << '"';
    }
    stream << "}}" << (index + 1U == report.findings.size() ? "\n" : ",\n");
  }
  stream << "  ]\n}\n";
  return stream.str();
}

double monotonic_seconds()
{
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

}  // namespace auto_aim_tools
