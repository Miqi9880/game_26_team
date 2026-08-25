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
  return topic == kImageTopic || topic == kVisionTopic;
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
  static const std::map<std::string, std::size_t> fixed_encodings{
    {"mono8", 1U}, {"mono16", 2U}, {"bgr8", 3U}, {"rgb8", 3U},
    {"bgra8", 4U}, {"rgba8", 4U}, {"bayer_rggb8", 1U},
    {"bayer_bggr8", 1U}, {"bayer_gbrg8", 1U}, {"bayer_grbg8", 1U},
    {"bayer_rggb16", 2U}, {"bayer_bggr16", 2U}, {"bayer_gbrg16", 2U},
    {"bayer_grbg16", 2U}, {"yuv422", 2U}, {"yuyv", 2U}, {"uyvy", 2U},
  };
  const auto fixed = fixed_encodings.find(lowercase(encoding));
  if (fixed != fixed_encodings.end()) {
    return fixed->second;
  }

  static const std::regex typed_encoding(R"(^([0-9]+)(U|S|F)C([1-9][0-9]*)$)");
  std::smatch match;
  const auto canonical = uppercase(encoding);
  if (!std::regex_match(canonical, match, typed_encoding)) {
    return std::nullopt;
  }
  const auto bits = std::stoul(match[1].str());
  const auto channels = std::stoul(match[3].str());
  if ((bits != 8U && bits != 16U && bits != 32U && bits != 64U) ||
    channels > std::numeric_limits<std::size_t>::max() / (bits / 8U))
  {
    return std::nullopt;
  }
  return channels * (bits / 8U);
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

void PreflightAnalyzer::observe_image(const ImageSample & sample, double arrival_s)
{
  observe_common(kImageTopic, &sample.stamp, arrival_s);
  const bool dimensions_valid = sample.width > 0U && sample.height > 0U;
  record(
    "image.dimensions", dimensions_valid ? Status::Pass : Status::Fail,
    "image.dimensions", kImageTopic,
    dimensions_valid ? "Image dimensions are positive" :
    "Image width and height must both be positive",
    {{"width", std::to_string(sample.width)}, {"height", std::to_string(sample.height)}});
  if (dimensions_valid) {
    latest_image_size_ = std::array<std::uint32_t, 2>{{sample.width, sample.height}};
  }

  const auto pixel_bytes = bytes_per_pixel(sample.encoding);
  if (sample.encoding.empty()) {
    record(
      "image.encoding", Status::Fail, "image.encoding", kImageTopic,
      "Image encoding is empty");
  } else if (!pixel_bytes.has_value()) {
    record(
      "image.encoding", Status::Warn, "image.encoding", kImageTopic,
      "Encoding byte width is unknown; minimum row width cannot be checked",
      {{"encoding", sample.encoding}});
  } else {
    record(
      "image.encoding", Status::Pass, "image.encoding", kImageTopic,
      "Encoding has a known byte width",
      {{"encoding", sample.encoding}, {"bytes_per_pixel", size_value(*pixel_bytes)}});
  }

  bool step_valid = sample.step > 0U;
  std::uint64_t minimum_step = 0U;
  if (pixel_bytes.has_value() && sample.width > 0U) {
    minimum_step = static_cast<std::uint64_t>(sample.width) * *pixel_bytes;
    step_valid = step_valid && sample.step >= minimum_step;
  }
  record(
    "image.step", step_valid ? Status::Pass : Status::Fail, "image.step", kImageTopic,
    step_valid ? "Image step is structurally valid" :
    "Image step must be positive and cover one encoded row",
    {{"step", std::to_string(sample.step)}, {"minimum_step", integer(minimum_step)}});

  const auto expected = static_cast<std::uint64_t>(sample.step) * sample.height;
  const bool length_valid = expected > 0U && sample.data_size == expected;
  record(
    "image.data_length", length_valid ? Status::Pass : Status::Fail,
    "image.data_length", kImageTopic,
    length_valid ? "Image data length matches step times height" :
    "Image data must be non-empty and equal step times height",
    {{"data_length", size_value(sample.data_size)}, {"expected_length", integer(expected)}});
}

void PreflightAnalyzer::observe_camera_info(
  const CameraInfoSample & sample, double arrival_s)
{
  observe_common(kCameraInfoTopic, nullptr, arrival_s);
  const bool dimensions_valid = sample.width > 0U && sample.height > 0U;
  record(
    "camera.dimensions", dimensions_valid ? Status::Pass : Status::Fail,
    "camera_info.dimensions", kCameraInfoTopic,
    dimensions_valid ? "CameraInfo dimensions are positive" :
    "CameraInfo width and height must both be positive",
    {{"width", std::to_string(sample.width)}, {"height", std::to_string(sample.height)}});
  if (dimensions_valid) {
    latest_camera_size_ = std::array<std::uint32_t, 2>{{sample.width, sample.height}};
  }

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
      d_status = Status::Warn;
      d_reason = "Unknown distortion_model; D is finite but its length is unverified";
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
      stamp_status = Status::Warn;
      stamp_reason = "One or more Header timestamps were zero/unset";
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

std::vector<Finding> PreflightAnalyzer::relationship_findings() const
{
  std::vector<Finding> result;
  if (latest_image_size_.has_value() && latest_camera_size_.has_value()) {
    const bool match = latest_image_size_ == latest_camera_size_;
    result.push_back(
      Finding{
        match ? Status::Pass : Status::Fail, "image_camera.dimensions", "cross_topic",
        match ? "Image and CameraInfo dimensions match" :
        "Image and CameraInfo dimensions do not match",
        {
          {"image", std::to_string((*latest_image_size_)[0]) + "x" +
            std::to_string((*latest_image_size_)[1])},
          {"camera_info", std::to_string((*latest_camera_size_)[0]) + "x" +
            std::to_string((*latest_camera_size_)[1])},
        }});
  }

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
  const auto relationships = relationship_findings();
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
         << report.config.shared_clock_domain << '\n';
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
         << ", \"sync_tolerance_ms\": " << report.config.sync_tolerance_ms << "},\n"
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
