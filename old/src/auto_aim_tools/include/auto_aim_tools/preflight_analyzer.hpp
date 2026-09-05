#ifndef AUTO_AIM_TOOLS__PREFLIGHT_ANALYZER_HPP_
#define AUTO_AIM_TOOLS__PREFLIGHT_ANALYZER_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace auto_aim_tools
{

inline constexpr const char * kImageTopic = "/image_raw";
inline constexpr const char * kCameraInfoTopic = "/camera_info";
inline constexpr const char * kVisionTopic = "/Vision_data";
inline constexpr const char * kExpectedImageType = "sensor_msgs/msg/Image";
inline constexpr const char * kExpectedCameraInfoType = "sensor_msgs/msg/CameraInfo";
inline constexpr const char * kExpectedVisionType = "auto_aim_interfaces/msg/Vision";
inline constexpr const char * kExpectedImageEncoding = "rgb8";

enum class Status : std::uint8_t
{
  Pass = 0,
  Warn,
  Fail,
};

struct Finding
{
  Status status{Status::Warn};
  std::string check;
  std::string topic;
  std::string reason;
  std::map<std::string, std::string> details;
};

struct HeaderStamp
{
  std::int64_t sec{0};
  std::uint32_t nanosec{0};
  bool readable{true};
};

struct ImageSample
{
  HeaderStamp stamp;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string encoding;
  std::uint32_t step{0};
  std::size_t data_size{0};
};

struct CameraInfoSample
{
  HeaderStamp stamp;
  std::string frame_id;
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::string distortion_model;
  std::vector<double> k;
  std::vector<double> d;
};

struct PublisherEndpointSample
{
  std::string topic_type;
  std::string reliability;
  std::string durability;
  std::string history;
  std::size_t depth{0U};
};

struct VisionSample
{
  HeaderStamp stamp;
  float yaw_degree{0.0F};
  float yaw_vel_degree_s{0.0F};
  std::optional<float> yaw_acc_degree_s2;
  float pitch_degree{0.0F};
  float pitch_vel_degree_s{0.0F};
  std::optional<float> pitch_acc_degree_s2;
  float roll_degree{0.0F};
  std::vector<float> quaternion_wxyz;
  float shoot_speed_m_s{0.0F};
};

struct PreflightConfig
{
  double timeout_s{1.0};
  std::string vehicle_profile{"unselected"};
  bool shared_clock_domain{false};
  double sync_tolerance_ms{50.0};
  std::string expected_frame_id{"camera_optical_frame"};
};

struct Report
{
  Status overall{Status::Warn};
  double observation_duration_s{0.0};
  PreflightConfig config;
  std::vector<Finding> findings;
  std::array<std::size_t, 3> counts{{0U, 0U, 0U}};
};

class PreflightAnalyzer
{
public:
  explicit PreflightAnalyzer(PreflightConfig config = {}, double start_s = 0.0);

  void observe_image(const ImageSample & sample, double arrival_s);
  void observe_camera_info(const CameraInfoSample & sample, double arrival_s);
  void observe_vision(const VisionSample & sample, double arrival_s);
  void observe_topic_publishers(
    const std::string & topic, const std::string & expected_type,
    const std::vector<PublisherEndpointSample> & publishers);
  void record_callback_error(const std::string & topic, const std::string & reason);

  Report build_report(double now_s) const;
  const PreflightConfig & config() const noexcept;

private:
  struct TopicStats
  {
    std::size_t count{0U};
    std::optional<double> first_arrival_s;
    std::optional<double> last_arrival_s;
    std::optional<std::int64_t> last_stamp_ns;
    std::size_t timestamp_rollbacks{0U};
    std::size_t duplicate_timestamps{0U};
    std::size_t invalid_timestamps{0U};
    std::size_t unset_timestamps{0U};
  };

  struct FrameMetadata
  {
    std::string frame_id;
    std::uint32_t width{0U};
    std::uint32_t height{0U};
    double arrival_s{0.0};
  };

  void record(
    const std::string & key, Status status, const std::string & check,
    const std::string & topic, const std::string & reason,
    std::map<std::string, std::string> details = {});
  void observe_common(
    const std::string & topic, const HeaderStamp * stamp, double arrival_s);
  void pair_frame(
    bool image, const HeaderStamp & stamp, const FrameMetadata & metadata);
  void validate_pair(
    const FrameMetadata & image, const FrameMetadata & camera_info);
  std::vector<Finding> runtime_findings(double now_s) const;
  std::vector<Finding> relationship_findings(double now_s) const;

  PreflightConfig config_;
  double start_s_{0.0};
  std::map<std::string, TopicStats> stats_;
  std::map<std::string, Finding> findings_;
  std::multimap<std::int64_t, FrameMetadata> unmatched_images_;
  std::multimap<std::int64_t, FrameMetadata> unmatched_camera_info_;
  std::optional<std::int64_t> last_matched_stamp_ns_;
  std::size_t matched_pairs_{0U};
  std::size_t dimension_mismatches_{0U};
  std::size_t frame_id_mismatches_{0U};
};

const char * status_name(Status status) noexcept;
std::optional<std::size_t> bytes_per_pixel(const std::string & encoding);
std::string format_report_text(const Report & report);
std::string format_report_json(const Report & report);
double monotonic_seconds();

}  // namespace auto_aim_tools

#endif  // AUTO_AIM_TOOLS__PREFLIGHT_ANALYZER_HPP_
