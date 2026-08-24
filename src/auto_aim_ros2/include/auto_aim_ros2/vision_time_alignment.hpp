#ifndef AUTO_AIM_ROS2__VISION_TIME_ALIGNMENT_HPP_
#define AUTO_AIM_ROS2__VISION_TIME_ALIGNMENT_HPP_

#include "auto_aim_ros2/auto_aim_core.hpp"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>

namespace rm_auto_aim::vision_time_alignment
{

// A timestamp value is only comparable with another value from the same
// clock/domain.  In particular, a ROS image header and a ROS Vision header
// are intentionally different domains until the system integration proves
// that they share a clock.
enum class TimestampDomain : std::uint8_t
{
  Unknown = 0,
  ImageHeader,
  VisionHeader,
  SharedRosHeader,
  SteadyClock,
  Mcu,
};

const char * timestamp_domain_name(TimestampDomain domain) noexcept;

enum class PairStatus : std::uint8_t
{
  Matched = 0,
  Missing,
  Stale,
  Future,
  Invalid,
  Incomparable,
  Unconfigured,
};

const char * pair_status_name(PairStatus status) noexcept;

enum class InsertStatus : std::uint8_t
{
  Accepted = 0,
  Invalid,
  TimestampRollback,
  DuplicateTimestamp,
  Incomparable,
};

const char * insert_status_name(InsertStatus status) noexcept;

// More specific information for a rejected insertion.  Pairing still uses
// PairStatus::Invalid as the fail-closed result; this enum is for diagnostics.
enum class InsertReason : std::uint8_t
{
  None = 0,
  ZeroOrNegativeTimestamp,
  UnknownTimestampDomain,
  NonFiniteState,
  InvalidQuaternion,
  CapacityZero,
  TimestampRollback,
  DuplicateTimestamp,
  DomainMismatch,
};

const char * insert_reason_name(InsertReason reason) noexcept;

struct VisionStateSample
{
  pipeline::VisionState state{};
  TimestampDomain timestamp_domain{TimestampDomain::Unknown};
};

struct InsertResult
{
  InsertStatus status{InsertStatus::Invalid};
  InsertReason reason{InsertReason::None};
  std::size_t size{0};

  bool accepted() const noexcept
  {
    return status == InsertStatus::Accepted;
  }
};

struct PairConfig
{
  // A null value is deliberate: no control-affecting association is allowed
  // until an explicit tolerance is supplied by the integration layer.
  std::optional<std::int64_t> tolerance_ns{};
  // Future Vision samples are not used by default.  Enabling this requires a
  // separately reviewed clock/latency contract.
  bool allow_future{false};
};

struct PairResult
{
  PairStatus status{PairStatus::Invalid};
  std::optional<VisionStateSample> sample{};
  std::int64_t image_stamp_ns{0};
  std::int64_t matched_stamp_ns{0};
  // image_stamp_ns - matched_stamp_ns.  Positive means a past sample; a
  // negative value is only possible when PairConfig::allow_future is true.
  std::int64_t delta_ns{0};

  bool matched() const noexcept
  {
    return status == PairStatus::Matched && sample.has_value();
  }
};

// Fixed-capacity, append-only (by timestamp) history of Vision state samples.
// This class deliberately performs no angle conversion, coordinate transform,
// quaternion interpretation, integration, or motion compensation.
class VisionStateHistory final
{
public:
  explicit VisionStateHistory(
    std::size_t capacity = 32,
    TimestampDomain expected_domain = TimestampDomain::Unknown);

  InsertResult insert(const pipeline::VisionState & state, TimestampDomain domain);
  InsertResult insert(const VisionStateSample & sample);

  // Alias useful at a ROS callback boundary; both operations have identical
  // validation and monotonic-timestamp semantics.
  InsertResult push(const pipeline::VisionState & state, TimestampDomain domain)
  {
    return insert(state, domain);
  }

  InsertResult push(const VisionStateSample & sample)
  {
    return insert(sample);
  }

  PairResult pair(
    std::int64_t image_stamp_ns,
    TimestampDomain image_domain,
    const PairConfig & config = {}) const;

  std::size_t size() const noexcept;
  std::size_t capacity() const noexcept;
  bool empty() const noexcept;
  TimestampDomain timestamp_domain() const noexcept;
  const VisionStateSample * oldest() const noexcept;
  const VisionStateSample * latest() const noexcept;

  // Clear samples while retaining an explicitly configured expected domain.
  // If the domain was inferred from the first sample, it is inferred again.
  void clear() noexcept;

private:
  static bool valid_state(const pipeline::VisionState & state) noexcept;

  std::size_t capacity_{0};
  TimestampDomain expected_domain_{TimestampDomain::Unknown};
  TimestampDomain timestamp_domain_{TimestampDomain::Unknown};
  std::deque<VisionStateSample> samples_{};
};

}  // namespace rm_auto_aim::vision_time_alignment

#endif  // AUTO_AIM_ROS2__VISION_TIME_ALIGNMENT_HPP_
