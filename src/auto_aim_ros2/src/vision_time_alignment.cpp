#include "auto_aim_ros2/vision_time_alignment.hpp"

#include <cmath>
#include <limits>

namespace rm_auto_aim::vision_time_alignment
{
namespace
{
constexpr float kQuaternionNormSquaredEpsilon = 1.0e-12F;

bool known_domain(TimestampDomain domain) noexcept
{
  return domain != TimestampDomain::Unknown;
}

std::int64_t absolute_delta(std::int64_t lhs, std::int64_t rhs) noexcept
{
  // Both inputs are positive in all calls from pair().  Subtract the smaller
  // value first so the operation cannot overflow before taking the absolute
  // value.
  return lhs >= rhs ? lhs - rhs : rhs - lhs;
}

PairResult make_pair_result(
  PairStatus status,
  std::int64_t image_stamp_ns,
  const VisionStateSample * sample = nullptr)
{
  PairResult result{};
  result.status = status;
  result.image_stamp_ns = image_stamp_ns;
  if (sample != nullptr) {
    // Keep the candidate timestamp available for diagnostics even when the
    // candidate is stale or future and therefore is not exposed as sample.
    result.matched_stamp_ns = sample->state.stamp_ns;
    result.delta_ns = image_stamp_ns - sample->state.stamp_ns;
    // A non-matched result is diagnostic only.  Do not expose a stale or
    // future state as a control candidate to a caller that forgets to inspect
    // the status first.
    if (status == PairStatus::Matched) {
      result.sample = *sample;
    }
  }
  return result;
}
}  // namespace

const char * timestamp_domain_name(TimestampDomain domain) noexcept
{
  switch (domain) {
    case TimestampDomain::Unknown:
      return "unknown";
    case TimestampDomain::ImageHeader:
      return "image_header";
    case TimestampDomain::VisionHeader:
      return "vision_header";
    case TimestampDomain::SharedRosHeader:
      return "shared_ros_header";
    case TimestampDomain::SteadyClock:
      return "steady_clock";
    case TimestampDomain::Mcu:
      return "mcu";
  }
  return "unknown";
}

const char * pair_status_name(PairStatus status) noexcept
{
  switch (status) {
    case PairStatus::Matched:
      return "matched";
    case PairStatus::Missing:
      return "missing";
    case PairStatus::Stale:
      return "stale";
    case PairStatus::Future:
      return "future";
    case PairStatus::Invalid:
      return "invalid";
    case PairStatus::Incomparable:
      return "incomparable";
    case PairStatus::Unconfigured:
      return "unconfigured";
  }
  return "invalid";
}

const char * insert_status_name(InsertStatus status) noexcept
{
  switch (status) {
    case InsertStatus::Accepted:
      return "accepted";
    case InsertStatus::Invalid:
      return "invalid";
    case InsertStatus::TimestampRollback:
      return "timestamp_rollback";
    case InsertStatus::DuplicateTimestamp:
      return "duplicate_timestamp";
    case InsertStatus::Incomparable:
      return "incomparable";
  }
  return "invalid";
}

const char * insert_reason_name(InsertReason reason) noexcept
{
  switch (reason) {
    case InsertReason::None:
      return "none";
    case InsertReason::ZeroOrNegativeTimestamp:
      return "zero_or_negative_timestamp";
    case InsertReason::UnknownTimestampDomain:
      return "unknown_timestamp_domain";
    case InsertReason::NonFiniteState:
      return "non_finite_state";
    case InsertReason::InvalidQuaternion:
      return "invalid_quaternion";
    case InsertReason::CapacityZero:
      return "capacity_zero";
    case InsertReason::TimestampRollback:
      return "timestamp_rollback";
    case InsertReason::DuplicateTimestamp:
      return "duplicate_timestamp";
    case InsertReason::DomainMismatch:
      return "domain_mismatch";
  }
  return "none";
}

VisionStateHistory::VisionStateHistory(
  std::size_t capacity, TimestampDomain expected_domain)
: capacity_(capacity),
  expected_domain_(expected_domain),
  timestamp_domain_(expected_domain)
{
}

bool VisionStateHistory::valid_state(const pipeline::VisionState & state) noexcept
{
  if (state.stamp_ns <= 0) {
    return false;
  }

  if (!std::isfinite(state.yaw_rad) || !std::isfinite(state.pitch_rad) ||
    !std::isfinite(state.roll_rad) || !std::isfinite(state.shoot_speed_mps))
  {
    return false;
  }

  float quaternion_norm_squared = 0.0F;
  for (const auto value : state.quaternion_wxyz) {
    if (!std::isfinite(value)) {
      return false;
    }
    quaternion_norm_squared += value * value;
  }
  return std::isfinite(quaternion_norm_squared) &&
         quaternion_norm_squared > kQuaternionNormSquaredEpsilon;
}

InsertResult VisionStateHistory::insert(
  const pipeline::VisionState & state, TimestampDomain domain)
{
  return insert(VisionStateSample{state, domain});
}

InsertResult VisionStateHistory::insert(const VisionStateSample & sample)
{
  InsertResult result{};
  result.size = samples_.size();

  if (capacity_ == 0U) {
    result.status = InsertStatus::Invalid;
    result.reason = InsertReason::CapacityZero;
    return result;
  }
  if (sample.state.stamp_ns <= 0) {
    result.status = InsertStatus::Invalid;
    result.reason = InsertReason::ZeroOrNegativeTimestamp;
    return result;
  }
  if (!known_domain(sample.timestamp_domain)) {
    result.status = InsertStatus::Invalid;
    result.reason = InsertReason::UnknownTimestampDomain;
    return result;
  }
  if (!valid_state(sample.state)) {
    result.status = InsertStatus::Invalid;
    result.reason = (std::isfinite(sample.state.quaternion_wxyz[0]) &&
      std::isfinite(sample.state.quaternion_wxyz[1]) &&
      std::isfinite(sample.state.quaternion_wxyz[2]) &&
      std::isfinite(sample.state.quaternion_wxyz[3])) ?
      InsertReason::NonFiniteState : InsertReason::InvalidQuaternion;
    // A zero quaternion is finite but invalid too.  Keep this reason stable
    // for diagnostics instead of accepting or normalizing it.
    float norm_squared = 0.0F;
    for (const auto value : sample.state.quaternion_wxyz) {
      if (std::isfinite(value)) {
        norm_squared += value * value;
      }
    }
    if (!std::isfinite(norm_squared) || norm_squared <= kQuaternionNormSquaredEpsilon) {
      result.reason = InsertReason::InvalidQuaternion;
    }
    return result;
  }

  if (expected_domain_ != TimestampDomain::Unknown &&
    sample.timestamp_domain != expected_domain_)
  {
    result.status = InsertStatus::Incomparable;
    result.reason = InsertReason::DomainMismatch;
    return result;
  }
  if (timestamp_domain_ != TimestampDomain::Unknown &&
    sample.timestamp_domain != timestamp_domain_)
  {
    result.status = InsertStatus::Incomparable;
    result.reason = InsertReason::DomainMismatch;
    return result;
  }

  if (!samples_.empty()) {
    const auto latest_stamp = samples_.back().state.stamp_ns;
    if (sample.state.stamp_ns == latest_stamp) {
      result.status = InsertStatus::DuplicateTimestamp;
      result.reason = InsertReason::DuplicateTimestamp;
      return result;
    }
    if (sample.state.stamp_ns < latest_stamp) {
      result.status = InsertStatus::TimestampRollback;
      result.reason = InsertReason::TimestampRollback;
      return result;
    }
  }

  if (timestamp_domain_ == TimestampDomain::Unknown) {
    timestamp_domain_ = sample.timestamp_domain;
  }
  if (samples_.size() == capacity_) {
    samples_.pop_front();
  }
  samples_.push_back(sample);
  result.status = InsertStatus::Accepted;
  result.reason = InsertReason::None;
  result.size = samples_.size();
  return result;
}

PairResult VisionStateHistory::pair(
  std::int64_t image_stamp_ns,
  TimestampDomain image_domain,
  const PairConfig & config) const
{
  if (image_stamp_ns <= 0 || !known_domain(image_domain)) {
    return make_pair_result(PairStatus::Invalid, image_stamp_ns);
  }
  if (samples_.empty()) {
    return make_pair_result(PairStatus::Missing, image_stamp_ns);
  }
  if (!known_domain(timestamp_domain_) || image_domain != timestamp_domain_) {
    return make_pair_result(PairStatus::Incomparable, image_stamp_ns);
  }
  if (!config.tolerance_ns.has_value()) {
    return make_pair_result(PairStatus::Unconfigured, image_stamp_ns);
  }
  const auto tolerance_ns = config.tolerance_ns.value();
  if (tolerance_ns < 0) {
    return make_pair_result(PairStatus::Invalid, image_stamp_ns);
  }

  const VisionStateSample * latest_past = nullptr;
  const VisionStateSample * earliest_future = nullptr;
  for (const auto & sample : samples_) {
    const auto stamp_ns = sample.state.stamp_ns;
    if (stamp_ns <= image_stamp_ns) {
      latest_past = &sample;
    } else if (earliest_future == nullptr) {
      earliest_future = &sample;
    }
  }

  if (!config.allow_future) {
    if (latest_past == nullptr) {
      return make_pair_result(PairStatus::Future, image_stamp_ns, earliest_future);
    }
    const auto delta_ns = image_stamp_ns - latest_past->state.stamp_ns;
    if (delta_ns <= tolerance_ns) {
      return make_pair_result(PairStatus::Matched, image_stamp_ns, latest_past);
    }
    return make_pair_result(PairStatus::Stale, image_stamp_ns, latest_past);
  }

  const VisionStateSample * nearest = nullptr;
  std::int64_t nearest_delta_ns = std::numeric_limits<std::int64_t>::max();
  for (const auto & sample : samples_) {
    const auto delta_ns = absolute_delta(image_stamp_ns, sample.state.stamp_ns);
    if (delta_ns < nearest_delta_ns) {
      nearest = &sample;
      nearest_delta_ns = delta_ns;
    }
  }
  if (nearest == nullptr) {
    return make_pair_result(PairStatus::Missing, image_stamp_ns);
  }
  if (nearest_delta_ns <= tolerance_ns) {
    return make_pair_result(PairStatus::Matched, image_stamp_ns, nearest);
  }
  return make_pair_result(
    nearest->state.stamp_ns > image_stamp_ns ? PairStatus::Future : PairStatus::Stale,
    image_stamp_ns,
    nearest);
}

std::size_t VisionStateHistory::size() const noexcept
{
  return samples_.size();
}

std::size_t VisionStateHistory::capacity() const noexcept
{
  return capacity_;
}

bool VisionStateHistory::empty() const noexcept
{
  return samples_.empty();
}

TimestampDomain VisionStateHistory::timestamp_domain() const noexcept
{
  return timestamp_domain_;
}

const VisionStateSample * VisionStateHistory::oldest() const noexcept
{
  return samples_.empty() ? nullptr : &samples_.front();
}

const VisionStateSample * VisionStateHistory::latest() const noexcept
{
  return samples_.empty() ? nullptr : &samples_.back();
}

void VisionStateHistory::clear() noexcept
{
  samples_.clear();
  timestamp_domain_ = expected_domain_;
}

}  // namespace rm_auto_aim::vision_time_alignment
