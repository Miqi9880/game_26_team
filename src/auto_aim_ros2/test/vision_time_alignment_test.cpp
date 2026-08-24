#include "auto_aim_ros2/vision_time_alignment.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace rm_auto_aim::vision_time_alignment
{
namespace
{
pipeline::VisionState state_at(std::int64_t stamp_ns)
{
  pipeline::VisionState state{};
  state.stamp_ns = stamp_ns;
  state.yaw_rad = 0.1F;
  state.pitch_rad = -0.2F;
  state.roll_rad = 0.03F;
  state.quaternion_wxyz = {{1.0F, 0.0F, 0.0F, 0.0F}};
  state.shoot_speed_mps = 20.0F;
  return state;
}

PairConfig tolerance(std::int64_t value)
{
  PairConfig config{};
  config.tolerance_ns = value;
  return config;
}
}  // namespace

TEST(VisionStateHistoryTest, ExactPairingAndBoundedCapacity)
{
  VisionStateHistory history(2);
  EXPECT_TRUE(history.insert(state_at(100), TimestampDomain::VisionHeader).accepted());
  EXPECT_TRUE(history.insert(state_at(200), TimestampDomain::VisionHeader).accepted());
  EXPECT_TRUE(history.insert(state_at(300), TimestampDomain::VisionHeader).accepted());
  EXPECT_EQ(history.size(), 2U);
  ASSERT_NE(history.oldest(), nullptr);
  EXPECT_EQ(history.oldest()->state.stamp_ns, 200);

  const auto result = history.pair(200, TimestampDomain::VisionHeader, tolerance(0));
  EXPECT_EQ(result.status, PairStatus::Matched);
  ASSERT_TRUE(result.sample.has_value());
  EXPECT_EQ(result.sample->state.stamp_ns, 200);
  EXPECT_EQ(result.delta_ns, 0);
}

TEST(VisionStateHistoryTest, EmptyHistoryAndUnconfiguredToleranceAreDiagnosticOnly)
{
  VisionStateHistory history(2);
  EXPECT_EQ(
    history.pair(100, TimestampDomain::VisionHeader, tolerance(1)).status,
    PairStatus::Missing);
  ASSERT_TRUE(history.insert(state_at(100), TimestampDomain::VisionHeader).accepted());
  EXPECT_EQ(
    history.pair(100, TimestampDomain::VisionHeader).status,
    PairStatus::Unconfigured);
}

TEST(VisionStateHistoryTest, StaleAndFutureAreFailClosed)
{
  VisionStateHistory history(4);
  ASSERT_TRUE(history.insert(state_at(100), TimestampDomain::VisionHeader).accepted());
  EXPECT_EQ(
    history.pair(200, TimestampDomain::VisionHeader, tolerance(10)).status,
    PairStatus::Stale);
  EXPECT_EQ(
    history.pair(50, TimestampDomain::VisionHeader, tolerance(100)).status,
    PairStatus::Future);
  const auto future = history.pair(50, TimestampDomain::VisionHeader, tolerance(100));
  EXPECT_EQ(future.matched_stamp_ns, 100);
  EXPECT_EQ(future.delta_ns, -50);
  EXPECT_FALSE(future.sample.has_value());
}

TEST(VisionStateHistoryTest, FutureCanOnlyBeUsedWhenExplicitlyEnabled)
{
  VisionStateHistory history(4);
  ASSERT_TRUE(history.insert(state_at(200), TimestampDomain::VisionHeader).accepted());
  const auto config = tolerance(10);
  EXPECT_EQ(history.pair(100, TimestampDomain::VisionHeader, config).status, PairStatus::Future);

  auto allow_future = config;
  allow_future.allow_future = true;
  allow_future.tolerance_ns = 100;
  EXPECT_EQ(
    history.pair(100, TimestampDomain::VisionHeader, allow_future).status,
    PairStatus::Matched);
  const auto paired = history.pair(100, TimestampDomain::VisionHeader, allow_future);
  ASSERT_TRUE(paired.sample.has_value());
  EXPECT_EQ(paired.sample->state.stamp_ns, 200);
}

TEST(VisionStateHistoryTest, RejectsTimestampRollbackAndDuplicate)
{
  VisionStateHistory history(4);
  ASSERT_TRUE(history.insert(state_at(100), TimestampDomain::VisionHeader).accepted());
  const auto duplicate = history.insert(state_at(100), TimestampDomain::VisionHeader);
  EXPECT_EQ(duplicate.status, InsertStatus::DuplicateTimestamp);
  EXPECT_EQ(duplicate.reason, InsertReason::DuplicateTimestamp);
  const auto rollback = history.insert(state_at(99), TimestampDomain::VisionHeader);
  EXPECT_EQ(rollback.status, InsertStatus::TimestampRollback);
  EXPECT_EQ(rollback.reason, InsertReason::TimestampRollback);
  EXPECT_EQ(history.size(), 1U);
}

TEST(VisionStateHistoryTest, RejectsInvalidStateAndUnknownDomain)
{
  VisionStateHistory history(4);
  auto zero_time = state_at(0);
  EXPECT_EQ(
    history.insert(zero_time, TimestampDomain::VisionHeader).reason,
    InsertReason::ZeroOrNegativeTimestamp);
  auto nonfinite = state_at(100);
  nonfinite.yaw_rad = std::numeric_limits<float>::quiet_NaN();
  EXPECT_EQ(
    history.insert(nonfinite, TimestampDomain::VisionHeader).reason,
    InsertReason::NonFiniteState);
  auto zero_quaternion = state_at(100);
  zero_quaternion.quaternion_wxyz = {{0.0F, 0.0F, 0.0F, 0.0F}};
  EXPECT_EQ(
    history.insert(zero_quaternion, TimestampDomain::VisionHeader).reason,
    InsertReason::InvalidQuaternion);
  EXPECT_EQ(
    history.insert(state_at(100), TimestampDomain::Unknown).reason,
    InsertReason::UnknownTimestampDomain);
}

TEST(VisionStateHistoryTest, DifferentTimestampDomainsAreIncomparable)
{
  VisionStateHistory history(4);
  ASSERT_TRUE(history.insert(state_at(100), TimestampDomain::VisionHeader).accepted());
  EXPECT_EQ(
    history.pair(100, TimestampDomain::ImageHeader, tolerance(0)).status,
    PairStatus::Incomparable);
  const auto insert_result = history.insert(state_at(200), TimestampDomain::ImageHeader);
  EXPECT_EQ(insert_result.status, InsertStatus::Incomparable);
  EXPECT_EQ(insert_result.reason, InsertReason::DomainMismatch);
}

TEST(VisionStateHistoryTest, NonFiniteQuaternionIsRejected)
{
  VisionStateHistory history(4);
  auto state = state_at(100);
  state.quaternion_wxyz[2] = std::numeric_limits<float>::infinity();
  const auto result = history.insert(state, TimestampDomain::VisionHeader);
  EXPECT_EQ(result.status, InsertStatus::Invalid);
  EXPECT_EQ(result.reason, InsertReason::InvalidQuaternion);
  EXPECT_TRUE(history.empty());
}

}  // namespace rm_auto_aim::vision_time_alignment
