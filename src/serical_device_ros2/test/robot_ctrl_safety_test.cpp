#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "robot_ctrl_safety.hpp"

namespace
{

using rm_auto_aim::safety::Config;
using rm_auto_aim::safety::RobotCtrlSafety;

io::RobotCtrlData make_control(std::int8_t lock = 49, std::int8_t fire = 0)
{
  io::RobotCtrlData control{};
  control.yaw = 1.25f;
  control.yaw_vel = -0.2f;
  control.yaw_acc = 0.3f;
  control.pitch = -0.45f;
  control.pitch_vel = 0.4f;
  control.pitch_acc = -0.5f;
  control.target_lock = lock;
  control.fire_command = fire;
  return control;
}

}  // namespace

TEST(RobotCtrlSafety, NoInputProducesSafeDefaults)
{
  std::int64_t now_ns = 0;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });

  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_FLOAT_EQ(output.control.yaw, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch, 0.0f);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.yaw_acc, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_acc, 0.0f);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
}

TEST(RobotCtrlSafety, FireIsDisabledByDefault)
{
  std::int64_t now_ns = 10;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });
  auto input = make_control(49, 2);

  ASSERT_TRUE(safety.Accept(input));
  const auto output = safety.Tick();
  EXPECT_TRUE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 49);
  EXPECT_EQ(output.control.fire_command, 0);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, input.yaw_vel);
}

TEST(RobotCtrlSafety, ConfiguredFireIsAllowedOnlyForLockedTarget)
{
  std::int64_t now_ns = 100;
  const Config config{true, 7, 9};
  RobotCtrlSafety safety(config, [&now_ns]() { return now_ns; });

  auto locked = make_control(49, 7);
  ASSERT_TRUE(safety.Accept(locked));
  EXPECT_EQ(safety.Tick().control.fire_command, 7);

  now_ns += 1;
  auto unlocked = make_control(50, 7);
  EXPECT_FALSE(safety.Accept(unlocked));
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
  EXPECT_FLOAT_EQ(output.control.yaw, unlocked.yaw);
  EXPECT_FLOAT_EQ(output.control.pitch, unlocked.pitch);
}

TEST(RobotCtrlSafety, TimeoutStartsAtTheHundredMillisecondBoundary)
{
  std::int64_t now_ns = 0;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });
  const auto input = make_control(49, 0);
  ASSERT_TRUE(safety.Accept(input));

  now_ns = 99'999'999;
  EXPECT_TRUE(safety.Tick().fresh);

  now_ns = 100'000'000;
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_FLOAT_EQ(output.control.yaw, input.yaw);
  EXPECT_FLOAT_EQ(output.control.pitch, input.pitch);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.yaw_acc, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_acc, 0.0f);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
}

TEST(RobotCtrlSafety, InvalidInputDoesNotRefreshOrReplaceHeldAngles)
{
  std::int64_t now_ns = 0;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });
  const auto valid = make_control(50, 0);
  ASSERT_TRUE(safety.Accept(valid));

  now_ns = 90'000'000;
  auto invalid = valid;
  invalid.yaw = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(safety.Accept(invalid));
  auto invalid_velocity = valid;
  invalid_velocity.pitch_vel = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(safety.Accept(invalid_velocity));

  invalid = valid;
  invalid.target_lock = 48;
  EXPECT_FALSE(safety.Accept(invalid));
  invalid = valid;
  invalid.fire_command = 3;
  EXPECT_FALSE(safety.Accept(invalid));

  now_ns = 100'000'000;
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_FLOAT_EQ(output.control.yaw, valid.yaw);
  EXPECT_FLOAT_EQ(output.control.pitch, valid.pitch);
}

TEST(RobotCtrlSafety, InvalidInputImmediatelyCancelsPreviouslyAcceptedFire)
{
  std::int64_t now_ns = 0;
  const Config config{true, 7, 9};
  RobotCtrlSafety safety(config, [&now_ns]() { return now_ns; });
  const auto locked_fire = make_control(49, 7);
  ASSERT_TRUE(safety.Accept(locked_fire));
  EXPECT_EQ(safety.Tick().control.fire_command, 7);

  now_ns = 1;
  auto invalid = locked_fire;
  invalid.yaw = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(safety.Accept(invalid));
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
  EXPECT_FLOAT_EQ(output.control.yaw, locked_fire.yaw);
  EXPECT_FLOAT_EQ(output.control.pitch, locked_fire.pitch);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.yaw_acc, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_acc, 0.0f);
}

TEST(RobotCtrlSafety, ClockRollbackIsStale)
{
  std::int64_t now_ns = 1'000'000;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });
  const auto input = make_control(49, 0);
  ASSERT_TRUE(safety.Accept(input));

  now_ns = 999'999;
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
  EXPECT_FLOAT_EQ(output.control.yaw, input.yaw);
  EXPECT_FLOAT_EQ(output.control.pitch, input.pitch);
}

TEST(RobotCtrlSafety, AllMotionFieldsMustBeFinite)
{
  const Config config{};
  const float non_finite[] = {
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::infinity(),
    -std::numeric_limits<float>::infinity()};

  for (std::size_t index = 0; index < 6; ++index) {
    auto input = make_control();
    const float value = non_finite[index % 3];
    switch (index) {
      case 0: input.yaw = value; break;
      case 1: input.yaw_vel = value; break;
      case 2: input.yaw_acc = value; break;
      case 3: input.pitch = value; break;
      case 4: input.pitch_vel = value; break;
      case 5: input.pitch_acc = value; break;
      default: break;
    }
    EXPECT_FALSE(RobotCtrlSafety::IsValid(input, config)) << "field index " << index;
  }
}
