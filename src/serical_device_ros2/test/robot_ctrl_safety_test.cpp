#include <cmath>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include "robot_ctrl_safety.hpp"

namespace
{

using rm_auto_aim::safety::Config;
using rm_auto_aim::safety::RobotCtrlSafety;
using auto_aim_interfaces::control::VehicleProfile;

Config selected_config(bool allow_fire = false)
{
  return Config{allow_fire, 7, 9, VehicleProfile::NewTurtle, rm_auto_aim::safety::kTimeoutNs};
}

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
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });

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
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });
  auto input = make_control(49, 9);

  ASSERT_TRUE(safety.Accept(input));
  const auto output = safety.Tick();
  EXPECT_TRUE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 49);
  EXPECT_EQ(output.control.fire_command, 0);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.yaw_acc, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_vel, 0.0f);
  EXPECT_FLOAT_EQ(output.control.pitch_acc, 0.0f);
}

TEST(RobotCtrlSafety, FireIsSuppressedUntilHardwareSemanticsAreConfirmed)
{
  std::int64_t now_ns = 100;
  const Config config = selected_config(true);
  RobotCtrlSafety safety(config, [&now_ns]() { return now_ns; });

  auto locked = make_control(49, 7);
  ASSERT_TRUE(safety.Accept(locked));
  EXPECT_EQ(safety.Tick().control.fire_command, 0);

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
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });
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
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });
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
  const Config config = selected_config(true);
  RobotCtrlSafety safety(config, [&now_ns]() { return now_ns; });
  const auto locked_fire = make_control(49, 7);
  ASSERT_TRUE(safety.Accept(locked_fire));
  EXPECT_EQ(safety.Tick().control.fire_command, 0);

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
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });
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
  const Config config = selected_config();
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

TEST(RobotCtrlSafety, UnselectedVehicleProfileFailsClosed)
{
  std::int64_t now_ns = 0;
  RobotCtrlSafety safety(Config{}, [&now_ns]() { return now_ns; });
  EXPECT_FALSE(safety.Accept(make_control(49, 0)));
  const auto output = safety.Tick();
  EXPECT_FALSE(output.fresh);
  EXPECT_EQ(output.control.target_lock, 50);
  EXPECT_EQ(output.control.fire_command, 0);
}

TEST(RobotCtrlSafety, NewTurtlePitchIsPreLimitedAndYawIsWrapped)
{
  std::int64_t now_ns = 0;
  RobotCtrlSafety safety(selected_config(), [&now_ns]() { return now_ns; });
  auto input = make_control(49, 0);
  input.yaw = 540.0F;
  input.pitch = 45.0F;
  ASSERT_TRUE(safety.Accept(input));
  const auto output = safety.Tick();
  EXPECT_TRUE(output.fresh);
  EXPECT_FLOAT_EQ(output.control.yaw, 180.0F);
  EXPECT_FLOAT_EQ(output.control.pitch, 19.0F);
  EXPECT_TRUE(output.yaw_wrapped);
  EXPECT_TRUE(output.pitch_clamped);
  EXPECT_FLOAT_EQ(output.control.yaw_vel, 0.0F);
  EXPECT_FLOAT_EQ(output.control.pitch_vel, 0.0F);
}

TEST(RobotCtrlSafety, DogLegPitchLimitsAreIndependent)
{
  std::int64_t now_ns = 0;
  auto config = selected_config();
  config.vehicle_profile = VehicleProfile::DogLeg;
  RobotCtrlSafety safety(config, [&now_ns]() { return now_ns; });
  auto input = make_control(49, 0);
  input.pitch = -20.0F;
  ASSERT_TRUE(safety.Accept(input));
  const auto output = safety.Tick();
  EXPECT_FLOAT_EQ(output.control.pitch, -10.0F);
  EXPECT_TRUE(output.pitch_clamped);
}

TEST(RobotCtrlSafety, ControlPeriodSupportsParameterizedHundredsOfHertz)
{
  const auto period_100 = rm_auto_aim::safety::control_period_from_hz(100.0);
  ASSERT_TRUE(period_100.has_value());
  EXPECT_EQ(period_100->count(), 10'000'000);

  const auto period_250 = rm_auto_aim::safety::control_period_from_hz(250.0);
  ASSERT_TRUE(period_250.has_value());
  EXPECT_EQ(period_250->count(), 4'000'000);

  const auto shared_period_333 =
    auto_aim_interfaces::control::control_period_from_hz(333.0);
  const auto bridge_period_333 = rm_auto_aim::safety::control_period_from_hz(333.0);
  ASSERT_TRUE(shared_period_333.has_value());
  ASSERT_TRUE(bridge_period_333.has_value());
  EXPECT_EQ(shared_period_333->count(), 3'003'003);
  EXPECT_EQ(*shared_period_333, *bridge_period_333);

  EXPECT_FALSE(rm_auto_aim::safety::control_period_from_hz(0.0).has_value());
  const auto too_slow_hz = 1'000'000'000.0 /
    static_cast<double>(std::numeric_limits<std::int64_t>::max());
  EXPECT_FALSE(auto_aim_interfaces::control::control_period_from_hz(too_slow_hz).has_value());
  EXPECT_FALSE(rm_auto_aim::safety::control_period_from_hz(too_slow_hz).has_value());
  EXPECT_FALSE(rm_auto_aim::safety::control_period_from_hz(
    std::numeric_limits<double>::quiet_NaN()).has_value());
}
