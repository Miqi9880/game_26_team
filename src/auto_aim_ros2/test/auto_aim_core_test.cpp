#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/ros_adapters.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include <gtest/gtest.h>

namespace
{
using namespace rm_auto_aim::pipeline;

AutoAimPipeline make_pipeline(std::optional<Detection> detection, CoreConfig config = {})
{
  std::unique_ptr<YoloStage> yolo;
  if (detection.has_value()) {
    yolo = std::make_unique<MockYoloStage>(detection);
  } else {
    yolo = std::make_unique<NullYoloStage>();
  }
  return AutoAimPipeline(
    std::move(yolo), std::make_unique<PassThroughArmorStage>(),
    std::make_unique<LatestTargetTracker>(), std::make_unique<FirstTargetStage>(),
    std::make_unique<CommandAimer>(), config);
}

ImageFrame frame()
{
  ImageFrame result;
  result.width = 640;
  result.height = 480;
  result.encoding = "rgb8";
  return result;
}
}  // namespace

TEST(AutoAimCore, NoTargetReportsUnlockedAndNoFire)
{
  auto pipeline = make_pipeline(std::nullopt);
  const auto command = pipeline.process(frame(), std::chrono::steady_clock::now());
  EXPECT_EQ(command.target_lock, kTargetUnlocked);
  EXPECT_EQ(command.fire_command, kFireNone);
}

TEST(AutoAimCore, ValidTargetReportsLockedAndPreservesInternalRadians)
{
  Detection detection{};
  detection.valid = true;
  detection.yaw_rad = 0.25F;
  detection.pitch_rad = -0.125F;
  detection.yaw_vel_rad_s = 0.5F;
  detection.pitch_acc_rad_s2 = -0.75F;
  auto pipeline = make_pipeline(detection);
  const auto command = pipeline.process(frame(), std::chrono::steady_clock::now());
  EXPECT_EQ(command.target_lock, kTargetLocked);
  EXPECT_FLOAT_EQ(command.yaw_rad, detection.yaw_rad);
  EXPECT_FLOAT_EQ(command.pitch_rad, detection.pitch_rad);
  EXPECT_FLOAT_EQ(command.yaw_vel_rad_s, detection.yaw_vel_rad_s);
  EXPECT_FLOAT_EQ(command.pitch_acc_rad_s2, detection.pitch_acc_rad_s2);
  EXPECT_EQ(command.fire_command, kFireNone);
}

TEST(AutoAimCore, FireIsDisabledWhenConfigurationDisallowsIt)
{
  Detection detection{};
  detection.valid = true;
  detection.fire_request = true;
  auto pipeline = make_pipeline(detection, CoreConfig{false, kFireBurst, kFireSingle});
  const auto command = pipeline.process(frame(), std::chrono::steady_clock::now());
  EXPECT_EQ(command.target_lock, kTargetLocked);
  EXPECT_EQ(command.fire_command, kFireNone);
}

TEST(AutoAimCore, FireUsesConfiguredCommandOnlyWhenEnabled)
{
  Detection detection{};
  detection.valid = true;
  detection.fire_request = true;
  auto pipeline = make_pipeline(detection, CoreConfig{true, 7, 9});
  const auto command = pipeline.process(frame(), std::chrono::steady_clock::now());
  EXPECT_EQ(command.fire_command, 7);
}

TEST(AutoAimCore, InvalidMotionIsRejectedAsNoTarget)
{
  Detection detection{};
  detection.valid = true;
  detection.yaw_rad = std::numeric_limits<float>::quiet_NaN();
  auto pipeline = make_pipeline(detection);
  const auto command = pipeline.process(frame(), std::chrono::steady_clock::now());
  EXPECT_EQ(command.target_lock, kTargetUnlocked);
  EXPECT_EQ(command.fire_command, kFireNone);
}

TEST(AutoAimCore, VisionBookkeepingFieldsAreOutsidePipelineInput)
{
  Detection detection{};
  detection.valid = true;
  detection.yaw_rad = 0.3F;
  auto pipeline = make_pipeline(detection);
  auto first = frame();
  first.shoot_speed_mps = 25.0F;
  first.bullet_count = 1;
  first.game_progress = 2;
  auto second = first;
  second.shoot_speed_mps = 30.0F;
  second.bullet_count = 999;
  second.game_progress = 17;

  const auto command_first = pipeline.process(first, std::chrono::steady_clock::now());
  const auto command_second = pipeline.process(second, std::chrono::steady_clock::now());
  EXPECT_EQ(command_first.target_lock, command_second.target_lock);
  EXPECT_FLOAT_EQ(command_first.yaw_rad, command_second.yaw_rad);
  EXPECT_FLOAT_EQ(command_first.pitch_rad, command_second.pitch_rad);
  EXPECT_EQ(command_first.fire_command, command_second.fire_command);
}

TEST(RosAdapter, ConvertsRadiansToDegreesAndZerosUnconfirmedMotionUnits)
{
  AimCommand command{};
  command.yaw_rad = static_cast<float>(rm_auto_aim::units::kPi / 2.0);
  command.yaw_vel_rad_s = -0.3F;
  command.yaw_acc_rad_s2 = 0.4F;
  command.pitch_rad = static_cast<float>(-rm_auto_aim::units::kPi / 2.0);
  command.pitch_vel_rad_s = 0.6F;
  command.pitch_acc_rad_s2 = -0.7F;
  command.target_lock = kTargetLocked;
  command.fire_command = kFireNone;
  const auto message = rm_auto_aim::ros_adapters::to_ros(command);
  EXPECT_NEAR(message.yaw, 90.0F, 1e-5F);
  EXPECT_NEAR(message.pitch, -90.0F, 1e-5F);
  EXPECT_FLOAT_EQ(message.yaw_vel, 0.0F);
  EXPECT_FLOAT_EQ(message.yaw_acc, 0.0F);
  EXPECT_FLOAT_EQ(message.pitch_vel, 0.0F);
  EXPECT_FLOAT_EQ(message.pitch_acc, 0.0F);
  EXPECT_EQ(message.target_lock, command.target_lock);
  EXPECT_EQ(message.fire_command, command.fire_command);
}

TEST(RosAdapter, ConvertsCanonicalAnglesBothDirections)
{
  EXPECT_FLOAT_EQ(rm_auto_aim::units::degrees_to_radians(0.0F), 0.0F);
  EXPECT_NEAR(
    rm_auto_aim::units::degrees_to_radians(90.0F),
    static_cast<float>(rm_auto_aim::units::kPi / 2.0), 1e-6F);
  EXPECT_NEAR(
    rm_auto_aim::units::degrees_to_radians(-90.0F),
    static_cast<float>(-rm_auto_aim::units::kPi / 2.0), 1e-6F);
  EXPECT_NEAR(
    rm_auto_aim::units::degrees_to_radians(180.0F),
    static_cast<float>(rm_auto_aim::units::kPi), 1e-6F);

  EXPECT_FLOAT_EQ(rm_auto_aim::units::radians_to_degrees(0.0F), 0.0F);
  EXPECT_NEAR(
    rm_auto_aim::units::radians_to_degrees(static_cast<float>(rm_auto_aim::units::kPi / 2.0)),
    90.0F, 1e-5F);
  EXPECT_NEAR(
    rm_auto_aim::units::radians_to_degrees(static_cast<float>(-rm_auto_aim::units::kPi / 2.0)),
    -90.0F, 1e-5F);
  EXPECT_NEAR(
    rm_auto_aim::units::radians_to_degrees(static_cast<float>(rm_auto_aim::units::kPi)),
    180.0F, 1e-5F);
}

TEST(RosAdapter, ConvertsVisionPositionAnglesToInternalRadians)
{
  auto message = auto_aim_interfaces::msg::Vision{};
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 345;
  message.header.frame_id = "imu";
  message.id = 7;
  message.mode = 33;
  message.yaw = 90.0F;
  message.yaw_vel = 123.0F;  // External unit is intentionally not interpreted.
  message.pitch = -90.0F;
  message.pitch_vel = -456.0F;
  message.roll = 180.0F;
  message.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
  message.shoot_speed = 24.5F;
  message.bullet_count = 19;
  message.game_progress = 4;

  const auto state = rm_auto_aim::ros_adapters::to_algorithm_vision(message);
  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->stamp_ns, 12'000'000'345LL);
  EXPECT_EQ(state->frame_id, "imu");
  EXPECT_EQ(state->id, 7U);
  EXPECT_EQ(state->mode, 33U);
  EXPECT_NEAR(state->yaw_rad, rm_auto_aim::units::kPi / 2.0, 1e-6);
  EXPECT_NEAR(state->pitch_rad, -rm_auto_aim::units::kPi / 2.0, 1e-6);
  EXPECT_NEAR(state->roll_rad, rm_auto_aim::units::kPi, 1e-6);
  EXPECT_EQ(state->quaternion_wxyz, (std::array<float, 4>{{1.0F, 0.0F, 0.0F, 0.0F}}));
  EXPECT_FLOAT_EQ(state->shoot_speed_mps, 24.5F);
  EXPECT_EQ(state->bullet_count, 19U);
  EXPECT_EQ(state->game_progress, 4U);
}

TEST(RosAdapter, RejectsNonFiniteVisionEvidence)
{
  auto message = auto_aim_interfaces::msg::Vision{};
  message.yaw = std::numeric_limits<float>::quiet_NaN();
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_algorithm_vision(message).has_value());

  message.yaw = 0.0F;
  message.quaternion[2] = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(rm_auto_aim::ros_adapters::to_algorithm_vision(message).has_value());
}
