#include "auto_aim_ros2/ros_backend.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::pipeline::ImageFrame;
using rm_auto_aim::ros_backend::Backend;
using rm_auto_aim::ros_backend::BackendKind;
using rm_auto_aim::ros_backend::Config;

ImageFrame frame(std::int64_t stamp_ns = 1)
{
  ImageFrame result{};
  result.stamp_ns = stamp_ns;
  result.width = 640;
  result.height = 480;
  result.encoding = "bgr8";
  return result;
}
}  // namespace

TEST(RosBackend, ParsesExplicitBackendKindsAndRejectsUnknown)
{
  EXPECT_EQ(
    rm_auto_aim::ros_backend::parse_backend_kind("null"), BackendKind::Null);
  EXPECT_EQ(
    rm_auto_aim::ros_backend::parse_backend_kind("mock"), BackendKind::Mock);
  EXPECT_EQ(
    rm_auto_aim::ros_backend::parse_backend_kind("offline_reference"),
    BackendKind::OfflineReference);
  EXPECT_THROW(
    rm_auto_aim::ros_backend::parse_backend_kind("default_fake"), std::invalid_argument);
}

TEST(RosBackend, NullBackendFailsClosedAndNeverFires)
{
  Config config{};
  config.kind = BackendKind::Null;
  config.dry_run = true;
  config.allow_fire = false;
  config.serial_enabled = false;
  Backend backend(config);

  const auto result = backend.process(frame(10));
  EXPECT_EQ(result.backend, "null");
  EXPECT_EQ(result.command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.diagnostic_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.command.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_FALSE(result.command_publishable);
  EXPECT_TRUE(result.error.empty());
}

TEST(RosBackend, MockTargetIsObservableButFireRemainsInhibited)
{
  Config config{};
  config.kind = BackendKind::Mock;
  config.dry_run = true;
  config.mock_target = true;
  config.mock_yaw_rad = 0.25F;
  config.mock_pitch_rad = -0.1F;
  config.mock_fire_request = true;
  Backend backend(config);

  const auto result = backend.process(frame(20));
  EXPECT_EQ(result.diagnostic_target_lock, rm_auto_aim::pipeline::kTargetLocked);
  EXPECT_EQ(result.command.target_lock, rm_auto_aim::pipeline::kTargetLocked);
  EXPECT_NEAR(result.command.yaw_rad, 0.25F, 1e-6F);
  EXPECT_NEAR(result.command.pitch_rad, -0.1F, 1e-6F);
  EXPECT_EQ(result.command.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_FALSE(result.command_publishable);
}

TEST(RosBackend, NegativeImageTimestampFailsClosedBeforeBackendProcessing)
{
  Config config{};
  config.kind = BackendKind::Mock;
  config.dry_run = true;
  config.mock_target = true;
  Backend backend(config);

  const auto result = backend.process(frame(-1));
  EXPECT_EQ(result.error, "invalid_image_timestamp");
  EXPECT_EQ(result.diagnostic_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.command.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_FALSE(result.command_publishable);
}

TEST(RosBackend, UnsetImageTimestampFailsClosedBeforeBackendProcessing)
{
  Config config{};
  config.kind = BackendKind::Mock;
  config.dry_run = true;
  config.mock_target = true;
  Backend backend(config);

  const auto result = backend.process(frame(0));
  EXPECT_EQ(result.error, "invalid_image_timestamp");
  EXPECT_EQ(result.diagnostic_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.command.target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(result.command.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_FALSE(result.command_publishable);
}

TEST(RosBackend, OfflineReferenceRequiresExplicitSafeConfiguration)
{
  Config config{};
  config.kind = BackendKind::OfflineReference;
  config.dry_run = false;
  EXPECT_THROW(Backend backend(config), std::invalid_argument);

  config.dry_run = true;
  config.serial_enabled = false;
  config.allow_fire = false;
  EXPECT_THROW(Backend backend(config), std::invalid_argument);

  config.model_path = "/does/not/exist.xml";
  config.pnp_config_path = "/does/not/exist.yaml";
  EXPECT_THROW(Backend backend(config), std::exception);

  config.model_profile_path = "/does/not/exist-profile.yaml";
  EXPECT_THROW(Backend backend(config), std::exception);
}

TEST(RosBackend, CsvDeclaresDiagnosticAndPublishedSafetyFields)
{
  const auto header = rm_auto_aim::ros_backend::csv_header();
  EXPECT_NE(header.find("backend"), std::string::npos);
  EXPECT_NE(header.find("calibration_profile"), std::string::npos);
  EXPECT_NE(header.find("model_profile"), std::string::npos);
  EXPECT_NE(header.find("aimer_mode"), std::string::npos);
  EXPECT_NE(header.find("diagnostic_target_lock"), std::string::npos);
  EXPECT_NE(header.find("published_target_lock"), std::string::npos);
  EXPECT_NE(header.find("absolute_command_valid"), std::string::npos);
  EXPECT_NE(header.find("command_publishable"), std::string::npos);
  EXPECT_NE(header.find("fire_command"), std::string::npos);

  Config config{};
  Backend backend(config);
  const auto row = rm_auto_aim::ros_backend::csv_row(backend.safe_result(30));
  EXPECT_NE(row.find(",0,\n"), std::string::npos);
}
