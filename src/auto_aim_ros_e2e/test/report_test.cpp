#include "auto_aim_ros_e2e/report.hpp"

#include <filesystem>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>

namespace
{

auto_aim_ros_e2e::NodeLiveness normal_liveness()
{
  auto_aim_ros_e2e::NodeLiveness liveness;
  liveness.alive_before_sampling = true;
  liveness.alive_during_sampling = true;
  liveness.alive_after_sampling = true;
  liveness.expected_exit_code = 0;
  liveness.observed_exit_code = 0;
  liveness.exit_code_matches = true;
  return liveness;
}

TEST(Report, StatusAndShaValidationAreStrict)
{
  EXPECT_STREQ(auto_aim_ros_e2e::status_name(auto_aim_ros_e2e::Status::Pass), "PASS");
  EXPECT_STREQ(
    auto_aim_ros_e2e::status_name(auto_aim_ros_e2e::Status::NotVerified), "NOT_VERIFIED");
  EXPECT_TRUE(auto_aim_ros_e2e::valid_git_sha(std::string(40U, 'a')));
  EXPECT_FALSE(auto_aim_ros_e2e::valid_git_sha(std::string(39U, 'a')));
  EXPECT_FALSE(auto_aim_ros_e2e::valid_git_sha(std::string(40U, 'z')));
}

TEST(Report, NodeLivenessIsFailClosed)
{
  const auto missing = auto_aim_ros_e2e::NodeLiveness{};
  EXPECT_FALSE(auto_aim_ros_e2e::node_liveness_valid(missing));

  auto early_exit = normal_liveness();
  early_exit.alive_during_sampling = false;
  EXPECT_FALSE(auto_aim_ros_e2e::node_liveness_valid(early_exit));

  auto mismatched_exit = normal_liveness();
  mismatched_exit.observed_exit_code = 130;
  mismatched_exit.exit_code_matches = false;
  EXPECT_FALSE(auto_aim_ros_e2e::node_liveness_valid(mismatched_exit));

  EXPECT_TRUE(auto_aim_ros_e2e::node_liveness_valid(normal_liveness()));
}

TEST(Report, MissingNodeLivenessCannotRenderPass)
{
  auto_aim_ros_e2e::CaseResult result;
  result.status = auto_aim_ros_e2e::Status::Pass;
  auto metadata = auto_aim_ros_e2e::ReportMetadata{
    std::string(40U, 'a'), std::string(40U, 'b'), "suite-run", "test", "133", "260033", 1U};
  const auto json = auto_aim_ros_e2e::render_json(metadata, {result}, {});
  EXPECT_NE(json.find("\"status\": \"FAIL\""), std::string::npos);
  EXPECT_NE(json.find("\"alive_during_sampling\": false"), std::string::npos);
  EXPECT_NE(json.find("\"expected_exit_code\": -1"), std::string::npos);
  EXPECT_NE(json.find("\"observed_exit_code\": -1"), std::string::npos);
}

TEST(Report, MissingCaseNodeLivenessCannotRenderPass)
{
  auto_aim_ros_e2e::CaseResult result;
  result.status = auto_aim_ros_e2e::Status::Pass;
  result.node_liveness.expected_exit_code = 0;
  auto metadata = auto_aim_ros_e2e::ReportMetadata{
    std::string(40U, 'a'), std::string(40U, 'b'), "suite-run", "test", "133", "260033", 1U};
  metadata.node_liveness = normal_liveness();
  const auto json = auto_aim_ros_e2e::render_json(metadata, {result}, {});
  EXPECT_NE(json.find("\"status\": \"FAIL\""), std::string::npos);
}

TEST(Report, PartialCaseNodeLivenessCannotRenderPass)
{
  auto_aim_ros_e2e::CaseResult result;
  result.status = auto_aim_ros_e2e::Status::Pass;
  result.node_liveness.observed_exit_code = 0;
  auto metadata = auto_aim_ros_e2e::ReportMetadata{
    std::string(40U, 'a'), std::string(40U, 'b'), "suite-run", "test", "133", "260033", 1U};
  metadata.node_liveness = normal_liveness();
  const auto json = auto_aim_ros_e2e::render_json(metadata, {result}, {});
  EXPECT_NE(json.find("\"status\": \"FAIL\""), std::string::npos);
}

TEST(Report, RenderersPreserveDetailedSafetyEvidence)
{
  auto_aim_ros_e2e::CaseResult result;
  result.round = 1U;
  result.id = "invalid_image";
  result.run_id = "run-1";
  result.status = auto_aim_ros_e2e::Status::Pass;
  result.input_summary = "short data";
  result.expected = "fail closed";
  result.actual = "unlocked";
  result.diagnostic = "adapter rejected input";
  result.node_exit_code = 130;
  result.preflight_exit_code = 2;
  result.control_messages = 4U;
  result.safety_fields_ok = true;
  result.target_lock = "locked=0; unlocked=4";
  result.cleanup_ok = true;
  const auto metadata = auto_aim_ros_e2e::ReportMetadata{
    std::string(40U, 'a'), std::string(40U, 'b'), "suite-run", "Ubuntu; humble",
    "133", "260033", 5U};
  auto report_metadata = metadata;
  report_metadata.node_liveness = normal_liveness();
  result.node_liveness = normal_liveness();
  const auto json = auto_aim_ros_e2e::render_json(report_metadata, {result}, {});
  const auto markdown = auto_aim_ros_e2e::render_markdown(report_metadata, {result}, {});
  EXPECT_NE(json.find("\"safety_fields_ok\": true"), std::string::npos);
  EXPECT_NE(json.find("\"NOT_VERIFIED\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"node_liveness\""), std::string::npos);
  EXPECT_NE(json.find("\"expected_exit_code\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"observed_exit_code\": 0"), std::string::npos);
  EXPECT_NE(json.find("\"status\": \"PASS\""), std::string::npos);
  EXPECT_NE(markdown.find("serial_enabled=false"), std::string::npos);
  EXPECT_NE(markdown.find("Message-level dry-run E2E PASS"), std::string::npos);
  EXPECT_NE(markdown.find("Alive during sampling"), std::string::npos);
}

TEST(Report, NewFileRefusesOverwriteAndSha256IsStable)
{
  const auto directory = std::filesystem::temp_directory_path() /
    ("auto_aim_ros_e2e_report_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(directory);
  const auto path = directory / "artifact.txt";
  auto_aim_ros_e2e::write_new_file(path, "abc");
  EXPECT_EQ(
    auto_aim_ros_e2e::sha256_file(path),
    "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  EXPECT_THROW(auto_aim_ros_e2e::write_new_file(path, "replacement"), std::runtime_error);
  std::filesystem::remove(path);
  std::filesystem::remove(directory);
}

}  // namespace
