#include "release_manifest_audit/audit.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace
{
namespace fs = std::filesystem;

class AuditTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = fs::temp_directory_path() / ("release_manifest_audit_" +
      std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    fs::create_directories(root_);
  }
  void TearDown() override { fs::remove_all(root_); }
  void write(const fs::path & path, const std::string & contents)
  {
    std::ofstream output(path); ASSERT_TRUE(output.good()); output << contents;
  }
  int execute(const std::string & config, const std::string & output_name = "result")
  {
    write(root_ / "config.json", config); std::ostringstream out; std::ostringstream error;
    return release_manifest_audit::run(root_ / "config.json", root_ / output_name, out, error);
  }
  fs::path root_;
};

constexpr const char * kHead = "1111111111111111111111111111111111111111";
constexpr const char * kBase = "2222222222222222222222222222222222222222";

std::string config(const std::string & source)
{
  return std::string("{\"schema_version\":1,\"candidate\":{\"head\":\"") + kHead +
    "\",\"main_baseline\":\"" + kBase +
    "\",\"branch\":\"fixture\",\"worktree_clean\":true},\"sources\":[{\"id\":\"smoke\",\"kind\":\"release_smoke\",\"path\":\"" + source + "\"}]}";
}

std::string safe_report(const std::string & status = "PASS")
{
  return std::string("{\"schema_version\":1,\"status\":\"") + status +
    "\",\"commit\":\"" + kHead + "\",\"baseline_main\":\"" + kBase +
    "\",\"safety_defaults\":{\"serial_enabled\":false,\"dry_run\":true,\"allow_fire\":false,\"fire_command\":0,\"yaw_vel\":0,\"pitch_vel\":0,\"yaw_acc\":0,\"pitch_acc\":0},\"counts\":{\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":0,\"NOT_RUN\":0,\"NOT_VERIFIED\":0},\"cases\":[{\"status\":\"PASS\"}]}";
}

TEST_F(AuditTest, ValidSourceProducesDeterministicPassManifest)
{
  write(root_ / "smoke.json", safe_report());
  ASSERT_EQ(execute(config((root_ / "smoke.json").string()), "one"), 0);
  ASSERT_EQ(execute(config((root_ / "smoke.json").string()), "two"), 0);
  EXPECT_THROW(
    release_manifest_audit::run(root_ / "config.json", root_ / "one", std::cout, std::cerr),
    std::runtime_error);
  std::ifstream first(root_ / "one" / "release-manifest.json"); std::ifstream second(root_ / "two" / "release-manifest.json");
  std::stringstream a; std::stringstream b; a << first.rdbuf(); b << second.rdbuf(); EXPECT_EQ(a.str(), b.str());
}

TEST_F(AuditTest, MissingMalformedAndUnsafeSourcesFailClosed)
{
  EXPECT_EQ(execute(config((root_ / "missing.json").string())), 1);
  write(root_ / "empty.json", ""); EXPECT_EQ(execute(config((root_ / "empty.json").string()), "empty"), 1);
  write(root_ / "bad.json", "{"); EXPECT_EQ(execute(config((root_ / "bad.json").string()), "bad"), 1);
  auto unsafe = safe_report(); unsafe.replace(unsafe.find("\"fire_command\":0"), 16, "\"fire_command\":1");
  write(root_ / "unsafe.json", unsafe);
  EXPECT_EQ(execute(config((root_ / "unsafe.json").string()), "unsafe"), 1);
}

TEST_F(AuditTest, UnknownSchemaAndProductionClaimsFailClosed)
{
  auto unknown = safe_report(); unknown.replace(unknown.find("\"schema_version\":1"), 18, "\"schema_version\":99");
  write(root_ / "unknown.json", unknown); EXPECT_EQ(execute(config((root_ / "unknown.json").string())), 1);
  auto claimed = safe_report(); claimed.insert(claimed.rfind('}'), ",\"production_ready\":true");
  write(root_ / "claimed.json", claimed); EXPECT_EQ(execute(config((root_ / "claimed.json").string()), "claimed"), 1);
}

TEST_F(AuditTest, HashAndStatisticsMismatchFailClosed)
{
  write(root_ / "smoke.json", safe_report());
  auto invalid_hash = config((root_ / "smoke.json").string());
  const auto source_end = invalid_hash.rfind("}]}");
  ASSERT_NE(source_end, std::string::npos);
  invalid_hash.insert(source_end, ",\"sha256\":\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"");
  EXPECT_EQ(execute(invalid_hash), 1);
  auto mismatch = safe_report(); mismatch.replace(mismatch.find("\"PASS\":1"), 8, "\"PASS\":2"); write(root_ / "counts.json", mismatch);
  EXPECT_EQ(execute(config((root_ / "counts.json").string()), "counts"), 1);
}

TEST_F(AuditTest, MissingE2ELivenessIsNotVerified)
{
  write(root_ / "e2e.json", safe_report());
  auto value = config((root_ / "e2e.json").string()); value.replace(value.find("release_smoke"), 13, "ros_e2e");
  EXPECT_EQ(execute(value), 2);
}

TEST_F(AuditTest, ArtifactHashMismatchAndAbsentArtifactPropagate)
{
  write(root_ / "smoke.json", safe_report()); write(root_ / "model.xml", "fixture model");
  auto value = config((root_ / "smoke.json").string());
  value.insert(value.rfind('}'), ",\"artifacts\":[{\"role\":\"model_xml\",\"path\":\"" +
    (root_ / "model.xml").string() + "\",\"sha256\":\"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa\"}]");
  EXPECT_EQ(execute(value), 1);
  auto unavailable = config((root_ / "smoke.json").string());
  unavailable.insert(unavailable.rfind('}'), ",\"artifacts\":[{\"role\":\"formal_calibration\",\"path\":\"" +
    (root_ / "no-calibration.yaml").string() + "\",\"absence_status\":\"NOT_VERIFIED\"}]");
  EXPECT_EQ(execute(unavailable, "unavailable"), 2);
}

}  // namespace
