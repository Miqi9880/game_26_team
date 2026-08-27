#include "auto_aim_release_smoke/release_smoke.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace smoke = auto_aim_release_smoke;

TEST(ReleaseSmokeStatus, UsesOnlyTheFiveDeclaredStates)
{
  EXPECT_STREQ(smoke::status_name(smoke::Status::Pass), "PASS");
  EXPECT_STREQ(smoke::status_name(smoke::Status::Fail), "FAIL");
  EXPECT_STREQ(smoke::status_name(smoke::Status::Unavailable), "UNAVAILABLE");
  EXPECT_STREQ(smoke::status_name(smoke::Status::NotRun), "NOT_RUN");
  EXPECT_STREQ(smoke::status_name(smoke::Status::NotVerified), "NOT_VERIFIED");

  EXPECT_EQ(smoke::parse_status("PASS"), smoke::Status::Pass);
  EXPECT_EQ(smoke::parse_status("FAIL"), smoke::Status::Fail);
  EXPECT_EQ(smoke::parse_status("UNAVAILABLE"), smoke::Status::Unavailable);
  EXPECT_EQ(smoke::parse_status("NOT_RUN"), smoke::Status::NotRun);
  EXPECT_EQ(smoke::parse_status("NOT_VERIFIED"), smoke::Status::NotVerified);
  EXPECT_THROW(smoke::parse_status("WARN"), std::invalid_argument);
}

TEST(ReleaseSmokeMetadata, RequiresFullGitObjectIds)
{
  EXPECT_TRUE(smoke::valid_git_sha("9de02ae662dfecec02ad4701beb626996554935d"));
  EXPECT_TRUE(smoke::valid_git_sha("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"));
  EXPECT_FALSE(smoke::valid_git_sha("9de02ae"));
  EXPECT_FALSE(smoke::valid_git_sha("9de02ae662dfecec02ad4701beb626996554935z"));
}

TEST(ReleaseSmokeReport, PreservesStatusSafetyAndLimitations)
{
  smoke::Options options;
  options.install_base = "/tmp/install";
  options.output_dir = "/tmp/report";
  options.baseline = "9de02ae662dfecec02ad4701beb626996554935d";
  options.commit = "0123456789012345678901234567890123456789";
  const std::vector<smoke::CaseResult> results{
    {"pass", smoke::Status::Pass, "available"},
    {"unavailable", smoke::Status::Unavailable, "missing SDK"},
    {"not-run", smoke::Status::NotRun, "intentionally skipped"},
    {"not-verified", smoke::Status::NotVerified, "hardware absent"},
  };

  const auto json = smoke::render_json(options, results, "Ubuntu\nHumble");
  EXPECT_NE(json.find("\"status\": \"PASS\""), std::string::npos);
  EXPECT_NE(json.find("\"UNAVAILABLE\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"NOT_RUN\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"NOT_VERIFIED\": 1"), std::string::npos);
  EXPECT_NE(json.find("\"serial_enabled\": false"), std::string::npos);
  EXPECT_NE(json.find("\"dry_run\": true"), std::string::npos);
  EXPECT_NE(json.find("\"fire_command\": 0"), std::string::npos);
  EXPECT_NE(json.find("Ubuntu\\nHumble"), std::string::npos);

  const auto markdown = smoke::render_markdown(options, results, "Ubuntu|Humble");
  EXPECT_NE(markdown.find("offline smoke"), std::string::npos);
  EXPECT_NE(markdown.find("`UNAVAILABLE`"), std::string::npos);
  EXPECT_NE(markdown.find("Ubuntu/Humble"), std::string::npos);
}

TEST(ReleaseSmokeReport, AnyRequiredFailureMakesOverallReportFail)
{
  smoke::Options options;
  options.install_base = "/tmp/install";
  options.output_dir = "/tmp/report";
  options.baseline = "9de02ae662dfecec02ad4701beb626996554935d";
  options.commit = "0123456789012345678901234567890123456789";
  const std::vector<smoke::CaseResult> results{
    {"required", smoke::Status::Fail, "contract regression"},
    {"hardware", smoke::Status::NotVerified, "absent"},
  };

  const auto json = smoke::render_json(options, results, "environment");
  EXPECT_NE(json.find("\"status\": \"FAIL\""), std::string::npos);
  EXPECT_NE(json.find("\"FAIL\": 1"), std::string::npos);
}

TEST(ReleaseSmokeReport, EscapesJsonControlCharacters)
{
  EXPECT_EQ(smoke::json_escape("a\"b\\c\n"), "a\\\"b\\\\c\\n");
}

TEST(ReleaseSmokeOutput, ExistingDirectoryIsRejectedWithoutMutation)
{
  const auto root = std::filesystem::temp_directory_path() /
    ("release_smoke_output_test_" + std::to_string(::getpid()));
  const auto install = root / "install";
  const auto output = root / "existing";
  std::filesystem::create_directories(install);
  std::filesystem::create_directories(output);
  const auto sentinel = output / "sentinel.txt";
  {
    std::ofstream file(sentinel);
    file << "preserve me";
  }

  smoke::Options options;
  options.install_base = install;
  options.output_dir = output;
  options.baseline = "9de02ae662dfecec02ad4701beb626996554935d";
  options.commit = "0123456789012345678901234567890123456789";
  std::ostringstream standard_output;
  std::ostringstream standard_error;
  EXPECT_THROW(smoke::run(options, standard_output, standard_error), std::invalid_argument);
  EXPECT_EQ(std::ifstream(sentinel).rdbuf()->sgetc(), 'p');
  EXPECT_FALSE(std::filesystem::exists(output / "logs"));

  std::filesystem::remove_all(root);
}
