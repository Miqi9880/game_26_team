#include "release_manifest_audit/audit.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <openssl/evp.h>

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
    write(root_ / "ctest.xml",
      "<?xml version=\"1.0\"?><Site><Testing><TestList><Test>./audit_test</Test></TestList>"
      "<Test Status=\"passed\"><Name>audit_test</Name></Test></Testing></Site>");
  }
  void TearDown() override { fs::remove_all(root_); }
  void write(const fs::path & path, const std::string & contents)
  {
    std::ofstream output(path); ASSERT_TRUE(output.good()); output << contents;
  }
  std::string read(const fs::path & path)
  {
    std::ifstream input(path); std::ostringstream contents; contents << input.rdbuf(); return contents.str();
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

std::string sha256_file(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input.good()) return {};
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  EXPECT_NE(context, nullptr);
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    if (context != nullptr) EVP_MD_CTX_free(context);
    return {};
  }
  char buffer[4096];
  while (input) {
    input.read(buffer, sizeof(buffer));
    const auto count = input.gcount();
    if (count > 0) EXPECT_EQ(EVP_DigestUpdate(context, buffer, static_cast<std::size_t>(count)), 1);
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned size = 0U;
  if (EVP_DigestFinal_ex(context, digest, &size) != 1) {
    EVP_MD_CTX_free(context);
    return {};
  }
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned i = 0; i < size; ++i) output << std::setw(2) << static_cast<unsigned>(digest[i]);
  return output.str();
}

std::string config(const std::string & source)
{
  const auto digest = sha256_file(source);
  const auto ctest = fs::path(source).parent_path() / "ctest.xml";
  const auto digest_field = digest.empty() ? std::string{} : std::string("\",\"sha256\":\"") + digest;
  const auto ctest_digest_field = sha256_file(ctest);
  const auto ctest_field = ctest_digest_field.empty() ? std::string{} : std::string("\",\"ctest_xml_sha256\":\"") + ctest_digest_field;
  return std::string("{\"schema_version\":1,\"candidate\":{\"head\":\"") + kHead +
    "\",\"main_baseline\":\"" + kBase +
    "\",\"branch\":\"fixture\",\"worktree_clean\":true},\"sources\":[{\"id\":\"smoke\",\"kind\":\"release_smoke\",\"path\":\"" + source + digest_field + "\",\"ctest_xml\":\"" +
    (fs::path(source).parent_path() / "ctest.xml").string() + ctest_field + "\"}]}";
}

std::string safe_report(const std::string & status = "PASS")
{
  return std::string("{\"schema_version\":1,\"status\":\"") + status +
    "\",\"commit\":\"" + kHead + "\",\"baseline_main\":\"" + kBase +
    "\",\"safety_defaults\":{\"serial_enabled\":false,\"dry_run\":true,\"allow_fire\":false,\"fire_command\":0,\"yaw_vel\":0,\"pitch_vel\":0,\"yaw_acc\":0,\"pitch_acc\":0},\"counts\":{\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":0,\"NOT_RUN\":0,\"NOT_VERIFIED\":0},\"cases\":[{\"name\":\"audit_test\",\"status\":\"PASS\"}]}";
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
  const auto source_hash = invalid_hash.find("\"sha256\":\"");
  ASSERT_NE(source_hash, std::string::npos);
  invalid_hash.replace(source_hash + std::string("\"sha256\":\"").size(), 64,
    "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa");
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

TEST_F(AuditTest, ExistingWarnStatusPropagatesAsNotVerified)
{
  write(root_ / "warn.json", safe_report("WARN"));
  EXPECT_EQ(execute(config((root_ / "warn.json").string())), 2);
}

TEST_F(AuditTest, AllUnavailableReportStatusesRemainNonPass)
{
  for (const auto & report_status : {"UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}) {
    auto report = safe_report(report_status);
    const auto counts = report.find(",\"counts\":");
    ASSERT_NE(counts, std::string::npos);
    report.erase(counts, report.rfind('}') - counts);
    write(root_ / "status.json", report);
    auto value = config((root_ / "status.json").string());
    const auto ctest = value.find(",\"ctest_xml\":");
    ASSERT_NE(ctest, std::string::npos);
    value.erase(ctest, value.find('}', ctest) - ctest);
    EXPECT_EQ(execute(value, report_status), 2);
  }
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
  for (const auto & absence_status : {"UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}) {
    auto absent = config((root_ / "smoke.json").string());
    absent.insert(absent.rfind('}'), ",\"artifacts\":[{\"role\":\"missing_" + std::string(absence_status) + "\",\"path\":\"" +
      (root_ / "missing-artifact").string() + "\",\"absence_status\":\"" + absence_status + "\"}]");
    EXPECT_EQ(execute(absent, std::string("absent_") + absence_status), 2);
  }
  auto invalid_absence = config((root_ / "smoke.json").string());
  invalid_absence.insert(invalid_absence.rfind('}'), ",\"artifacts\":[{\"role\":\"missing_pass\",\"path\":\"" +
    (root_ / "missing-artifact").string() + "\",\"absence_status\":\"PASS\"}]");
  EXPECT_EQ(execute(invalid_absence, "invalid_absence"), 1);
}

TEST_F(AuditTest, PresentSourcesAndArtifactsCannotCarryAbsenceStatus)
{
  write(root_ / "present.json", safe_report());
  auto source_config = config((root_ / "present.json").string());
  const auto source_marker = source_config.rfind("\"ctest_xml_sha256\"");
  ASSERT_NE(source_marker, std::string::npos);
  // Insert the declaration next to the source's explicit path/hash fields.
  const auto source_end = source_config.find("}]", source_marker);
  ASSERT_NE(source_end, std::string::npos);
  source_config.insert(source_end, ",\"absence_status\":\"NOT_VERIFIED\"");
  EXPECT_EQ(execute(source_config, "present_source_absence"), 1);
  const auto source_manifest = read(root_ / "present_source_absence" / "release-manifest.json");
  EXPECT_NE(source_manifest.find("absence_status contradicts present source"), std::string::npos);

  write(root_ / "artifact.xml", "model artifact");
  auto artifact_config = config((root_ / "present.json").string());
  const auto artifact_hash = sha256_file(root_ / "artifact.xml");
  artifact_config.insert(artifact_config.rfind('}'), ",\"artifacts\":[{\"role\":\"model_xml\",\"path\":\"" +
    (root_ / "artifact.xml").string() + "\",\"sha256\":\"" + artifact_hash +
    "\",\"absence_status\":\"NOT_VERIFIED\"}]");
  EXPECT_EQ(execute(artifact_config, "present_artifact_absence"), 1);
  const auto artifact_manifest = read(root_ / "present_artifact_absence" / "release-manifest.json");
  EXPECT_NE(artifact_manifest.find("absence_status contradicts present artifact"), std::string::npos);
}

TEST_F(AuditTest, CTestXmlMissingMalformedAndMismatchedFailClosed)
{
  write(root_ / "smoke.json", safe_report());
  auto missing = config((root_ / "smoke.json").string());
  const auto ctest = (root_ / "ctest.xml").string();
  missing.replace(missing.find(ctest), ctest.size(), (root_ / "missing.xml").string());
  EXPECT_EQ(execute(missing, "missing_xml"), 1);
  write(root_ / "broken.xml", "<Site><Test Status=\"passed\">");
  auto broken = config((root_ / "smoke.json").string());
  broken.replace(broken.find(ctest), ctest.size(), (root_ / "broken.xml").string());
  EXPECT_EQ(execute(broken, "broken_xml"), 1);
  write(root_ / "mismatch.xml", "<Site><Testing><Test Status=\"failed\"></Test></Testing></Site>");
  auto mismatch = config((root_ / "smoke.json").string());
  mismatch.replace(mismatch.find(ctest), ctest.size(), (root_ / "mismatch.xml").string());
  EXPECT_EQ(execute(mismatch, "mismatch_xml"), 1);
}

TEST_F(AuditTest, StandardCTestTestListAndResultStatusesAreParsed)
{
  auto report = safe_report("FAIL");
  const std::string old_counts = "\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":0,\"NOT_RUN\":0";
  report.replace(report.find(old_counts), old_counts.size(),
    "\"PASS\":1,\"FAIL\":1,\"UNAVAILABLE\":0,\"NOT_RUN\":1");
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  report.replace(report.find(old_case), old_case.size(),
    "[{\"name\":\"pass\",\"status\":\"PASS\"},{\"name\":\"fail\",\"status\":\"FAIL\"},{\"name\":\"notrun\",\"status\":\"NOT_RUN\"}]");
  write(root_ / "mixed.json", report);
  write(root_ / "ctest.xml",
    "<?xml version=\"1.0\"?><Site><Testing><TestList><Test>./pass</Test><Test>./fail</Test>"
    "<Test>./notrun</Test></TestList><Test Status=\"passed\"><Name>pass</Name></Test>"
    "<Test Status=\"failed\"><Name>fail</Name></Test>"
    "<Test Status=\"notrun\"><Name>notrun</Name></Test></Testing></Site>");
  EXPECT_EQ(execute(config((root_ / "mixed.json").string()), "mixed_statuses"), 1);
  const auto manifest = read(root_ / "mixed_statuses" / "release-manifest.json");
  EXPECT_NE(manifest.find("source reports FAIL"), std::string::npos);
  EXPECT_EQ(manifest.find("disagree with CTest XML"), std::string::npos);
}

TEST_F(AuditTest, DirectCTestResultWithoutStatusFailsClosed)
{
  write(root_ / "smoke.json", safe_report());
  write(root_ / "ctest.xml",
    "<?xml version=\"1.0\"?><Site><Testing><TestList><Test>./audit_test</Test></TestList>"
    "<Test><Name>incomplete_result</Name></Test>"
    "<Test Status=\"passed\"><Name>audit_test</Name></Test></Testing></Site>");
  EXPECT_EQ(execute(config((root_ / "smoke.json").string()), "missing_result_status"), 1);
}

TEST_F(AuditTest, TopLevelPassContradictingFailedCaseFailsClosed)
{
  auto report = safe_report("PASS");
  const std::string old_counts = "\"PASS\":1,\"FAIL\":0";
  report.replace(report.find(old_counts), old_counts.size(), "\"PASS\":0,\"FAIL\":1");
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  report.replace(report.find(old_case), old_case.size(), "[{\"name\":\"failed_test\",\"status\":\"FAIL\"}]");
  write(root_ / "contradiction.json", report);
  write(root_ / "ctest.xml",
    "<?xml version=\"1.0\"?><Site><Testing><TestList><Test>./failed_test</Test></TestList>"
    "<Test Status=\"failed\"><Name>failed_test</Name></Test></Testing></Site>");
  EXPECT_EQ(execute(config((root_ / "contradiction.json").string()), "pass_with_failure"), 1);
  const auto manifest = read(root_ / "pass_with_failure" / "release-manifest.json");
  EXPECT_NE(manifest.find("report top-level PASS hides failed wrapper case"),
    std::string::npos);
}

TEST_F(AuditTest, TopLevelWarnCannotHideFailedCase)
{
  auto report = safe_report("WARN");
  const std::string old_counts = "\"PASS\":1,\"FAIL\":0";
  report.replace(report.find(old_counts), old_counts.size(), "\"PASS\":0,\"FAIL\":1");
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  report.replace(report.find(old_case), old_case.size(), "[{\"name\":\"failed_test\",\"status\":\"FAIL\"}]");
  write(root_ / "warn-contradiction.json", report);
  write(root_ / "ctest.xml",
    "<?xml version=\"1.0\"?><Site><Testing><TestList><Test>./failed_test</Test></TestList>"
    "<Test Status=\"failed\"><Name>failed_test</Name></Test></Testing></Site>");
  EXPECT_EQ(execute(config((root_ / "warn-contradiction.json").string()), "warn_with_failure"), 1);
}

TEST_F(AuditTest, DeclaredCTestXmlRequiresCountsAndCases)
{
  auto report = safe_report();
  const auto counts = report.find(",\"counts\":");
  ASSERT_NE(counts, std::string::npos);
  report.erase(counts, report.rfind('}') - counts);
  write(root_ / "no-statistics.json", report);
  EXPECT_EQ(execute(config((root_ / "no-statistics.json").string()), "ctest_without_statistics"), 1);
}

TEST_F(AuditTest, ReviewedWrapperCanExplicitlyMarkCTestNotApplicable)
{
  auto report = safe_report();
  const std::string old_counts = "\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":0,\"NOT_RUN\":0,\"NOT_VERIFIED\":0";
  report.replace(report.find(old_counts), old_counts.size(),
    "\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":1,\"NOT_RUN\":0,\"NOT_VERIFIED\":0");
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  report.replace(report.find(old_case), old_case.size(),
    "[{\"name\":\"audit_test\",\"status\":\"PASS\"},{\"name\":\"camera\",\"status\":\"UNAVAILABLE\"}]");
  write(root_ / "wrapper_not_applicable.json", report);

  auto value = config((root_ / "wrapper_not_applicable.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end, ",\"ctest_not_applicable\":true");
  EXPECT_EQ(execute(value, "wrapper_not_applicable"), 0);
}

TEST_F(AuditTest, WrapperPassCannotHideFailedCaseWhenCTestIsNotApplicable)
{
  auto report = safe_report();
  const std::string old_counts = "\"PASS\":1,\"FAIL\":0,\"UNAVAILABLE\":0,\"NOT_RUN\":0,\"NOT_VERIFIED\":0";
  report.replace(report.find(old_counts), old_counts.size(),
    "\"PASS\":1,\"FAIL\":1,\"UNAVAILABLE\":0,\"NOT_RUN\":0,\"NOT_VERIFIED\":0");
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  report.replace(report.find(old_case), old_case.size(),
    "[{\"name\":\"audit_test\",\"status\":\"PASS\"},{\"name\":\"failed\",\"status\":\"FAIL\"}]");
  write(root_ / "wrapper_hidden_failure.json", report);

  auto value = config((root_ / "wrapper_hidden_failure.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end, ",\"ctest_not_applicable\":true");
  EXPECT_EQ(execute(value, "wrapper_hidden_failure"), 1);
  const auto manifest = read(root_ / "wrapper_hidden_failure" / "release-manifest.json");
  EXPECT_NE(manifest.find("report top-level PASS hides failed wrapper case"), std::string::npos);
}

TEST_F(AuditTest, WrapperNonPassMustMatchCaseAggregateWhenCTestIsNotApplicable)
{
  auto report = safe_report("NOT_VERIFIED");
  write(root_ / "wrapper_stale_status.json", report);

  auto value = config((root_ / "wrapper_stale_status.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end, ",\"ctest_not_applicable\":true");
  EXPECT_EQ(execute(value, "wrapper_stale_status"), 1);
  const auto manifest = read(root_ / "wrapper_stale_status" / "release-manifest.json");
  EXPECT_NE(manifest.find("top-level status NOT_VERIFIED contradicts aggregated case status PASS"), std::string::npos);
}

TEST_F(AuditTest, CTestNotApplicableIsRestrictedToReviewedWrappers)
{
  write(root_ / "non_wrapper.json", safe_report());
  auto value = config((root_ / "non_wrapper.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto kind = value.find("release_smoke");
  ASSERT_NE(kind, std::string::npos);
  value.replace(kind, std::string("release_smoke").size(), "ctest");
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end, ",\"ctest_not_applicable\":true");
  EXPECT_EQ(execute(value, "non_wrapper_not_applicable"), 1);
  const auto manifest = read(root_ / "non_wrapper_not_applicable" / "release-manifest.json");
  EXPECT_NE(manifest.find("ctest_not_applicable is restricted to reviewed wrapper kinds"), std::string::npos);
}

TEST_F(AuditTest, UnknownKindAndHardwarePassAreRejected)
{
  write(root_ / "smoke.json", safe_report());
  auto unknown = config((root_ / "smoke.json").string());
  unknown.replace(unknown.find("release_smoke"), 13, "unknown_kind");
  EXPECT_EQ(execute(unknown, "unknown_kind"), 1);
  auto hardware = config((root_ / "smoke.json").string());
  hardware.replace(hardware.find("release_smoke"), 13, "hardware");
  EXPECT_EQ(execute(hardware, "hardware_pass"), 1);
}

TEST_F(AuditTest, DuplicateStatusAliasAndCTestCaseFailClosed)
{
  auto report = safe_report();
  report.insert(report.find("{\"schema_version\":1") + 1, "\"overall_status\":\"FAIL\",");
  write(root_ / "aliases.json", report);
  EXPECT_EQ(execute(config((root_ / "aliases.json").string()), "aliases"), 1);
  auto mismatch = config((root_ / "aliases.json").string());
  write(root_ / "valid.json", safe_report());
  mismatch.replace(mismatch.find("aliases.json"), std::string("aliases.json").size(), "valid.json");
  write(root_ / "ctest.xml", "<Site><Testing><Test Status=\"passed\"><Name>one</Name></Test><Test Status=\"failed\"><Name>one</Name></Test></Testing></Site>");
  EXPECT_EQ(execute(mismatch, "duplicate_ctest"), 1);
}

TEST_F(AuditTest, EveryStatusAliasMustAgree)
{
  auto report = safe_report();
  report.insert(report.find("{\"schema_version\":1") + 1, "\"report_status\":\"FAIL\",");
  write(root_ / "status_alias.json", report);
  EXPECT_EQ(execute(config((root_ / "status_alias.json").string()), "status_alias"), 1);
  const auto manifest = read(root_ / "status_alias" / "release-manifest.json");
  EXPECT_NE(manifest.find("status aliases disagree"), std::string::npos);

  auto camel = safe_report();
  camel.insert(camel.find("{\"schema_version\":1") + 1, "\"overallStatus\":\"FAIL\",");
  write(root_ / "camel_status_alias.json", camel);
  EXPECT_EQ(execute(config((root_ / "camel_status_alias.json").string()), "camel_status_alias"), 1);
}

TEST_F(AuditTest, CTestXmlDigestMustBeDeclaredByConfiguration)
{
  auto report = safe_report();
  const auto xml_digest = sha256_file(root_ / "ctest.xml");
  report.insert(report.rfind('}'), ",\"ctest_xml_sha256\":\"" + xml_digest + "\"");
  write(root_ / "root_digest_only.json", report);
  auto value = config((root_ / "root_digest_only.json").string());
  const auto marker = value.find(",\"ctest_xml_sha256\":\"");
  ASSERT_NE(marker, std::string::npos);
  const auto end_quote = value.find('"', marker + std::string(",\"ctest_xml_sha256\":\"").size());
  ASSERT_NE(end_quote, std::string::npos);
  value.erase(marker, end_quote - marker + 1U);
  EXPECT_EQ(execute(value, "missing_ctest_digest"), 1);
  const auto manifest = read(root_ / "missing_ctest_digest" / "release-manifest.json");
  EXPECT_NE(manifest.find("ctest_xml_sha256 is required when ctest_xml is declared"), std::string::npos);
}

TEST_F(AuditTest, SkipCTestIsNeverAnAdmissionBypass)
{
  write(root_ / "skip_config.json", safe_report());
  auto value = config((root_ / "skip_config.json").string());
  value.insert(value.rfind("}]"), ",\"skip_ctest\":true");
  EXPECT_EQ(execute(value, "skip_config"), 1);

  auto report = safe_report();
  report.insert(report.rfind('}'), ",\"skip_ctest\":true");
  write(root_ / "skip_report.json", report);
  EXPECT_EQ(execute(config((root_ / "skip_report.json").string()), "skip_report"), 1);
}

TEST_F(AuditTest, CountsRejectUnknownStatusKeys)
{
  auto report = safe_report();
  report.replace(report.find("\"counts\":{") + std::string("\"counts\":{").size(), 0,
    "\"MYSTERY\":0,");
  write(root_ / "unknown_count.json", report);
  EXPECT_EQ(execute(config((root_ / "unknown_count.json").string()), "unknown_count"), 1);
  const auto manifest = read(root_ / "unknown_count" / "release-manifest.json");
  EXPECT_NE(manifest.find("counts contains unknown status key"), std::string::npos);
}

TEST_F(AuditTest, BranchWhitespaceIsNotAValidCandidateIdentity)
{
  write(root_ / "branch.json", safe_report());
  auto value = config((root_ / "branch.json").string());
  const auto marker = value.find("\"branch\":\"fixture\"");
  ASSERT_NE(marker, std::string::npos);
  value.replace(marker, std::string("\"branch\":\"fixture\"").size(), "\"branch\":\" \\t\\n \"");
  EXPECT_THROW(execute(value, "blank_branch"), std::runtime_error);
}

TEST_F(AuditTest, CandidateSHAAliasConflictsAreRejected)
{
  write(root_ / "candidate_alias.json", safe_report());
  auto head_conflict = config((root_ / "candidate_alias.json").string());
  const auto head_marker = head_conflict.find("\",\"main_baseline\":\"");
  ASSERT_NE(head_marker, std::string::npos);
  head_conflict.insert(head_marker, ",\"candidate_commit\":\"3333333333333333333333333333333333333333\"");
  EXPECT_THROW(execute(head_conflict, "candidate_head_alias_conflict"), std::runtime_error);

  auto base_conflict = config((root_ / "candidate_alias.json").string());
  const auto base_marker = base_conflict.find("\",\"branch\":\"");
  ASSERT_NE(base_marker, std::string::npos);
  base_conflict.insert(base_marker, ",\"origin_main_sha\":\"4444444444444444444444444444444444444444\"");
  EXPECT_THROW(execute(base_conflict, "candidate_base_alias_conflict"), std::runtime_error);
}

TEST_F(AuditTest, UnsafeClaimKeysAreCanonicalizedAcrossSeparatorsAndCase)
{
  auto report = safe_report();
  report.insert(report.rfind('}'), ",\"PRODUCTIONREADY\":true");
  write(root_ / "canonical_claim.json", report);
  EXPECT_EQ(execute(config((root_ / "canonical_claim.json").string()), "canonical_claim"), 1);
}

TEST_F(AuditTest, NegativeEvidenceRequiresExactCalibrationPromotionDiagnostic)
{
  const auto make_negative = [&](const std::string & warnings, const std::string & errors) {
    return std::string("{\"schema_version\":1,\"status\":\"FAIL\",\"production_claim_rejected\":true,") +
      "\"observed_exit_code\":1,\"fixture_sha256\":\"" + std::string(64, '0') +
      "\",\"commit\":\"" + kHead + "\",\"baseline_main\":\"" + kBase +
      "\",\"safety_defaults\":{\"serial_enabled\":false,\"dry_run\":true,\"allow_fire\":false,\"fire_command\":0,\"yaw_vel\":0,\"pitch_vel\":0,\"yaw_acc\":0,\"pitch_acc\":0},\"diagnostics\":{\"errors\":" + errors + ",\"warnings\":" + warnings + "}}";
  };
  const auto declaration = [&](const fs::path & path, const std::string & output_name) {
    auto value = config(path.string());
    const auto ctest = value.find(",\"ctest_xml\":");
    if (ctest == std::string::npos) {
      ADD_FAILURE() << "CTest declaration missing";
      return std::string{};
    }
    const auto source_close = value.find('}', ctest);
    if (source_close == std::string::npos) {
      ADD_FAILURE() << "source object terminator missing";
      return std::string{};
    }
    value.erase(ctest, source_close - ctest);
    const auto source_end = value.rfind("}]");
    if (source_end == std::string::npos) {
      ADD_FAILURE() << "source declaration terminator missing";
      return std::string{};
    }
    value.insert(source_end, ",\"negative_test\":true,\"expected_failure\":true,\"expected_diagnostic_code\":\"calibration_promotion\",\"expected_exit_code\":1,\"fixture_sha256\":\"" + std::string(64, '0') + "\"");
    EXPECT_EQ(execute(value, output_name), 1);
    return read(root_ / output_name / "release-manifest.json");
  };

  write(root_ / "negative_warning.json", make_negative("[{\"code\":\"extra\"}]", "[{\"code\":\"calibration_promotion\"}]"));
  const auto warning_manifest = declaration(root_ / "negative_warning.json", "negative_warning");
  EXPECT_NE(warning_manifest.find("diagnostics.warnings must be an empty array"), std::string::npos);

  write(root_ / "negative_extra_error.json", make_negative("[]", "[{\"code\":\"calibration_promotion\"},{\"code\":\"other\"}]"));
  const auto error_manifest = declaration(root_ / "negative_extra_error.json", "negative_extra_error");
  EXPECT_NE(error_manifest.find("exactly one diagnostic error"), std::string::npos);

  auto missing_code = config((root_ / "negative_extra_error.json").string());
  const auto missing_ctest = missing_code.find(",\"ctest_xml\":");
  ASSERT_NE(missing_ctest, std::string::npos);
  const auto missing_source_close = missing_code.find('}', missing_ctest);
  ASSERT_NE(missing_source_close, std::string::npos);
  missing_code.erase(missing_ctest, missing_source_close - missing_ctest);
  const auto missing_source_end = missing_code.rfind("}]");
  ASSERT_NE(missing_source_end, std::string::npos);
  missing_code.insert(missing_source_end,
    ",\"negative_test\":true,\"expected_failure\":true,\"expected_exit_code\":1,\"fixture_sha256\":\"" + std::string(64, '0') + "\"");
  EXPECT_EQ(execute(missing_code, "negative_missing_declaration"), 1);
  const auto missing_manifest = read(root_ / "negative_missing_declaration" / "release-manifest.json");
  EXPECT_NE(missing_manifest.find("negative evidence requires expected_diagnostic_code"), std::string::npos);
}

TEST_F(AuditTest, NegativeEvidenceFixtureAliasesBindFixtureNotReportBytes)
{
  const auto fixture_sha = std::string(64, 'a');
  const auto report = std::string(
    "{\"schema_version\":1,\"status\":\"FAIL\",\"production_claim_rejected\":true,"
    "\"production_ready\":true,\"exit_code\":1,"
    "\"fixture_hash\":\"") + fixture_sha +
    "\",\"commit\":\"" + kHead + "\",\"baseline_main\":\"" + kBase +
    "\",\"safety_defaults\":{\"serial_enabled\":false,\"dry_run\":true,"
    "\"allow_fire\":false,\"fire_command\":0,\"yaw_vel\":0,\"pitch_vel\":0,"
    "\"yaw_acc\":0,\"pitch_acc\":0},\"diagnostics\":{\"errors\":[{"
    "\"code\":\"calibration_promotion\"}],\"warnings\":[]}}";
  write(root_ / "negative_aliases.json", report);
  auto value = config((root_ / "negative_aliases.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end,
    ",\"negative_test\":true,\"expected_failure\":true,"
    "\"expected_diagnostic_code\":\"calibration_promotion\","
    "\"expected_failure_exit_code\":1,\"expected_fixture_sha256\":\"" + fixture_sha + "\"");
  EXPECT_EQ(execute(value, "negative_aliases"), 0);
  const auto manifest = read(root_ / "negative_aliases" / "release-manifest.json");
  EXPECT_NE(manifest.find("\"status\":\"PASS\""), std::string::npos);
  EXPECT_EQ(manifest.find("production or hardware claim detected"), std::string::npos);

  auto conflict = report;
  const auto marker = conflict.find("\"fixture_hash\":\"");
  ASSERT_NE(marker, std::string::npos);
  conflict.insert(marker, "\"fixture_sha256\":\"" + std::string(64, 'b') + "\",");
  write(root_ / "negative_aliases_conflict.json", conflict);
  auto conflict_config = config((root_ / "negative_aliases_conflict.json").string());
  const auto conflict_ctest = conflict_config.find(",\"ctest_xml\":");
  ASSERT_NE(conflict_ctest, std::string::npos);
  const auto conflict_source_close = conflict_config.find('}', conflict_ctest);
  ASSERT_NE(conflict_source_close, std::string::npos);
  conflict_config.erase(conflict_ctest, conflict_source_close - conflict_ctest);
  const auto conflict_source_end = conflict_config.rfind("}]");
  ASSERT_NE(conflict_source_end, std::string::npos);
  conflict_config.insert(conflict_source_end,
    ",\"negative_test\":true,\"expected_failure\":true,"
    "\"expected_diagnostic_code\":\"calibration_promotion\","
    "\"expected_exit_code\":1,\"fixture_sha256\":\"" + fixture_sha + "\"");
  EXPECT_EQ(execute(conflict_config, "negative_aliases_conflict"), 1);
  const auto conflict_manifest = read(root_ / "negative_aliases_conflict" / "release-manifest.json");
  EXPECT_NE(conflict_manifest.find("negative evidence fixture hash aliases disagree"), std::string::npos);

  auto nested_unsafe = report;
  const auto safety_marker = nested_unsafe.find("\"safety_defaults\":{");
  ASSERT_NE(safety_marker, std::string::npos);
  nested_unsafe.insert(safety_marker, "\"claims\":{\"production_ready\":true},");
  write(root_ / "negative_nested_unsafe.json", nested_unsafe);
  auto nested_config = config((root_ / "negative_nested_unsafe.json").string());
  const auto nested_ctest = nested_config.find(",\"ctest_xml\":");
  ASSERT_NE(nested_ctest, std::string::npos);
  const auto nested_source_close = nested_config.find('}', nested_ctest);
  ASSERT_NE(nested_source_close, std::string::npos);
  nested_config.erase(nested_ctest, nested_source_close - nested_ctest);
  const auto nested_source_end = nested_config.rfind("}]");
  ASSERT_NE(nested_source_end, std::string::npos);
  nested_config.insert(nested_source_end,
    ",\"negative_test\":true,\"expected_failure\":true,"
    "\"expected_diagnostic_code\":\"calibration_promotion\","
    "\"expected_exit_code\":1,\"fixture_sha256\":\"" + fixture_sha + "\"");
  EXPECT_EQ(execute(nested_config, "negative_nested_unsafe"), 1);
  const auto nested_manifest = read(root_ / "negative_nested_unsafe" / "release-manifest.json");
  EXPECT_NE(nested_manifest.find("production or hardware claim detected"), std::string::npos);
}

TEST_F(AuditTest, NegativeEvidenceRejectsUnavailableExitSentinel)
{
  const auto report = std::string(
    "{\"schema_version\":1,\"status\":\"FAIL\",\"production_claim_rejected\":true,"
    "\"production_ready\":true,\"exit_code\":-1,\"fixture_hash\":\"" + std::string(64, 'a') +
    "\",\"commit\":\"" + kHead + "\",\"baseline_main\":\"" + kBase +
    "\",\"synthetic\":true,\"test_only\":true,\"safety_defaults\":{\"serial_enabled\":false,\"dry_run\":true,\"allow_fire\":false,\"fire_command\":0,\"yaw_vel\":0,\"pitch_vel\":0,\"yaw_acc\":0,\"pitch_acc\":0},\"diagnostics\":{\"errors\":[{\"code\":\"calibration_promotion\"}],\"warnings\":[]}}");
  write(root_ / "negative_unavailable_exit.json", report);
  auto value = config((root_ / "negative_unavailable_exit.json").string());
  const auto ctest = value.find(",\"ctest_xml\":");
  ASSERT_NE(ctest, std::string::npos);
  const auto source_close = value.find('}', ctest);
  ASSERT_NE(source_close, std::string::npos);
  value.erase(ctest, source_close - ctest);
  const auto source_end = value.rfind("}]");
  ASSERT_NE(source_end, std::string::npos);
  value.insert(source_end,
    ",\"negative_test\":true,\"expected_failure\":true,"
    "\"expected_diagnostic_code\":\"calibration_promotion\","
    "\"expected_exit_code\":-1,\"fixture_sha256\":\"" + std::string(64, 'a') + "\"");
  EXPECT_EQ(execute(value, "negative_unavailable_exit"), 1);
  const auto manifest = read(root_ / "negative_unavailable_exit" / "release-manifest.json");
  EXPECT_NE(manifest.find("expected/observed exit codes must match and be non-zero"), std::string::npos);
}

TEST_F(AuditTest, RosE2EPassCasesRequireApplicableLivenessEvidence)
{
  const std::string liveness = "\"node_liveness\":{\"alive_before_sampling\":true,\"alive_during_sampling\":true,\"alive_after_sampling\":true,\"expected_exit_code\":0,\"observed_exit_code\":0,\"exit_code_matches\":true}";
  auto valid = safe_report();
  valid.insert(valid.rfind('}'), "," + liveness);
  const std::string old_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\"}]";
  const auto new_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\",\"node_liveness_applicable\":true,\"node_liveness\":{\"alive_before_sampling\":true,\"alive_during_sampling\":true,\"alive_after_sampling\":true,\"expected_exit_code\":0,\"observed_exit_code\":0,\"exit_code_matches\":true}}]";
  ASSERT_NE(valid.find(old_case), std::string::npos);
  valid.replace(valid.find(old_case), old_case.size(), new_case);
  write(root_ / "e2e_liveness.json", valid);
  auto valid_config = config((root_ / "e2e_liveness.json").string());
  valid_config.replace(valid_config.find("release_smoke"), std::string("release_smoke").size(), "ros_e2e");
  EXPECT_EQ(execute(valid_config, "e2e_liveness"), 0);

  auto invalid = valid;
  const auto invalid_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\",\"node_liveness_applicable\":true}]";
  const auto valid_case = "[{\"name\":\"audit_test\",\"status\":\"PASS\",\"node_liveness_applicable\":true,\"node_liveness\":{\"alive_before_sampling\":true,\"alive_during_sampling\":true,\"alive_after_sampling\":true,\"expected_exit_code\":0,\"observed_exit_code\":0,\"exit_code_matches\":true}}]";
  ASSERT_NE(invalid.find(valid_case), std::string::npos);
  invalid.replace(invalid.find(valid_case), std::string(valid_case).size(), invalid_case);
  write(root_ / "e2e_liveness_invalid.json", invalid);
  auto invalid_config = config((root_ / "e2e_liveness_invalid.json").string());
  invalid_config.replace(invalid_config.find("release_smoke"), std::string("release_smoke").size(), "ros_e2e");
  EXPECT_EQ(execute(invalid_config, "e2e_liveness_invalid"), 2);

  const auto sentinel_case =
    "[{\"name\":\"audit_test\",\"status\":\"PASS\","
    "\"node_liveness_applicable\":false,\"node_liveness\":{"
    "\"alive_before_sampling\":false,\"alive_during_sampling\":false,"
    "\"alive_after_sampling\":false,\"expected_exit_code\":-1,"
    "\"observed_exit_code\":-1,\"exit_code_matches\":false}}]";
  auto valid_sentinel = valid;
  const auto valid_case_start = valid_sentinel.find(new_case);
  ASSERT_NE(valid_case_start, std::string::npos);
  valid_sentinel.replace(valid_case_start, std::string(new_case).size(), sentinel_case);
  write(root_ / "e2e_liveness_sentinel.json", valid_sentinel);
  auto sentinel_config = config((root_ / "e2e_liveness_sentinel.json").string());
  sentinel_config.replace(sentinel_config.find("release_smoke"), std::string("release_smoke").size(), "ros_e2e");
  EXPECT_EQ(execute(sentinel_config, "e2e_liveness_sentinel"), 0);

  auto invalid_sentinel = valid_sentinel;
  const auto sentinel_object =
    "\"node_liveness\":{\"alive_before_sampling\":false,\"alive_during_sampling\":false,"
    "\"alive_after_sampling\":false,\"expected_exit_code\":-1,\"observed_exit_code\":-1,"
    "\"exit_code_matches\":false}";
  const auto sentinel_object_start = invalid_sentinel.find(sentinel_object);
  ASSERT_NE(sentinel_object_start, std::string::npos);
  invalid_sentinel.replace(sentinel_object_start, std::string(sentinel_object).size(), "\"node_liveness\":{}");
  write(root_ / "e2e_liveness_invalid_sentinel.json", invalid_sentinel);
  auto invalid_sentinel_config = config((root_ / "e2e_liveness_invalid_sentinel.json").string());
  invalid_sentinel_config.replace(invalid_sentinel_config.find("release_smoke"), std::string("release_smoke").size(), "ros_e2e");
  EXPECT_EQ(execute(invalid_sentinel_config, "e2e_liveness_invalid_sentinel"), 2);
}

}  // namespace
