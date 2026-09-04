#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace auto_aim_ros_e2e
{

enum class Status
{
  Pass,
  Fail,
  Unavailable,
  NotRun,
  NotVerified,
};

struct Artifact
{
  std::string path;
  std::string role;
  std::string sha256;
};

// Liveness evidence for the AutoAimNode process that is sampled by the
// message-level runner.  All fields are intentionally fail-closed: a report
// that does not establish each phase and an equal controlled-stop exit code
// cannot be treated as a PASS by downstream release audits.
struct NodeLiveness
{
  bool alive_before_sampling{false};
  bool alive_during_sampling{false};
  bool alive_after_sampling{false};
  int expected_exit_code{-1};
  int observed_exit_code{-1};
  bool exit_code_matches{false};
};

struct CaseResult
{
  std::size_t round{0U};
  std::string id;
  std::string run_id;
  Status status{Status::Fail};
  std::string input_summary;
  std::string expected;
  std::string actual;
  std::string diagnostic;
  int node_exit_code{-1};
  int preflight_exit_code{-1};
  // True only for cases that actually launch and sample AutoAimNode. Other
  // lifecycle, expected-failure, and unavailable-boundary cases intentionally
  // have no node liveness contract.
  bool node_liveness_applicable{false};
  NodeLiveness node_liveness{};
  std::string topics;
  std::string publishers;
  std::size_t control_messages{0U};
  bool safety_fields_ok{false};
  std::string target_lock;
  bool cleanup_ok{false};
  bool deterministic{true};
  bool flaky{false};
  bool rerun{false};
  std::vector<Artifact> artifacts;
};

struct ReportMetadata
{
  std::string baseline;
  std::string commit;
  std::string suite_run_id;
  std::string environment;
  std::string ros_domain_id;
  std::string seed;
  std::size_t rounds{0U};
  NodeLiveness node_liveness{};
};

const char * status_name(Status status) noexcept;
bool valid_git_sha(const std::string & value) noexcept;
bool node_liveness_valid(const NodeLiveness & liveness) noexcept;
std::string json_escape(const std::string & value);
std::string sha256_file(const std::filesystem::path & path);
std::string render_json(
  const ReportMetadata & metadata, const std::vector<CaseResult> & results,
  const std::vector<Artifact> & artifacts);
std::string render_markdown(
  const ReportMetadata & metadata, const std::vector<CaseResult> & results,
  const std::vector<Artifact> & artifacts);
void write_new_file(const std::filesystem::path & path, const std::string & contents);

}  // namespace auto_aim_ros_e2e
