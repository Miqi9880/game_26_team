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
};

const char * status_name(Status status) noexcept;
bool valid_git_sha(const std::string & value) noexcept;
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
