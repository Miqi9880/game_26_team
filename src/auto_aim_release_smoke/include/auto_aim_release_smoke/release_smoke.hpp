#pragma once

#include <filesystem>
#include <iosfwd>
#include <string>
#include <utility>
#include <vector>

namespace auto_aim_release_smoke
{

enum class Status
{
  Pass,
  Fail,
  Unavailable,
  NotRun,
  NotVerified,
};

struct CaseResult
{
  CaseResult(
    std::string input_id, Status input_status, std::string input_summary,
    std::string input_command = {}, int input_exit_code = -1,
    bool input_timed_out = false)
  : id(std::move(input_id)), status(input_status), summary(std::move(input_summary)),
    command(std::move(input_command)), exit_code(input_exit_code), timed_out(input_timed_out)
  {}

  std::string id;
  Status status{Status::Fail};
  std::string summary;
  std::string command;
  int exit_code{-1};
  bool timed_out{false};
};

struct Options
{
  std::filesystem::path install_base;
  std::filesystem::path output_dir;
  std::filesystem::path orin_preflight;
  std::string baseline;
  std::string commit;
  Status rosdep_status{Status::NotRun};
};

const char * status_name(Status status) noexcept;
Status parse_status(const std::string & text);
bool valid_git_sha(const std::string & text) noexcept;
std::string json_escape(const std::string & text);
std::string render_json(
  const Options & options, const std::vector<CaseResult> & results,
  const std::string & environment);
std::string render_markdown(
  const Options & options, const std::vector<CaseResult> & results,
  const std::string & environment);
int run(const Options & options, std::ostream & output, std::ostream & error);

}  // namespace auto_aim_release_smoke
