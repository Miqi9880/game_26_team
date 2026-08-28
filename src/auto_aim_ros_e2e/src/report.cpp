#include "auto_aim_ros_e2e/report.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <fcntl.h>
#include <openssl/evp.h>
#include <sys/stat.h>
#include <unistd.h>

namespace auto_aim_ros_e2e
{
namespace
{

std::size_t status_index(Status status) noexcept
{
  return static_cast<std::size_t>(status);
}

bool any_failure(const std::vector<CaseResult> & results)
{
  return std::any_of(results.begin(), results.end(), [](const CaseResult & result) {
      return result.status == Status::Fail;
    });
}

bool report_passes(
  const ReportMetadata & metadata, const std::vector<CaseResult> & results) noexcept
{
  return !any_failure(results) && node_liveness_valid(metadata.node_liveness);
}

std::string markdown_cell(std::string value)
{
  std::replace(value.begin(), value.end(), '|', '/');
  std::replace(value.begin(), value.end(), '\n', ' ');
  return value;
}

std::array<std::size_t, 5> counts(const std::vector<CaseResult> & results)
{
  std::array<std::size_t, 5> result{};
  for (const auto & item : results) {
    ++result.at(status_index(item.status));
  }
  return result;
}

}  // namespace

const char * status_name(Status status) noexcept
{
  switch (status) {
    case Status::Pass: return "PASS";
    case Status::Fail: return "FAIL";
    case Status::Unavailable: return "UNAVAILABLE";
    case Status::NotRun: return "NOT_RUN";
    case Status::NotVerified: return "NOT_VERIFIED";
  }
  return "FAIL";
}

bool valid_git_sha(const std::string & value) noexcept
{
  return value.size() == 40U &&
         std::all_of(value.begin(), value.end(), [](const unsigned char character) {
           return std::isxdigit(character) != 0;
         });
}

bool node_liveness_valid(const NodeLiveness & liveness) noexcept
{
  return liveness.alive_before_sampling && liveness.alive_during_sampling &&
         liveness.alive_after_sampling && liveness.expected_exit_code >= 0 &&
         liveness.observed_exit_code == liveness.expected_exit_code &&
         liveness.exit_code_matches;
}

std::string json_escape(const std::string & value)
{
  std::ostringstream stream;
  for (const unsigned char character : value) {
    switch (character) {
      case '"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (character < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<int>(character) << std::dec << std::setfill(' ');
        } else {
          stream << static_cast<char>(character);
        }
    }
  }
  return stream.str();
}

std::string sha256_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("cannot hash file: " + path.string());
  }
  EVP_MD_CTX * context = EVP_MD_CTX_new();
  if (context == nullptr || EVP_DigestInit_ex(context, EVP_sha256(), nullptr) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("cannot initialize SHA-256");
  }
  std::array<char, 64 * 1024> buffer{};
  while (input) {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto size = input.gcount();
    if (size > 0 &&
      EVP_DigestUpdate(context, buffer.data(), static_cast<std::size_t>(size)) != 1)
    {
      EVP_MD_CTX_free(context);
      throw std::runtime_error("cannot update SHA-256 for: " + path.string());
    }
  }
  if (!input.eof()) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("cannot read file for SHA-256: " + path.string());
  }
  std::array<unsigned char, EVP_MAX_MD_SIZE> digest{};
  unsigned int digest_size = 0U;
  if (EVP_DigestFinal_ex(context, digest.data(), &digest_size) != 1) {
    EVP_MD_CTX_free(context);
    throw std::runtime_error("cannot finalize SHA-256 for: " + path.string());
  }
  EVP_MD_CTX_free(context);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (unsigned int index = 0U; index < digest_size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(digest[index]);
  }
  return output.str();
}

void write_new_file(const std::filesystem::path & path, const std::string & contents)
{
  const int descriptor = ::open(
    path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
    S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (descriptor < 0) {
    throw std::runtime_error("refusing to overwrite file: " + path.string());
  }
  std::size_t offset = 0U;
  while (offset < contents.size()) {
    const auto written = ::write(descriptor, contents.data() + offset, contents.size() - offset);
    if (written <= 0) {
      ::close(descriptor);
      throw std::runtime_error("cannot write file: " + path.string());
    }
    offset += static_cast<std::size_t>(written);
  }
  if (::close(descriptor) != 0) {
    throw std::runtime_error("cannot close file: " + path.string());
  }
}

std::string render_json(
  const ReportMetadata & metadata, const std::vector<CaseResult> & results,
  const std::vector<Artifact> & artifacts)
{
  const auto totals = counts(results);
  std::ostringstream stream;
  stream << "{\n"
    << "  \"schema_version\": 1,\n"
    << "  \"suite\": \"auto_aim_ros_message_e2e\",\n"
    << "  \"status\": \"" << (report_passes(metadata, results) ? "PASS" : "FAIL") << "\",\n"
    << "  \"baseline_main\": \"" << json_escape(metadata.baseline) << "\",\n"
    << "  \"candidate_commit\": \"" << json_escape(metadata.commit) << "\",\n"
    << "  \"run_id\": \"" << json_escape(metadata.suite_run_id) << "\",\n"
    << "  \"environment\": \"" << json_escape(metadata.environment) << "\",\n"
    << "  \"ros_domain_id\": \"" << json_escape(metadata.ros_domain_id) << "\",\n"
    << "  \"synthetic_seed\": \"" << json_escape(metadata.seed) << "\",\n"
    << "  \"rounds\": " << metadata.rounds << ",\n"
    << "  \"message_contract\": {\n"
    << "    \"image\": {\"topic\": \"/image_raw (uniquely remapped per case)\", "
       "\"type\": \"sensor_msgs/msg/Image\", \"qos\": \"SensorDataQoS: best_effort, volatile, keep_last(5)\", "
       "\"encoding\": \"rgb8\", \"width\": 4, \"height\": 3, \"step\": 12, "
       "\"data_size\": 36, \"frame_id\": \"camera_optical_frame\", "
       "\"timestamp_source\": \"fixed synthetic sequence from seed\"},\n"
    << "    \"camera_info\": {\"topic\": \"/camera_info (uniquely remapped per case)\", "
       "\"type\": \"sensor_msgs/msg/CameraInfo\", \"qos\": \"SensorDataQoS\", "
       "\"width\": 4, \"height\": 3, \"frame_id\": \"camera_optical_frame\"},\n"
    << "    \"control\": {\"topic\": \"/Robot_ctrl_data (uniquely remapped per case)\", "
       "\"publisher\": \"existing AutoAimNode only\", "
       "\"observer\": \"test-only subscriber; no serial path\"}\n"
    << "  },\n"
    << "  \"safety_assertions\": {\"serial_enabled\": false, \"dry_run\": true, "
       "\"allow_fire\": false, \"fire_command\": 0, \"yaw_vel\": 0, "
       "\"pitch_vel\": 0, \"yaw_acc\": 0, \"pitch_acc\": 0},\n"
    << "  \"node_liveness\": {\n"
    << "    \"alive_before_sampling\": "
    << (metadata.node_liveness.alive_before_sampling ? "true" : "false") << ",\n"
    << "    \"alive_during_sampling\": "
    << (metadata.node_liveness.alive_during_sampling ? "true" : "false") << ",\n"
    << "    \"alive_after_sampling\": "
    << (metadata.node_liveness.alive_after_sampling ? "true" : "false") << ",\n"
    << "    \"expected_exit_code\": " << metadata.node_liveness.expected_exit_code << ",\n"
    << "    \"observed_exit_code\": " << metadata.node_liveness.observed_exit_code << ",\n"
    << "    \"exit_code_matches\": "
    << (metadata.node_liveness.exit_code_matches ? "true" : "false") << "\n"
    << "  },\n"
    << "  \"counts\": {\"PASS\": " << totals[0] << ", \"FAIL\": " << totals[1]
    << ", \"UNAVAILABLE\": " << totals[2] << ", \"NOT_RUN\": " << totals[3]
    << ", \"NOT_VERIFIED\": " << totals[4] << "},\n"
    << "  \"cases\": [\n";
  for (std::size_t index = 0U; index < results.size(); ++index) {
    const auto & item = results[index];
    stream << "    {\"round\": " << item.round << ", \"id\": \"" << json_escape(item.id)
      << "\", \"run_id\": \"" << json_escape(item.run_id) << "\", \"status\": \""
      << status_name(item.status) << "\", \"input_summary\": \""
      << json_escape(item.input_summary) << "\", \"expected\": \""
      << json_escape(item.expected) << "\", \"actual\": \"" << json_escape(item.actual)
      << "\", \"diagnostic\": \"" << json_escape(item.diagnostic)
      << "\", \"node_exit_code\": " << item.node_exit_code
      << ", \"preflight_exit_code\": " << item.preflight_exit_code
      << ", \"node_liveness\": {\"alive_before_sampling\": "
      << (item.node_liveness.alive_before_sampling ? "true" : "false")
      << ", \"alive_during_sampling\": "
      << (item.node_liveness.alive_during_sampling ? "true" : "false")
      << ", \"alive_after_sampling\": "
      << (item.node_liveness.alive_after_sampling ? "true" : "false")
      << ", \"expected_exit_code\": " << item.node_liveness.expected_exit_code
      << ", \"observed_exit_code\": " << item.node_liveness.observed_exit_code
      << ", \"exit_code_matches\": "
      << (item.node_liveness.exit_code_matches ? "true" : "false") << "}"
      << ", \"topics\": \"" << json_escape(item.topics) << "\", \"publishers\": \""
      << json_escape(item.publishers) << "\", \"control_messages\": "
      << item.control_messages << ", \"safety_fields_ok\": "
      << (item.safety_fields_ok ? "true" : "false") << ", \"target_lock\": \""
      << json_escape(item.target_lock) << "\", \"cleanup_ok\": "
      << (item.cleanup_ok ? "true" : "false") << ", \"deterministic\": "
      << (item.deterministic ? "true" : "false") << ", \"flaky\": "
      << (item.flaky ? "true" : "false") << ", \"rerun\": "
      << (item.rerun ? "true" : "false") << ", \"artifacts\": [";
    for (std::size_t artifact_index = 0U; artifact_index < item.artifacts.size(); ++artifact_index) {
      const auto & artifact = item.artifacts[artifact_index];
      stream << "{\"path\": \"" << json_escape(artifact.path) << "\", \"role\": \""
        << json_escape(artifact.role) << "\", \"sha256\": \""
        << json_escape(artifact.sha256) << "\"}"
        << (artifact_index + 1U == item.artifacts.size() ? "" : ", ");
    }
    stream << "]}" << (index + 1U == results.size() ? "\n" : ",\n");
  }
  stream << "  ],\n  \"artifact_manifest\": [\n";
  for (std::size_t index = 0U; index < artifacts.size(); ++index) {
    const auto & artifact = artifacts[index];
    stream << "    {\"path\": \"" << json_escape(artifact.path) << "\", \"role\": \""
      << json_escape(artifact.role) << "\", \"sha256\": \""
      << json_escape(artifact.sha256) << "\"}"
      << (index + 1U == artifacts.size() ? "\n" : ",\n");
  }
  stream << "  ],\n"
    << "  \"unverified\": [\"MVS SDK\", \"real camera\", \"Orin\", "
       "\"formal model and calibration\", \"CDC serial\", \"robot\", "
       "\"gimbal motion\", \"firing\", \"competition performance\"],\n"
    << "  \"hash_scope\": \"artifact_manifest covers all pre-report inputs/outputs; SHA256SUMS also covers the JSON and Markdown reports and excludes only itself\",\n"
    << "  \"scope_notice\": \"Message-level dry-run E2E PASS is not hardware or competition validation.\"\n"
    << "}\n";
  return stream.str();
}

std::string render_markdown(
  const ReportMetadata & metadata, const std::vector<CaseResult> & results,
  const std::vector<Artifact> & artifacts)
{
  const auto totals = counts(results);
  std::ostringstream stream;
  stream << "# ROS message-level dry-run E2E report\n\n"
    << "- Status: `" << (report_passes(metadata, results) ? "PASS" : "FAIL") << "`\n"
    << "- Baseline main: `" << metadata.baseline << "`\n"
    << "- Candidate commit: `" << metadata.commit << "`\n"
    << "- Run ID: `" << metadata.suite_run_id << "`\n"
    << "- Environment: `" << markdown_cell(metadata.environment) << "`\n"
    << "- ROS_DOMAIN_ID: `" << metadata.ros_domain_id << "`\n"
    << "- Synthetic seed: `" << metadata.seed << "`\n"
    << "- Full-matrix rounds: `" << metadata.rounds << "`\n"
    << "- Counts: PASS=" << totals[0] << ", FAIL=" << totals[1]
    << ", UNAVAILABLE=" << totals[2] << ", NOT_RUN=" << totals[3]
    << ", NOT_VERIFIED=" << totals[4] << "\n\n"
    << "## Node liveness\n\n"
    << "| Check | Observed |\n|---|---|\n"
    << "| Alive before sampling | `"
    << (metadata.node_liveness.alive_before_sampling ? "true" : "false") << "` |\n"
    << "| Alive during sampling | `"
    << (metadata.node_liveness.alive_during_sampling ? "true" : "false") << "` |\n"
    << "| Alive after sampling | `"
    << (metadata.node_liveness.alive_after_sampling ? "true" : "false") << "` |\n"
    << "| Expected controlled-stop exit | `" << metadata.node_liveness.expected_exit_code
    << "` |\n"
    << "| Observed controlled-stop exit | `" << metadata.node_liveness.observed_exit_code
    << "` |\n"
    << "| Exit code matches | `"
    << (metadata.node_liveness.exit_code_matches ? "true" : "false") << "` |\n\n"
    << "## Cases\n\n"
    << "| Round | Case | Status | Node / Preflight exit | Controls | Cleanup | Summary |\n"
    << "|---:|---|---|---|---:|---|---|\n";
  for (const auto & item : results) {
    stream << "| " << item.round << " | `" << markdown_cell(item.id) << "` | `"
      << status_name(item.status) << "` | " << item.node_exit_code << " / "
      << item.preflight_exit_code << " | " << item.control_messages << " | "
      << (item.cleanup_ok ? "PASS" : "FAIL") << " | "
      << markdown_cell(item.actual + "; " + item.diagnostic) << " |\n";
  }
  stream << "\n## Contract and safety\n\n"
    << "The test-only fixture node publishes uniquely remapped `sensor_msgs/msg/Image`, "
       "`sensor_msgs/msg/CameraInfo`, and bookkeeping-only `Vision` messages. It only "
       "subscribes to the remapped RobotCtrl output and is not connected to the real sender. "
       "The read-only preflight creates no RobotCtrl publisher.\n\n"
    << "All node launches enforce `serial_enabled=false`, `dry_run=true`, "
       "`allow_fire=false`. Every observed control in required safe cases has "
       "`fire_command=0`, `yaw_vel=0`, `pitch_vel=0`, `yaw_acc=0`, and `pitch_acc=0`. "
       "Target lock is checked against the existing 49=locked / 50=unlocked semantics.\n\n"
    << "## Artifact SHA-256\n\n"
    << "| Path | Role | SHA-256 |\n|---|---|---|\n";
  for (const auto & artifact : artifacts) {
    stream << "| `" << markdown_cell(artifact.path) << "` | "
      << markdown_cell(artifact.role) << " | `" << artifact.sha256 << "` |\n";
  }
  stream << "\n## Not verified\n\n"
    << "MVS SDK, real camera, Orin, formal model/calibration, CDC serial, robot, gimbal "
       "motion, firing, and competition performance were not verified. Message-level "
       "dry-run E2E PASS does not validate any of those paths.\n\n"
    << "The artifact table covers all files created before report rendering. `SHA256SUMS` "
       "also hashes both reports and excludes only itself to avoid a recursive hash.\n";
  return stream.str();
}

}  // namespace auto_aim_ros_e2e
