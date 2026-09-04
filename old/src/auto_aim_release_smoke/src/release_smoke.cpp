#include "auto_aim_release_smoke/release_smoke.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

namespace auto_aim_release_smoke
{
namespace
{
namespace fs = std::filesystem;
using namespace std::chrono_literals;

struct ProcessResult
{
  int exit_code{-1};
  bool timed_out{false};
  bool residual_process{false};
  std::string output;
};

enum class StopMode
{
  Wait,
  InterruptTwice,
  Kill,
};

std::string read_text(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return {};
  }
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

void write_new_file(const fs::path & path, const std::string & contents)
{
  const int descriptor = ::open(
    path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (descriptor < 0) {
    throw std::runtime_error("refusing to overwrite report path: " + path.string());
  }
  std::size_t offset = 0U;
  while (offset < contents.size()) {
    const auto written = ::write(descriptor, contents.data() + offset, contents.size() - offset);
    if (written <= 0) {
      ::close(descriptor);
      throw std::runtime_error("failed to write report path: " + path.string());
    }
    offset += static_cast<std::size_t>(written);
  }
  if (::close(descriptor) != 0) {
    throw std::runtime_error("failed to close report path: " + path.string());
  }
}

std::string command_text(const std::vector<std::string> & arguments)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & argument : arguments) {
    if (!first) {
      stream << ' ';
    }
    first = false;
    const bool quote = argument.empty() || argument.find_first_of(" \t\n\"'") != std::string::npos;
    if (!quote) {
      stream << argument;
      continue;
    }
    stream << '\'';
    for (const char character : argument) {
      if (character == '\'') {
        stream << "'\\''";
      } else {
        stream << character;
      }
    }
    stream << '\'';
  }
  return stream.str();
}

bool process_group_has_live_member(pid_t process_group)
{
  std::error_code error;
  for (const auto & entry : fs::directory_iterator("/proc", error)) {
    if (error) {
      break;
    }
    const auto name = entry.path().filename().string();
    if (name.empty() || !std::all_of(name.begin(), name.end(), ::isdigit)) {
      continue;
    }
    std::ifstream stat(entry.path() / "stat");
    std::string line;
    std::getline(stat, line);
    const auto close = line.rfind(')');
    if (close == std::string::npos || close + 2U >= line.size()) {
      continue;
    }
    std::istringstream fields(line.substr(close + 2U));
    char state = '\0';
    pid_t parent = 0;
    pid_t group = 0;
    fields >> state >> parent >> group;
    if (fields && group == process_group && state != 'Z' && state != 'X') {
      return true;
    }
  }
  return false;
}

bool wait_for_group_exit(pid_t process_group, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!process_group_has_live_member(process_group)) {
      return true;
    }
    std::this_thread::sleep_for(25ms);
  }
  return !process_group_has_live_member(process_group);
}

int wait_status_code(int status)
{
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    return 128 + WTERMSIG(status);
  }
  return -1;
}

ProcessResult run_process(
  const std::vector<std::string> & arguments, const fs::path & log_path,
  std::chrono::seconds timeout = 10s, StopMode stop_mode = StopMode::Wait)
{
  if (arguments.empty()) {
    throw std::invalid_argument("cannot run an empty command");
  }
  const int descriptor = ::open(
    log_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
    S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (descriptor < 0) {
    throw std::runtime_error("cannot create command log: " + log_path.string());
  }

  const pid_t child = ::fork();
  if (child < 0) {
    ::close(descriptor);
    throw std::runtime_error("fork failed for: " + command_text(arguments));
  }
  if (child == 0) {
    ::setpgid(0, 0);
    ::dup2(descriptor, STDOUT_FILENO);
    ::dup2(descriptor, STDERR_FILENO);
    ::close(descriptor);
    std::vector<char *> raw_arguments;
    raw_arguments.reserve(arguments.size() + 1U);
    for (const auto & argument : arguments) {
      raw_arguments.push_back(const_cast<char *>(argument.c_str()));
    }
    raw_arguments.push_back(nullptr);
    ::execvp(raw_arguments.front(), raw_arguments.data());
    ::dprintf(STDERR_FILENO, "exec failed: %s\n", arguments.front().c_str());
    ::_exit(127);
  }

  ::close(descriptor);
  ::setpgid(child, child);
  const auto started = std::chrono::steady_clock::now();
  bool stop_sent = false;
  bool timed_out = false;
  int status = 0;
  while (true) {
    const pid_t waited = ::waitpid(child, &status, WNOHANG);
    if (waited == child) {
      break;
    }
    if (waited < 0) {
      throw std::runtime_error("waitpid failed for: " + command_text(arguments));
    }
    const auto elapsed = std::chrono::steady_clock::now() - started;
    if (!stop_sent && stop_mode != StopMode::Wait && elapsed >= 350ms) {
      ::kill(-child, stop_mode == StopMode::Kill ? SIGKILL : SIGINT);
      if (stop_mode == StopMode::InterruptTwice) {
        ::kill(-child, SIGINT);
      }
      stop_sent = true;
    }
    if (elapsed >= timeout) {
      timed_out = true;
      ::kill(-child, SIGTERM);
      if (!wait_for_group_exit(child, 1s)) {
        ::kill(-child, SIGKILL);
      }
      if (::waitpid(child, &status, 0) < 0) {
        throw std::runtime_error("failed to reap timed-out command: " + command_text(arguments));
      }
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  if (process_group_has_live_member(child)) {
    ::kill(-child, SIGTERM);
    if (!wait_for_group_exit(child, 1s)) {
      ::kill(-child, SIGKILL);
      wait_for_group_exit(child, 1s);
    }
  }
  ProcessResult result;
  result.exit_code = wait_status_code(status);
  result.timed_out = timed_out;
  result.residual_process = process_group_has_live_member(child);
  result.output = read_text(log_path);
  return result;
}

fs::path package_prefix(const fs::path & install_base, const std::string & package)
{
  const auto isolated = install_base / package;
  return fs::is_directory(isolated) ? isolated : install_base;
}

fs::path executable_path(
  const fs::path & install_base, const std::string & package, const std::string & executable)
{
  return package_prefix(install_base, package) / "lib" / package / executable;
}

fs::path share_path(const fs::path & install_base, const std::string & package)
{
  return package_prefix(install_base, package) / "share" / package;
}

bool contains(const std::string & text, const std::string & expected)
{
  return text.find(expected) != std::string::npos;
}

bool has_nonzero_publisher(const std::string & text)
{
  const std::string marker = "Publisher count:";
  const auto position = text.find(marker);
  if (position == std::string::npos) {
    return false;
  }
  const auto digit = text.find_first_of("0123456789", position + marker.size());
  return digit != std::string::npos && text[digit] != '0';
}

std::string markdown_cell(std::string text)
{
  std::replace(text.begin(), text.end(), '|', '/');
  std::replace(text.begin(), text.end(), '\n', ' ');
  return text;
}

std::string environment_description()
{
  struct utsname details {};
  std::ostringstream stream;
  if (::uname(&details) == 0) {
    stream << details.sysname << ' ' << details.release << ' ' << details.machine;
  } else {
    stream << "uname unavailable";
  }
  const char * ros_distro = std::getenv("ROS_DISTRO");
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  stream << "; ROS_DISTRO=" << (ros_distro == nullptr ? "UNSET" : ros_distro) <<
    "; ROS_DOMAIN_ID=" << (domain == nullptr ? "UNSET" : domain);
  std::ifstream os_release("/etc/os-release");
  std::string line;
  while (std::getline(os_release, line)) {
    if (line.rfind("PRETTY_NAME=", 0) == 0) {
      stream << "; " << line.substr(std::string("PRETTY_NAME=").size());
      break;
    }
  }
  return stream.str();
}

void add_path_group(
  std::vector<CaseResult> * results, const std::string & id,
  const std::vector<fs::path> & paths)
{
  std::vector<std::string> missing;
  for (const auto & path : paths) {
    if (!fs::exists(path)) {
      missing.push_back(path.string());
    }
  }
  if (missing.empty()) {
    results->push_back({id, Status::Pass, "all declared installed paths exist"});
    return;
  }
  std::ostringstream summary;
  summary << "missing installed paths:";
  for (const auto & path : missing) {
    summary << ' ' << path;
  }
  results->push_back({id, Status::Fail, summary.str()});
}

void add_command_case(
  std::vector<CaseResult> * results, const fs::path & logs, const std::string & id,
  const std::vector<std::string> & command, const std::vector<int> & accepted_codes,
  const std::string & required_output = {})
{
  const auto result = run_process(command, logs / (id + ".log"));
  const bool code_ok = std::find(
    accepted_codes.begin(), accepted_codes.end(), result.exit_code) != accepted_codes.end();
  const bool output_ok = required_output.empty() || contains(result.output, required_output);
  const bool passed = code_ok && output_ok && !result.timed_out && !result.residual_process;
  std::ostringstream summary;
  summary << (passed ? "command contract satisfied" : "command contract failed") <<
    "; exit=" << result.exit_code;
  if (result.timed_out) {
    summary << "; timed out";
  }
  if (result.residual_process) {
    summary << "; residual process group";
  }
  if (!output_ok) {
    summary << "; missing diagnostic '" << required_output << "'";
  }
  results->push_back(
    {id, passed ? Status::Pass : Status::Fail, summary.str(), command_text(command),
      result.exit_code, result.timed_out});
}

bool any_fail(const std::vector<CaseResult> & results)
{
  return std::any_of(results.begin(), results.end(), [](const CaseResult & result) {
      return result.status == Status::Fail;
    });
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

Status parse_status(const std::string & text)
{
  if (text == "PASS") {return Status::Pass;}
  if (text == "FAIL") {return Status::Fail;}
  if (text == "UNAVAILABLE") {return Status::Unavailable;}
  if (text == "NOT_RUN") {return Status::NotRun;}
  if (text == "NOT_VERIFIED") {return Status::NotVerified;}
  throw std::invalid_argument("unknown smoke status: " + text);
}

bool valid_git_sha(const std::string & text) noexcept
{
  return text.size() == 40U && std::all_of(text.begin(), text.end(), [](unsigned char value) {
      return std::isxdigit(value) != 0;
    });
}

std::string json_escape(const std::string & text)
{
  std::ostringstream stream;
  for (const unsigned char value : text) {
    switch (value) {
      case '\"': stream << "\\\""; break;
      case '\\': stream << "\\\\"; break;
      case '\b': stream << "\\b"; break;
      case '\f': stream << "\\f"; break;
      case '\n': stream << "\\n"; break;
      case '\r': stream << "\\r"; break;
      case '\t': stream << "\\t"; break;
      default:
        if (value < 0x20U) {
          stream << "\\u" << std::hex << std::setw(4) << std::setfill('0') <<
            static_cast<int>(value) << std::dec << std::setfill(' ');
        } else {
          stream << static_cast<char>(value);
        }
    }
  }
  return stream.str();
}

std::string render_json(
  const Options & options, const std::vector<CaseResult> & results,
  const std::string & environment)
{
  std::array<int, 5> counts{};
  for (const auto & result : results) {
    counts.at(static_cast<std::size_t>(result.status))++;
  }
  std::ostringstream stream;
  stream << "{\n"
    << "  \"schema_version\": 1,\n"
    << "  \"status\": \"" << (any_fail(results) ? "FAIL" : "PASS") << "\",\n"
    << "  \"baseline_main\": \"" << json_escape(options.baseline) << "\",\n"
    << "  \"commit\": \"" << json_escape(options.commit) << "\",\n"
    << "  \"environment\": \"" << json_escape(environment) << "\",\n"
    << "  \"install_base\": \"" << json_escape(options.install_base.string()) << "\",\n"
    << "  \"safety_defaults\": {\n"
    << "    \"serial_enabled\": false, \"dry_run\": true, \"allow_fire\": false,\n"
    << "    \"fire_command\": 0, \"yaw_vel\": 0, \"pitch_vel\": 0,\n"
    << "    \"yaw_acc\": 0, \"pitch_acc\": 0\n"
    << "  },\n"
    << "  \"counts\": {\"PASS\": " << counts[0] << ", \"FAIL\": " << counts[1] <<
    ", \"UNAVAILABLE\": " << counts[2] << ", \"NOT_RUN\": " << counts[3] <<
    ", \"NOT_VERIFIED\": " << counts[4] << "},\n"
    << "  \"cases\": [\n";
  for (std::size_t index = 0; index < results.size(); ++index) {
    const auto & result = results[index];
    stream << "    {\"id\": \"" << json_escape(result.id) << "\", \"status\": \"" <<
      status_name(result.status) << "\", \"summary\": \"" << json_escape(result.summary) <<
      "\", \"command\": \"" << json_escape(result.command) << "\", \"exit_code\": " <<
      result.exit_code << ", \"timed_out\": " << (result.timed_out ? "true" : "false") << "}";
    stream << (index + 1U == results.size() ? "\n" : ",\n");
  }
  stream << "  ],\n"
    << "  \"limitations\": [\n"
    << "    \"MVS SDK and a real camera were not verified\",\n"
    << "    \"OpenVINO model inference and the formal model were not verified\",\n"
    << "    \"Orin, CDC serial, robot, gimbal motion, firing, and formal calibration were not verified\",\n"
    << "    \"offline smoke PASS is not hardware validation\"\n"
    << "  ]\n"
    << "}\n";
  return stream.str();
}

std::string render_markdown(
  const Options & options, const std::vector<CaseResult> & results,
  const std::string & environment)
{
  std::ostringstream stream;
  stream << "# Software candidate release smoke\n\n"
    << "- Status: `" << (any_fail(results) ? "FAIL" : "PASS") << "`\n"
    << "- Baseline main: `" << options.baseline << "`\n"
    << "- Commit: `" << options.commit << "`\n"
    << "- Environment: `" << markdown_cell(environment) << "`\n"
    << "- Install base: `" << options.install_base.string() << "`\n\n"
    << "| Case | Status | Exit | Summary |\n"
    << "|---|---|---:|---|\n";
  for (const auto & result : results) {
    stream << "| `" << markdown_cell(result.id) << "` | `" << status_name(result.status) <<
      "` | " << result.exit_code << " | " << markdown_cell(result.summary) << " |\n";
  }
  stream << "\n## Safety defaults\n\n"
    << "`serial_enabled=false`, `dry_run=true`, `allow_fire=false`, `fire_command=0`, "
    << "`yaw_vel=0`, `pitch_vel=0`, `yaw_acc=0`, `pitch_acc=0`.\n\n"
    << "## Limitations\n\n"
    << "MVS SDK, real camera, formal OpenVINO model inference, Orin, CDC serial, robot, "
    << "gimbal motion, firing, and formal calibration were not verified. An offline smoke "
    << "PASS does not mean any of those hardware paths passed.\n";
  return stream.str();
}

int run(const Options & options, std::ostream & output, std::ostream & error)
{
  if (!fs::is_directory(options.install_base)) {
    throw std::invalid_argument("install base is not a directory: " + options.install_base.string());
  }
  if (fs::exists(options.output_dir)) {
    throw std::invalid_argument(
            "output directory must not already exist: " + options.output_dir.string());
  }
  if (!fs::create_directories(options.output_dir / "logs")) {
    throw std::runtime_error("cannot create smoke output directory");
  }
  const auto logs = options.output_dir / "logs";
  const auto artifacts = options.output_dir / "artifacts";
  fs::create_directory(artifacts);
  ::setenv("ROS2CLI_NO_DAEMON", "1", 1);

  std::vector<CaseResult> results;
  results.push_back({
      "package_dependency_resolution", options.rosdep_status,
      options.rosdep_status == Status::Pass ?
      "rosdep declared dependencies resolved" :
      "rosdep was unavailable, not run, or reported unresolved dependencies"});

  const std::vector<std::string> packages{
    "auto_aim_interfaces", "serical_device_ros2", "auto_aim_ros2", "auto_aim_tools",
    "hik_camera", "auto_aim_release_smoke"};
  std::vector<fs::path> manifests;
  for (const auto & package : packages) {
    manifests.push_back(share_path(options.install_base, package) / "package.xml");
  }
  add_path_group(&results, "installed_package_manifests", manifests);

  const std::vector<std::pair<std::string, std::string>> executables{
    {"auto_aim_ros2", "auto_aim_node"},
    {"auto_aim_ros2", "auto_aim_dry_run"},
    {"auto_aim_ros2", "auto_aim_detector_smoke"},
    {"auto_aim_ros2", "auto_aim_pnp_smoke"},
    {"auto_aim_ros2", "auto_aim_offline"},
    {"auto_aim_ros2", "auto_aim_scenario_benchmark"},
    {"auto_aim_ros2", "auto_aim_camera_calibrate"},
    {"auto_aim_tools", "auto_aim_calibration_dataset"},
    {"auto_aim_tools", "auto_aim_calibration_dataset_recorder"},
    {"auto_aim_tools", "ros_input_preflight"},
    {"serical_device_ros2", "vision_pub_node"},
    {"serical_device_ros2", "robot_ctrl_main"},
    {"auto_aim_release_smoke", "auto_aim_release_smoke"},
  };
  std::vector<fs::path> executable_paths;
  for (const auto & executable : executables) {
    executable_paths.push_back(
      executable_path(options.install_base, executable.first, executable.second));
  }
  add_path_group(&results, "installed_ros_executables", executable_paths);

  const auto release_share = share_path(options.install_base, "auto_aim_release_smoke");
  add_path_group(&results, "installed_release_resources", {
      release_share / "docs" / "release_smoke.md",
      release_share / "tools" / "auto_aim_qualification" / "auto_aim_qualification.py",
      release_share / "tools" / "offline_evidence_report" / "auto_aim_evidence_report.py",
      release_share / "tools" / "offline_evidence_report" / "offline_evidence_bundle.py",
      release_share / "fixtures" / "model_profile_test.yaml",
      release_share / "fixtures" / "pnp_test_config.yaml",
      share_path(options.install_base, "hik_camera") / "launch" / "hik_camera.launch.py",
      share_path(options.install_base, "hik_camera") / "config" / "camera_params.yaml",
    });

  const auto auto_aim_prefix = package_prefix(options.install_base, "auto_aim_ros2");
  const auto tools_prefix = package_prefix(options.install_base, "auto_aim_tools");
  const auto auto_exe = [&](const std::string & name) {
      return (auto_aim_prefix / "lib" / "auto_aim_ros2" / name).string();
    };
  const auto tools_exe = [&](const std::string & name) {
      return (tools_prefix / "lib" / "auto_aim_tools" / name).string();
    };

  const std::vector<std::pair<std::string, std::string>> help_commands{
    {"help_auto_aim_dry_run", auto_exe("auto_aim_dry_run")},
    {"help_detector_smoke", auto_exe("auto_aim_detector_smoke")},
    {"help_pnp_smoke", auto_exe("auto_aim_pnp_smoke")},
    {"help_auto_aim_offline", auto_exe("auto_aim_offline")},
    {"help_scenario_benchmark", auto_exe("auto_aim_scenario_benchmark")},
    {"help_camera_calibrate", auto_exe("auto_aim_camera_calibrate")},
    {"help_calibration_dataset", tools_exe("auto_aim_calibration_dataset")},
    {"help_calibration_recorder", tools_exe("auto_aim_calibration_dataset_recorder")},
    {"help_ros_input_preflight", tools_exe("ros_input_preflight")},
  };
  for (const auto & command : help_commands) {
    add_command_case(&results, logs, command.first, {command.second, "--help"}, {0}, "Usage:");
  }

  const auto evidence_script = release_share / "tools" / "offline_evidence_report" /
    "auto_aim_evidence_report.py";
  const auto bundle_script = release_share / "tools" / "offline_evidence_report" /
    "offline_evidence_bundle.py";
  const auto qualification_script = release_share / "tools" / "auto_aim_qualification" /
    "auto_aim_qualification.py";
  add_command_case(
    &results, logs, "help_evidence_report", {"python3", evidence_script.string(), "--help"},
    {0}, "Generate a read-only");
  add_command_case(
    &results, logs, "help_evidence_bundle", {"python3", bundle_script.string(), "--help"},
    {0}, "Build or verify");
  add_command_case(
    &results, logs, "help_qualification", {"python3", qualification_script.string(), "--help"},
    {0}, "Read-only model/profile");

  if (!options.orin_preflight.empty() && fs::exists(options.orin_preflight)) {
    add_command_case(
      &results, logs, "help_orin_preflight", {options.orin_preflight.string(), "--help"}, {0},
      "Usage:");
    const auto orin = run_process(
      {options.orin_preflight.string(), "--repo-root", release_share.string()},
      logs / "orin_environment.log");
    const bool honest = orin.exit_code != 0 && contains(orin.output, "hardware_validation=NOT_RUN") &&
      !orin.residual_process;
    results.push_back({
        "orin_environment", honest ? Status::Unavailable : Status::Fail,
        honest ? "non-Orin WSL environment reported without a hardware PASS" :
        "Orin preflight did not preserve the unavailable boundary",
        command_text({options.orin_preflight.string(), "--repo-root", release_share.string()}),
        orin.exit_code, orin.timed_out});
  } else {
    results.push_back({
        "orin_environment", Status::Unavailable,
        "standalone Orin preflight executable was not supplied"});
  }

  add_command_case(
    &results, logs, "detector_required_inputs_fail_closed",
    {auto_exe("auto_aim_detector_smoke")}, {1}, "--model and --video are required");
  add_command_case(
    &results, logs, "pnp_required_inputs_fail_closed",
    {auto_exe("auto_aim_pnp_smoke")}, {1}, "are required");
  add_command_case(
    &results, logs, "offline_required_inputs_fail_closed",
    {auto_exe("auto_aim_offline")}, {1}, "are required");
  add_command_case(
    &results, logs, "calibration_required_inputs_fail_closed",
    {auto_exe("auto_aim_camera_calibrate")}, {1}, "required");

  const auto detector_missing = run_process({
      auto_exe("auto_aim_detector_smoke"), "--model", "/missing/model.xml", "--video",
      "/missing/input.avi"}, logs / "detector_runtime_missing_artifacts.log");
  const bool detector_failed_closed = detector_missing.exit_code != 0 && !detector_missing.timed_out &&
    !detector_missing.residual_process;
  const bool openvino_unavailable = contains(detector_missing.output, "OpenVINO runtime unavailable");
  results.push_back({
      "detector_runtime_missing_artifacts",
      detector_failed_closed ? (openvino_unavailable ? Status::Unavailable : Status::Pass) : Status::Fail,
      !detector_failed_closed ? "detector did not fail closed" :
      (openvino_unavailable ? "OpenVINO unavailable; runtime artifact path not falsely passed" :
      "missing model/input failed closed"),
      command_text({auto_exe("auto_aim_detector_smoke"), "--model", "/missing/model.xml",
        "--video", "/missing/input.avi"}), detector_missing.exit_code,
      detector_missing.timed_out});

  const auto openvino_probe_model = artifacts / "invalid_openvino_probe.xml";
  write_new_file(openvino_probe_model, "<not-an-openvino-model/>\n");
  const std::vector<std::string> openvino_probe_command{
    auto_exe("auto_aim_detector_smoke"), "--model", openvino_probe_model.string(),
    "--video", "/missing/input.avi"};
  const auto openvino_probe = run_process(
    openvino_probe_command, logs / "openvino_build_support.log");
  const bool openvino_not_built = contains(
    openvino_probe.output, "OpenVINO support is not built");
  const bool openvino_built = contains(
    openvino_probe.output, "OpenVINO model initialization failed");
  const bool openvino_probe_safe = openvino_probe.exit_code != 0 &&
    !openvino_probe.timed_out && !openvino_probe.residual_process;
  results.push_back({
      "openvino_build_support",
      openvino_probe_safe ?
      (openvino_not_built ? Status::Unavailable :
      (openvino_built ? Status::Pass : Status::Fail)) : Status::Fail,
      !openvino_probe_safe ? "OpenVINO capability probe did not fail closed" :
      (openvino_not_built ? "OpenVINO support was not built" :
      (openvino_built ? "OpenVINO support is built; invalid probe model was rejected" :
      "OpenVINO capability probe returned an unrecognized diagnostic")),
      command_text(openvino_probe_command), openvino_probe.exit_code,
      openvino_probe.timed_out});

  const auto scenario_dir = artifacts / "scenario";
  const std::vector<std::string> scenario_command{
    auto_exe("auto_aim_scenario_benchmark"), "--scenario", "all", "--seed", "31",
    "--output-dir", scenario_dir.string()};
  const auto scenario = run_process(scenario_command, logs / "scenario_normal.log", 20s);
  const std::array<std::string, 8> safety_lines{
    "serial_enabled=false", "dry_run=true", "allow_fire=false", "fire_command=0",
    "yaw_vel=0", "pitch_vel=0", "yaw_acc=0", "pitch_acc=0"};
  const bool safety_ok = std::all_of(safety_lines.begin(), safety_lines.end(), [&](const auto & line) {
      return contains(scenario.output, line);
    });
  const bool scenario_ok = scenario.exit_code == 0 && safety_ok &&
    fs::exists(scenario_dir / "benchmark.csv") && fs::exists(scenario_dir / "summary.json") &&
    fs::exists(scenario_dir / "summary.md") && !scenario.residual_process;
  const auto summary_before = read_text(scenario_dir / "summary.json");
  results.push_back({
      "scenario_normal_safe_defaults", scenario_ok ? Status::Pass : Status::Fail,
      scenario_ok ? "synthetic normal fixture emitted all eight safe defaults" :
      "synthetic scenario or safety-default contract failed",
      command_text(scenario_command), scenario.exit_code, scenario.timed_out});
  const auto repeat = run_process(scenario_command, logs / "scenario_existing_output.log");
  const bool no_overwrite = repeat.exit_code == 2 &&
    contains(repeat.output, "must not already exist") &&
    read_text(scenario_dir / "summary.json") == summary_before;
  results.push_back({
      "existing_output_not_overwritten", no_overwrite ? Status::Pass : Status::Fail,
      no_overwrite ? "existing output directory was rejected without mutation" :
      "existing output protection failed", command_text(scenario_command), repeat.exit_code,
      repeat.timed_out});

  const auto fixture_root = release_share / "tools" / "offline_evidence_report" / "fixtures";
  const auto normal_json = artifacts / "normal_evidence.json";
  const auto normal_md = artifacts / "normal_evidence.md";
  add_command_case(
    &results, logs, "evidence_normal_fixture",
    {"python3", evidence_script.string(), "--input-csv", (fixture_root / "normal.csv").string(),
      "--json-report", normal_json.string(), "--markdown-report", normal_md.string()},
    {0}, "status=PASS");
  const auto anomaly_json = artifacts / "anomaly_evidence.json";
  const auto anomaly_md = artifacts / "anomaly_evidence.md";
  add_command_case(
    &results, logs, "evidence_failure_fixture",
    {"python3", evidence_script.string(), "--input-csv", (fixture_root / "anomaly.csv").string(),
      "--json-report", anomaly_json.string(), "--markdown-report", anomaly_md.string()},
    {1}, "status=FAIL");

  const auto bundle_dir = artifacts / "evidence_bundle";
  add_command_case(
    &results, logs, "evidence_bundle_fixture",
    {"python3", bundle_script.string(), "--input-csv", (fixture_root / "normal.csv").string(),
      "--output-dir", bundle_dir.string(), "--mode", "evidence_only"},
    {0, 2}, "status=");

  const auto qualification_json = artifacts / "qualification.json";
  const auto qualification_md = artifacts / "qualification.md";
  add_command_case(
    &results, logs, "qualification_evidence_only_fixture",
    {"python3", qualification_script.string(), "--mode", "evidence_only", "--allow-test-only",
      "--model-profile", (release_share / "fixtures" / "model_profile_test.yaml").string(),
      "--pnp-config", (release_share / "fixtures" / "pnp_test_config.yaml").string(),
      "--input-csv", (fixture_root / "normal.csv").string(), "--output-json",
      qualification_json.string(), "--output-markdown", qualification_md.string()},
    {2}, "status=WARN");
  add_command_case(
    &results, logs, "qualification_missing_inputs_fail_closed",
    {"python3", qualification_script.string(), "--model-profile", "/missing/profile.yaml",
      "--pnp-config", "/missing/pnp.yaml", "--output-json",
      (artifacts / "qualification_missing.json").string(), "--output-markdown",
      (artifacts / "qualification_missing.md").string()},
    {1}, "status=FAIL");

  add_command_case(
    &results, logs, "installed_hik_launch_arguments",
    {"ros2", "launch", "hik_camera", "hik_camera.launch.py", "--show-args"}, {0},
    "camera_info_url");

  const auto graph_before = run_process(
    {"ros2", "topic", "info", "/Robot_ctrl_data", "--verbose"},
    logs / "robot_ctrl_graph_before.log");
  const bool publisher_before = has_nonzero_publisher(graph_before.output);
  const auto graph_after = run_process(
    {"ros2", "topic", "info", "/Robot_ctrl_data", "--verbose"},
    logs / "robot_ctrl_graph_after.log");
  const bool publisher_after = has_nonzero_publisher(graph_after.output);
  results.push_back({
      "offline_no_robot_ctrl_publisher", !publisher_before && !publisher_after ?
      Status::Pass : Status::Fail,
      !publisher_before && !publisher_after ?
      "isolated ROS domain had no /Robot_ctrl_data publisher before or after offline CLIs" :
      "a /Robot_ctrl_data publisher was observed in the isolated smoke domain",
      "ros2 topic info /Robot_ctrl_data --verbose", graph_after.exit_code,
      graph_before.timed_out || graph_after.timed_out});

  const auto preflight = tools_exe("ros_input_preflight");
  const auto natural = run_process({
      preflight, "--duration", "0.2", "--timeout", "0.05", "--format", "json",
      "--output", (artifacts / "preflight_natural.json").string()},
    logs / "process_natural_stop.log", 10s);
  const bool natural_ok = natural.exit_code == 2 && !natural.timed_out && !natural.residual_process &&
    fs::exists(artifacts / "preflight_natural.json");
  results.push_back({
      "process_natural_stop", natural_ok ? Status::Pass : Status::Fail,
      natural_ok ? "no-input preflight exited fail-closed and was reaped" :
      "natural stop left an invalid result or residual process", preflight,
      natural.exit_code, natural.timed_out});

  const auto repeated = run_process({
      preflight, "--duration", "30", "--timeout", "1", "--format", "json",
      "--output", (artifacts / "preflight_repeated_stop.json").string()},
    logs / "process_repeated_stop.log", 8s, StopMode::InterruptTwice);
  const bool repeated_ok = repeated.exit_code == 130 && !repeated.timed_out &&
    !repeated.residual_process;
  results.push_back({
      "process_repeated_stop", repeated_ok ? Status::Pass : Status::Fail,
      repeated_ok ? "repeated SIGINT was idempotent and the process group was reaped" :
      "repeated stop contract failed", preflight, repeated.exit_code, repeated.timed_out});

  const auto killed = run_process({
      preflight, "--duration", "30", "--timeout", "1", "--format", "json",
      "--output", (artifacts / "preflight_killed.json").string()},
    logs / "process_abnormal_exit.log", 8s, StopMode::Kill);
  const bool killed_ok = killed.exit_code == 137 && !killed.timed_out && !killed.residual_process;
  results.push_back({
      "process_abnormal_exit", killed_ok ? Status::Pass : Status::Fail,
      killed_ok ? "SIGKILL fixture was reaped without a residual process group" :
      "abnormal-exit cleanup contract failed", preflight, killed.exit_code, killed.timed_out});

  const auto camera_node = executable_path(options.install_base, "hik_camera", "hik_camera_node");
  results.push_back({
      "mvs_camera_executable", fs::exists(camera_node) ? Status::Pass : Status::Unavailable,
      fs::exists(camera_node) ? "MVS-backed camera executable is installed" :
      "MVS SDK libraries absent; real camera executable intentionally not installed"});
  results.push_back({
      "real_camera_launch_runtime", Status::NotRun,
      "real camera launch was intentionally not run without MVS SDK and hardware"});

  for (const auto & item : std::array<std::pair<const char *, const char *>, 8>{
      std::pair{"real_camera", "no real camera was connected"},
      std::pair{"orin", "not running on an Orin target"},
      std::pair{"cdc_serial", "no CDC serial device was opened"},
      std::pair{"robot", "no robot was connected"},
      std::pair{"gimbal_motion", "gimbal motion was not exercised"},
      std::pair{"firing", "firing was not enabled or exercised"},
      std::pair{"formal_calibration", "formal K/D and extrinsics were not supplied"},
      std::pair{"formal_model", "formal model inference was not executed"},
    }) {
    results.push_back({std::string("hardware_") + item.first, Status::NotVerified, item.second});
  }

  const auto environment = environment_description();
  write_new_file(options.output_dir / "smoke-report.json", render_json(options, results, environment));
  write_new_file(
    options.output_dir / "smoke-report.md", render_markdown(options, results, environment));
  const bool failed = any_fail(results);
  output << "status=" << (failed ? "FAIL" : "PASS") << '\n'
         << "json=" << (options.output_dir / "smoke-report.json") << '\n'
         << "markdown=" << (options.output_dir / "smoke-report.md") << '\n';
  if (failed) {
    error << "auto_aim_release_smoke: one or more required software cases failed\n";
  }
  return failed ? 1 : 0;
}

}  // namespace auto_aim_release_smoke
