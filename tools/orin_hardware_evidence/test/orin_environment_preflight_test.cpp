#include "orin_hardware_evidence/orin_environment_preflight.hpp"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <cstdlib>
#endif

namespace
{

int failures = 0;

void set_mvs_environment(const std::string & value)
{
#if defined(_WIN32)
  _putenv_s("HIK_MVS_LIBRARY_DIR", value.c_str());
#else
  setenv("HIK_MVS_LIBRARY_DIR", value.c_str(), 1);
#endif
}

void clear_mvs_environment()
{
#if defined(_WIN32)
  _putenv_s("HIK_MVS_LIBRARY_DIR", "");
#else
  unsetenv("HIK_MVS_LIBRARY_DIR");
#endif
}

void expect(const bool condition, const std::string & message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

orin_hardware_evidence::EnvironmentSnapshot complete_orin_snapshot()
{
  orin_hardware_evidence::EnvironmentSnapshot snapshot;
  snapshot.os = "linux";
  snapshot.architecture = "aarch64";
  snapshot.device_model = "NVIDIA Jetson AGX Orin Developer Kit";
  snapshot.dependencies = {true, true, true, true, true, true};
  return snapshot;
}

void test_complete_metadata_is_not_hardware_validation()
{
  const auto snapshot = complete_orin_snapshot();
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  expect(evaluation.target_environment, "Orin snapshot should be a target environment");
  expect(evaluation.dependencies_complete, "complete dependencies should be recognized");
  expect(evaluation.exit_code == 0, "complete metadata should use exit code 0");

  std::ostringstream report;
  orin_hardware_evidence::print_report(report, snapshot, evaluation, {}, {});
  expect(
    report.str().find("overall=ENVIRONMENT_PREFLIGHT_COMPLETE_NOT_HARDWARE_VALIDATED") !=
    std::string::npos,
    "complete metadata must not claim hardware validation");
  expect(
    report.str().find("hardware_validation=NOT_RUN") != std::string::npos,
    "report must state hardware validation was not run");
}

void test_wsl_is_non_target()
{
  auto snapshot = complete_orin_snapshot();
  snapshot.is_wsl = true;
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  expect(!evaluation.target_environment, "WSL must be rejected as non-target");
  expect(evaluation.exit_code == 2, "non-target with dependencies should use exit code 2");
}

void test_non_orin_architecture_is_non_target()
{
  auto snapshot = complete_orin_snapshot();
  snapshot.architecture = "x86_64";
  snapshot.device_model.clear();
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  expect(!evaluation.target_environment, "x86_64 without Orin evidence must be non-target");
  expect(evaluation.target_reasons.size() == 2, "both architecture and model reasons are required");
}

void test_missing_dependencies_are_explicit()
{
  auto snapshot = complete_orin_snapshot();
  snapshot.dependencies.openvino = false;
  snapshot.dependencies.mvs_arm64_libraries = false;
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  expect(!evaluation.dependencies_complete, "missing dependencies must not be complete");
  expect(evaluation.exit_code == 3, "target with missing dependencies should use exit code 3");
  expect(evaluation.missing_dependencies.size() == 2, "all missing dependencies must be listed");

  std::ostringstream report;
  orin_hardware_evidence::print_report(report, snapshot, evaluation, {}, {});
  expect(
    report.str().find("MISSING_DEPENDENCIES") != std::string::npos,
    "missing dependency marker must be printed");
}

void test_safety_defaults_and_no_default_device_probe()
{
  auto snapshot = complete_orin_snapshot();
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  const auto camera = orin_hardware_evidence::inspect_device_permissions("camera", std::nullopt);
  const auto serial = orin_hardware_evidence::inspect_device_permissions("serial", std::nullopt);
  expect(!camera.requested && !serial.requested, "devices must not be probed by default");

  std::ostringstream report;
  orin_hardware_evidence::print_report(report, snapshot, evaluation, camera, serial);
  const auto text = report.str();
  expect(text.find("serial_enabled=false") != std::string::npos, "serial must remain disabled");
  expect(text.find("dry_run=true") != std::string::npos, "dry-run must remain enabled");
  expect(text.find("allow_fire=false") != std::string::npos, "fire authorization must be false");
  expect(text.find("fire_command=0") != std::string::npos, "fire command must remain zero");
  expect(
    text.find("camera_opened=false") != std::string::npos &&
    text.find("serial_opened=false") != std::string::npos,
    "report must state devices were not opened");
}

void test_mvs_search_paths_match_camera_cmake()
{
  clear_mvs_environment();
  const auto candidates = orin_hardware_evidence::mvs_library_search_paths(MVS_TEST_REPO);
  expect(candidates.size() == 2, "default MVS search should contain two paths");
  expect(
    candidates[0].find("src/ros2-hik-camera/hikSDK/lib/arm64") != std::string::npos,
    "repository arm64 SDK path must be searched");
  expect(candidates[1] == "/opt/MVS/lib/aarch64", "system aarch64 MVS path must be searched");
}

void test_repository_mvs_directory_is_detected()
{
  clear_mvs_environment();
  const auto candidates = orin_hardware_evidence::mvs_library_search_paths(MVS_TEST_REPO);
  const auto match = orin_hardware_evidence::find_complete_mvs_library_directory(candidates);
  expect(
    match == std::string(MVS_TEST_REPO) + "/src/ros2-hik-camera/hikSDK/lib/arm64",
    "complete repository MVS directory must be reported");
}

void test_opt_layout_mvs_directory_is_detected()
{
  const auto match = orin_hardware_evidence::find_complete_mvs_library_directory(
    {"/missing/repository/path", MVS_TEST_OPT_LIB});
  expect(match == MVS_TEST_OPT_LIB, "complete /opt-style MVS directory must be reported");
}

void test_explicit_mvs_directory_takes_priority()
{
  set_mvs_environment("/environment/mvs/path");
  const auto candidates = orin_hardware_evidence::mvs_library_search_paths(
    MVS_TEST_REPO, MVS_TEST_EXPLICIT_LIB);
  expect(candidates.front() == MVS_TEST_EXPLICIT_LIB, "explicit MVS directory must be first");
  expect(
    std::find(candidates.begin(), candidates.end(), "/environment/mvs/path") == candidates.end(),
    "explicit MVS directory must override the environment variable");
  const auto match = orin_hardware_evidence::find_complete_mvs_library_directory(candidates);
  expect(match == MVS_TEST_EXPLICIT_LIB, "explicit complete MVS directory must be reported");
  clear_mvs_environment();
}

void test_environment_mvs_directory_is_supported()
{
  set_mvs_environment(MVS_TEST_EXPLICIT_LIB);
  const auto candidates = orin_hardware_evidence::mvs_library_search_paths(MVS_TEST_REPO);
  expect(candidates.front() == MVS_TEST_EXPLICIT_LIB, "environment MVS directory must be first");
  const auto match = orin_hardware_evidence::find_complete_mvs_library_directory(candidates);
  expect(match == MVS_TEST_EXPLICIT_LIB, "environment MVS directory must be reported");
  clear_mvs_environment();
}

void test_report_includes_selected_mvs_directory()
{
  auto snapshot = complete_orin_snapshot();
  snapshot.dependencies.mvs_library_directory = MVS_TEST_EXPLICIT_LIB;
  const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
  std::ostringstream report;
  orin_hardware_evidence::print_report(report, snapshot, evaluation, {}, {});
  expect(
    report.str().find("dependency.mvs_library_directory=" + std::string(MVS_TEST_EXPLICIT_LIB)) !=
    std::string::npos,
    "report must include the selected MVS library directory");
}

}  // namespace

int main()
{
  test_complete_metadata_is_not_hardware_validation();
  test_wsl_is_non_target();
  test_non_orin_architecture_is_non_target();
  test_missing_dependencies_are_explicit();
  test_safety_defaults_and_no_default_device_probe();
  test_mvs_search_paths_match_camera_cmake();
  test_repository_mvs_directory_is_detected();
  test_opt_layout_mvs_directory_is_detected();
  test_explicit_mvs_directory_takes_priority();
  test_environment_mvs_directory_is_supported();
  test_report_includes_selected_mvs_directory();
  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
  }
  std::cout << "All orin environment preflight tests passed\n";
  return 0;
}
