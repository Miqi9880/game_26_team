#include "orin_hardware_evidence/orin_environment_preflight.hpp"

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

namespace
{

int failures = 0;

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

}  // namespace

int main()
{
  test_complete_metadata_is_not_hardware_validation();
  test_wsl_is_non_target();
  test_non_orin_architecture_is_non_target();
  test_missing_dependencies_are_explicit();
  test_safety_defaults_and_no_default_device_probe();
  if (failures != 0) {
    std::cerr << failures << " test assertion(s) failed\n";
    return 1;
  }
  std::cout << "All orin environment preflight tests passed\n";
  return 0;
}
