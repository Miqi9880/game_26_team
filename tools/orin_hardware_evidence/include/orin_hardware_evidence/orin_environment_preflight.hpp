#ifndef ORIN_HARDWARE_EVIDENCE__ORIN_ENVIRONMENT_PREFLIGHT_HPP_
#define ORIN_HARDWARE_EVIDENCE__ORIN_ENVIRONMENT_PREFLIGHT_HPP_

#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

namespace orin_hardware_evidence
{

struct DependencyEvidence
{
  bool ros2_executable{false};
  bool ros_distro{false};
  bool opencv{false};
  bool openvino{false};
  bool mvs_headers{false};
  bool mvs_arm64_libraries{false};
  std::string mvs_library_directory;
};

struct EnvironmentSnapshot
{
  std::string os;
  std::string architecture;
  std::string device_model;
  bool is_wsl{false};
  DependencyEvidence dependencies;
};

struct DevicePermissionEvidence
{
  std::string label;
  bool requested{false};
  bool exists{false};
  bool readable{false};
  bool writable{false};
};

struct Evaluation
{
  bool target_environment{false};
  bool dependencies_complete{false};
  std::vector<std::string> target_reasons;
  std::vector<std::string> missing_dependencies;
  int exit_code{0};
};

struct Options
{
  std::string repo_root{"."};
  std::optional<std::string> camera_device;
  std::optional<std::string> serial_device;
  std::optional<std::string> mvs_library_dir;
};

EnvironmentSnapshot collect_environment(
  const std::string & repo_root, const std::optional<std::string> & mvs_library_dir = std::nullopt);
std::vector<std::string> mvs_library_search_paths(
  const std::string & repo_root, const std::optional<std::string> & explicit_directory = std::nullopt);
std::string find_complete_mvs_library_directory(const std::vector<std::string> & candidates);
DevicePermissionEvidence inspect_device_permissions(
  const std::string & label, const std::optional<std::string> & path);
Evaluation evaluate(const EnvironmentSnapshot & snapshot);
void print_report(
  std::ostream & output, const EnvironmentSnapshot & snapshot, const Evaluation & evaluation,
  const DevicePermissionEvidence & camera, const DevicePermissionEvidence & serial);

}  // namespace orin_hardware_evidence

#endif  // ORIN_HARDWARE_EVIDENCE__ORIN_ENVIRONMENT_PREFLIGHT_HPP_
