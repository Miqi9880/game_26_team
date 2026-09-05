#include "orin_hardware_evidence/orin_environment_preflight.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <ostream>
#include <sstream>

#if defined(_WIN32)
#include <io.h>
#include <sys/stat.h>
#else
#include <dirent.h>
#include <sys/stat.h>
#include <sys/utsname.h>
#include <unistd.h>
#endif

namespace orin_hardware_evidence
{
namespace
{

std::string lowercase(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string read_small_text_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return {};
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  std::string result = contents.str();
  std::replace(result.begin(), result.end(), '\0', ' ');
  while (!result.empty() && std::isspace(static_cast<unsigned char>(result.back()))) {
    result.pop_back();
  }
  return result;
}

bool environment_has_value(const char * name)
{
  const char * value = std::getenv(name);
  return value != nullptr && value[0] != '\0';
}

std::string join_path(const std::string & left, const std::string & right)
{
  if (left.empty() || left == ".") {
    return left == "." ? "./" + right : right;
  }
  const char last = left.back();
  return (last == '/' || last == '\\') ? left + right : left + "/" + right;
}

bool regular_file(const std::string & path)
{
#if defined(_WIN32)
  struct _stat metadata {};
  return _stat(path.c_str(), &metadata) == 0 && (metadata.st_mode & _S_IFREG) != 0;
#else
  struct stat metadata {};
  return stat(path.c_str(), &metadata) == 0 && S_ISREG(metadata.st_mode);
#endif
}

bool path_exists(const std::string & path)
{
#if defined(_WIN32)
  struct _stat metadata {};
  return _stat(path.c_str(), &metadata) == 0;
#else
  struct stat metadata {};
  return stat(path.c_str(), &metadata) == 0;
#endif
}

bool executable_on_path(const std::string & executable)
{
  const char * raw_path = std::getenv("PATH");
  if (raw_path == nullptr) {
    return false;
  }
#if defined(_WIN32)
  constexpr char separator = ';';
  const std::vector<std::string> suffixes{"", ".exe", ".cmd", ".bat"};
#else
  constexpr char separator = ':';
  const std::vector<std::string> suffixes{""};
#endif
  std::istringstream paths(raw_path);
  std::string directory;
  while (std::getline(paths, directory, separator)) {
    if (directory.empty()) {
      continue;
    }
    for (const auto & suffix : suffixes) {
      const auto candidate = join_path(directory, executable + suffix);
      if (regular_file(candidate)) {
        return true;
      }
    }
  }
  return false;
}

bool any_regular_file(const std::vector<std::string> & candidates)
{
  return std::any_of(candidates.begin(), candidates.end(), [](const auto & candidate) {
    return regular_file(candidate);
  });
}

bool library_file_present(const std::string & directory, const std::string & name)
{
  if (regular_file(join_path(directory, "lib" + name + ".so")) ||
    regular_file(join_path(directory, "lib" + name + ".so.1")))
  {
    return true;
  }
#if !defined(_WIN32)
  DIR * handle = opendir(directory.c_str());
  if (handle == nullptr) {
    return false;
  }
  const std::string prefix = "lib" + name + ".so.";
  bool found = false;
  while (const dirent * entry = readdir(handle)) {
    const std::string filename = entry->d_name;
    if (filename.rfind(prefix, 0) == 0 && regular_file(join_path(directory, filename))) {
      found = true;
      break;
    }
  }
  closedir(handle);
  return found;
#else
  return false;
#endif
}

bool openvino_config_present()
{
  const char * configured = std::getenv("OpenVINO_DIR");
  if (configured != nullptr && configured[0] != '\0') {
    if (any_regular_file({join_path(configured, "OpenVINOConfig.cmake")})) {
      return true;
    }
  }

#if !defined(_WIN32)
  DIR * directory = opendir("/opt/intel");
  if (directory == nullptr) {
    return false;
  }
  bool found = false;
  while (const dirent * entry = readdir(directory)) {
    const std::string name = entry->d_name;
    if (name == "." || name == "..") {
      continue;
    }
    const auto config = join_path(join_path("/opt/intel", name), "runtime/cmake/OpenVINOConfig.cmake");
    if (regular_file(config)) {
      found = true;
      break;
    }
  }
  closedir(directory);
  return found;
#else
  return false;
#endif
}

std::vector<std::string> mvs_library_candidates(
  const std::string & repo_root, const std::optional<std::string> & explicit_directory)
{
  std::vector<std::string> candidates;
  if (explicit_directory && !explicit_directory->empty()) {
    candidates.emplace_back(*explicit_directory);
  }
  const char * configured = std::getenv("HIK_MVS_LIBRARY_DIR");
  if ((!explicit_directory || explicit_directory->empty()) && configured != nullptr && configured[0] != '\0') {
    candidates.emplace_back(configured);
  }
  candidates.emplace_back(join_path(repo_root, "src/ros2-hik-camera/hikSDK/lib/arm64"));
  candidates.emplace_back("/opt/MVS/lib/aarch64");
  return candidates;
}

std::string first_complete_mvs_library_directory(const std::vector<std::string> & candidates)
{
  const std::vector<std::string> libraries{
    "FormatConversion", "MediaProcess", "MvCameraControl", "MVRender", "MvUsb3vTL"};
  for (const auto & candidate : candidates) {
    const bool complete = std::all_of(libraries.begin(), libraries.end(), [&candidate](const auto & library) {
      return library_file_present(candidate, library);
    });
    if (complete) {
      return candidate;
    }
  }
  return {};
}

std::string bool_text(const bool value)
{
  return value ? "true" : "false";
}

std::string permission_status(const DevicePermissionEvidence & evidence)
{
  if (!evidence.requested) {
    return "NOT_REQUESTED (no device probe performed)";
  }
  if (!evidence.exists) {
    return "PATH_NOT_FOUND";
  }
  if (evidence.readable && evidence.writable) {
    return "METADATA_PERMISSION_PRESENT (device not opened)";
  }
  return "PERMISSION_MISSING (device not opened)";
}

}  // namespace

std::vector<std::string> mvs_library_search_paths(
  const std::string & repo_root, const std::optional<std::string> & explicit_directory)
{
  return mvs_library_candidates(repo_root, explicit_directory);
}

std::string find_complete_mvs_library_directory(const std::vector<std::string> & candidates)
{
  return first_complete_mvs_library_directory(candidates);
}

EnvironmentSnapshot collect_environment(
  const std::string & repo_root, const std::optional<std::string> & mvs_library_dir)
{
  EnvironmentSnapshot snapshot;
#if defined(_WIN32)
  snapshot.os = "windows";
#if defined(_M_ARM64)
  snapshot.architecture = "aarch64";
#elif defined(_M_X64)
  snapshot.architecture = "x86_64";
#else
  snapshot.architecture = "other";
#endif
#else
  snapshot.os = "linux";
  struct utsname system_info {};
  if (uname(&system_info) == 0) {
    snapshot.architecture = system_info.machine;
  } else {
    snapshot.architecture = "unknown";
  }
#endif

  const std::string os_release = lowercase(read_small_text_file("/proc/sys/kernel/osrelease"));
  snapshot.is_wsl = environment_has_value("WSL_INTEROP") ||
    environment_has_value("WSL_DISTRO_NAME") || os_release.find("microsoft") != std::string::npos;
  snapshot.device_model = read_small_text_file("/proc/device-tree/model");

  snapshot.dependencies.ros2_executable = executable_on_path("ros2");
  snapshot.dependencies.ros_distro = environment_has_value("ROS_DISTRO");
  snapshot.dependencies.opencv = any_regular_file({
    "/usr/include/opencv4/opencv2/core.hpp",
    "/usr/local/include/opencv4/opencv2/core.hpp"});
  snapshot.dependencies.openvino = openvino_config_present();
  snapshot.dependencies.mvs_headers = any_regular_file({
    join_path(repo_root, "src/ros2-hik-camera/hikSDK/include/MvCameraControl.h")});
  snapshot.dependencies.mvs_library_directory = find_complete_mvs_library_directory(
    mvs_library_search_paths(repo_root, mvs_library_dir));
  snapshot.dependencies.mvs_arm64_libraries = !snapshot.dependencies.mvs_library_directory.empty();
  return snapshot;
}

DevicePermissionEvidence inspect_device_permissions(
  const std::string & label, const std::optional<std::string> & path)
{
  DevicePermissionEvidence evidence;
  evidence.label = label;
  evidence.requested = path.has_value();
  if (!path) {
    return evidence;
  }

  evidence.exists = path_exists(*path);
  if (!evidence.exists) {
    return evidence;
  }
#if defined(_WIN32)
  evidence.readable = _access(path->c_str(), 4) == 0;
  evidence.writable = _access(path->c_str(), 2) == 0;
#else
  evidence.readable = access(path->c_str(), R_OK) == 0;
  evidence.writable = access(path->c_str(), W_OK) == 0;
#endif
  return evidence;
}

Evaluation evaluate(const EnvironmentSnapshot & snapshot)
{
  Evaluation result;
  if (snapshot.os != "linux") {
    result.target_reasons.emplace_back("OS is not Linux");
  }
  if (snapshot.is_wsl) {
    result.target_reasons.emplace_back("WSL is not the target Orin environment");
  }
  if (snapshot.architecture != "aarch64" && snapshot.architecture != "arm64") {
    result.target_reasons.emplace_back("architecture is not aarch64/arm64");
  }
  const std::string model = lowercase(snapshot.device_model);
  if (model.find("orin") == std::string::npos) {
    result.target_reasons.emplace_back("Orin device-tree model evidence is absent");
  }
  result.target_environment = result.target_reasons.empty();

  if (!snapshot.dependencies.ros2_executable) {
    result.missing_dependencies.emplace_back("ros2 executable on PATH");
  }
  if (!snapshot.dependencies.ros_distro) {
    result.missing_dependencies.emplace_back("ROS_DISTRO environment");
  }
  if (!snapshot.dependencies.opencv) {
    result.missing_dependencies.emplace_back("OpenCV development headers");
  }
  if (!snapshot.dependencies.openvino) {
    result.missing_dependencies.emplace_back("OpenVINO CMake package");
  }
  if (!snapshot.dependencies.mvs_headers) {
    result.missing_dependencies.emplace_back("MVS SDK headers");
  }
  if (!snapshot.dependencies.mvs_arm64_libraries) {
    result.missing_dependencies.emplace_back("MVS SDK arm64 libraries");
  }
  result.dependencies_complete = result.missing_dependencies.empty();

  if (!result.target_environment && !result.dependencies_complete) {
    result.exit_code = 4;
  } else if (!result.target_environment) {
    result.exit_code = 2;
  } else if (!result.dependencies_complete) {
    result.exit_code = 3;
  }
  return result;
}

void print_report(
  std::ostream & output, const EnvironmentSnapshot & snapshot, const Evaluation & evaluation,
  const DevicePermissionEvidence & camera, const DevicePermissionEvidence & serial)
{
  output << "orin_hardware_environment_preflight.version=2\n"
         << "scope=READ_ONLY_METADATA\n"
         << "hardware_access=DISABLED\n"
         << "camera_opened=false\n"
         << "serial_opened=false\n"
         << "mvs_sdk_loaded=false\n"
         << "serial_enabled=false\n"
         << "dry_run=true\n"
         << "allow_fire=false\n"
         << "fire_command=0\n"
         << "os=" << snapshot.os << '\n'
         << "architecture=" << snapshot.architecture << '\n'
         << "wsl=" << bool_text(snapshot.is_wsl) << '\n'
         << "orin_model_evidence=" << bool_text(
    lowercase(snapshot.device_model).find("orin") != std::string::npos) << '\n'
         << "dependency.ros2=" << bool_text(snapshot.dependencies.ros2_executable) << '\n'
         << "dependency.ros_distro=" << bool_text(snapshot.dependencies.ros_distro) << '\n'
         << "dependency.opencv=" << bool_text(snapshot.dependencies.opencv) << '\n'
         << "dependency.openvino=" << bool_text(snapshot.dependencies.openvino) << '\n'
         << "dependency.mvs_headers=" << bool_text(snapshot.dependencies.mvs_headers) << '\n'
         << "dependency.mvs_arm64_libraries=" <<
    bool_text(snapshot.dependencies.mvs_arm64_libraries) << '\n'
         << "dependency.mvs_library_directory=" <<
    (snapshot.dependencies.mvs_library_directory.empty() ? "NONE" : snapshot.dependencies.mvs_library_directory) << '\n'
         << "camera_permission=" << permission_status(camera) << '\n'
         << "serial_permission=" << permission_status(serial) << '\n';

  if (evaluation.target_environment) {
    output << "environment.status=TARGET_ENVIRONMENT_METADATA_FOUND\n";
  } else {
    output << u8"environment.status=非目标环境 (NON_TARGET_ENVIRONMENT)\n";
    for (const auto & reason : evaluation.target_reasons) {
      output << "environment.reason=" << reason << '\n';
    }
  }

  if (evaluation.dependencies_complete) {
    output << "dependencies.status=DEPENDENCY_METADATA_FOUND\n";
  } else {
    output << u8"dependencies.status=缺依赖 (MISSING_DEPENDENCIES)\n";
    for (const auto & dependency : evaluation.missing_dependencies) {
      output << "dependency.missing=" << dependency << '\n';
    }
  }

  if (!evaluation.target_environment && !evaluation.dependencies_complete) {
    output << u8"overall=非目标环境/缺依赖 (NON_TARGET_ENVIRONMENT/MISSING_DEPENDENCIES)\n";
  } else if (!evaluation.target_environment) {
    output << u8"overall=非目标环境 (NON_TARGET_ENVIRONMENT)\n";
  } else if (!evaluation.dependencies_complete) {
    output << u8"overall=缺依赖 (MISSING_DEPENDENCIES)\n";
  } else {
    output << "overall=ENVIRONMENT_PREFLIGHT_COMPLETE_NOT_HARDWARE_VALIDATED\n";
  }
  output << "hardware_validation=NOT_RUN\n";
}

}  // namespace orin_hardware_evidence
