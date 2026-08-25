#include "auto_aim_ros2/camera_calibration.hpp"

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct Options
{
  std::string config_path;
  std::string image_list_path;
  std::string report_path;
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_camera_calibrate --config CONFIG.yaml --image-list IMAGES.txt "
    "--report REPORT.yaml\n"
    "\n"
    "Reads local raw images only. The output is evidence_only and is never a "
    "production PnP configuration.\n";
}

Options parse_options(int argc, char ** argv)
{
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto require_value = [&](const char * option) -> std::string {
      if (index + 1 >= argc) {
        throw std::invalid_argument(std::string(option) + " requires a value");
      }
      return argv[++index];
    };
    if (argument == "--config") {
      result.config_path = require_value("--config");
    } else if (argument == "--image-list") {
      result.image_list_path = require_value("--image-list");
    } else if (argument == "--report") {
      result.report_path = require_value("--report");
    } else if (argument == "--help" || argument == "-h") {
      usage();
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (result.config_path.empty() || result.image_list_path.empty() || result.report_path.empty()) {
    throw std::invalid_argument("--config, --image-list, and --report are required");
  }
  return result;
}

void require_absolute_path(const std::string & path, const char * option)
{
  if (!std::filesystem::path(path).is_absolute()) {
    throw std::invalid_argument(std::string(option) + " must be an absolute path");
  }
}

void reject_report_input_aliases(const Options & options)
{
  const auto report = std::filesystem::path(options.report_path).lexically_normal();
  const auto config = std::filesystem::path(options.config_path).lexically_normal();
  const auto manifest = std::filesystem::path(options.image_list_path).lexically_normal();
  if (report == config || report == manifest) {
    throw std::invalid_argument("--report must not overwrite --config or --image-list");
  }
  if (std::filesystem::exists(report) && std::filesystem::is_directory(report)) {
    throw std::invalid_argument("--report must name a file, not a directory");
  }
  const auto parent = report.parent_path();
  if (!parent.empty() && !std::filesystem::exists(parent)) {
    throw std::invalid_argument("parent directory of --report does not exist: " + parent.string());
  }
}

std::vector<std::string> resolve_image_paths(
  const std::string & manifest_path,
  const std::vector<std::string> & entries)
{
  const auto base = std::filesystem::path(manifest_path).parent_path();
  std::vector<std::string> result;
  result.reserve(entries.size());
  for (const auto & entry : entries) {
    const auto path = std::filesystem::path(entry);
    result.push_back((path.is_absolute() ? path : base / path).lexically_normal().string());
  }
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    require_absolute_path(options.config_path, "--config");
    require_absolute_path(options.image_list_path, "--image-list");
    require_absolute_path(options.report_path, "--report");
    reject_report_input_aliases(options);

    const auto config = rm_auto_aim::camera_calibration::load_calibration_input(options.config_path);
    const auto manifest_entries = rm_auto_aim::camera_calibration::load_image_manifest(
      options.image_list_path);
    const auto image_paths = resolve_image_paths(options.image_list_path, manifest_entries);
    const auto result = rm_auto_aim::camera_calibration::run_image_calibration(config, image_paths);
    rm_auto_aim::camera_calibration::write_evidence_report(result, options.report_path);

    std::cout << "serial_enabled=false dry_run=true allow_fire=false fire_command=0\n";
    std::cout << "profile=evidence_only production_ready=false accepted_views=" <<
      std::count_if(
        result.image_evidence.begin(), result.image_evidence.end(),
        [](const auto & evidence) { return evidence.accepted; }) <<
      " rejected_views=" <<
      std::count_if(
        result.image_evidence.begin(), result.image_evidence.end(),
        [](const auto & evidence) { return !evidence.accepted; }) << '\n';
    if (result.global_rms_reprojection_error_px.has_value()) {
      std::cout << "global_rms_reprojection_error_px=" <<
        *result.global_rms_reprojection_error_px << '\n';
    }
    if (!result.quality_accepted) {
      std::cerr << "auto_aim_camera_calibrate: rejected evidence report written to " <<
        options.report_path << '\n';
      for (const auto & reason : result.rejection_reasons) {
        std::cerr << "  rejection=" << reason << '\n';
      }
    }
    return rm_auto_aim::camera_calibration::quality_exit_code(result);
  } catch (const std::exception & error) {
    std::cerr << "auto_aim_camera_calibrate: " << error.what() << '\n';
    usage();
    return 1;
  }
}
