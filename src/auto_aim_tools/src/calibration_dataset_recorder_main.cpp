#include "auto_aim_tools/calibration_dataset_recorder_node.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#ifndef AUTO_AIM_DATASET_GIT_COMMIT
#define AUTO_AIM_DATASET_GIT_COMMIT "unknown"
#endif

namespace
{

struct Options
{
  std::filesystem::path config;
  std::filesystem::path output;
  std::size_t max_frames{0};
  std::size_t max_buffered_image_bytes{256U * 1024U * 1024U};
  double timeout_s{0.0};
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_calibration_dataset_recorder --config CONFIG.yaml "
    "--output DATASET_DIR --max-frames COUNT --timeout-s SECONDS "
    "[--max-buffered-image-bytes BYTES]\n"
    "Subscribes only to /image_raw and /camera_info. It never opens a camera or creates a "
    "publisher.\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto value = [&](const char * name) {
        if (index + 1 >= argc) {
          throw std::invalid_argument(std::string(name) + " requires a value");
        }
        return std::string(argv[++index]);
      };
    if (argument == "--config") {
      options.config = value("--config");
    } else if (argument == "--output") {
      options.output = value("--output");
    } else if (argument == "--max-frames") {
      options.max_frames = std::stoull(value("--max-frames"));
    } else if (argument == "--timeout-s") {
      options.timeout_s = std::stod(value("--timeout-s"));
    } else if (argument == "--max-buffered-image-bytes") {
      options.max_buffered_image_bytes = std::stoull(value("--max-buffered-image-bytes"));
    } else if (argument == "--help" || argument == "-h") {
      usage();
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.config.empty() || options.output.empty() || options.max_frames == 0U ||
    options.timeout_s <= 0.0 || options.max_buffered_image_bytes == 0U)
  {
    throw std::invalid_argument(
            "--config, --output, positive --max-frames and positive --timeout-s are required");
  }
  if (!options.config.is_absolute() || !options.output.is_absolute()) {
    throw std::invalid_argument("--config and --output must be absolute paths");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::optional<Options> options;
  try {
    options = parse_options(argc, argv);
    const auto config =
      auto_aim_tools::calibration_dataset::load_ros_config(options->config);
    rclcpp::init(argc, argv);
    auto node = std::make_shared<auto_aim_tools::CalibrationDatasetRecorderNode>(
      config, options->output, options->max_frames,
      std::chrono::milliseconds(
        static_cast<std::int64_t>(options->timeout_s * 1000.0)),
      AUTO_AIM_DATASET_GIT_COMMIT, options->max_buffered_image_bytes);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && !node->finished()) {
      executor.spin_once(std::chrono::milliseconds(50));
    }
    if (!node->finished()) {
      node->stop("interrupted");
    }
    const auto result = node->result();
    rclcpp::shutdown();
    if (!result.has_value()) {
      throw std::runtime_error("recorder stopped without an evidence manifest");
    }
    std::cout << "profile=evidence_only production_ready=false camera_sdk_status=not_used\n";
    std::cout << "status=" << (result->accepted ? "accepted" : "rejected") <<
      " manifest=" << result->manifest_path <<
      " dataset_manifest_sha256=" << result->manifest_sha256 << '\n';
    for (const auto & reason : result->rejection_reasons) {
      std::cerr << "  rejection=" << reason << '\n';
    }
    return auto_aim_tools::calibration_dataset::dataset_exit_code(*result);
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (options.has_value() && !std::filesystem::exists(options->output)) {
      try {
        auto_aim_tools::calibration_dataset::write_input_failure_manifest(
          options->output, "ros", AUTO_AIM_DATASET_GIT_COMMIT, {error.what()});
        std::cerr << "auto_aim_calibration_dataset_recorder: rejected manifest written: " <<
          error.what() << '\n';
        return 2;
      } catch (const std::exception & manifest_error) {
        std::cerr << "auto_aim_calibration_dataset_recorder: " << manifest_error.what() << '\n';
      }
    }
    std::cerr << "auto_aim_calibration_dataset_recorder: " << error.what() << '\n';
    usage();
    return 1;
  }
}
