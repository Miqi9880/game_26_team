#include "auto_aim_tools/calibration_dataset.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef AUTO_AIM_DATASET_GIT_COMMIT
#define AUTO_AIM_DATASET_GIT_COMMIT "unknown"
#endif

namespace
{

struct Options
{
  std::filesystem::path fixture;
  std::filesystem::path output;
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_calibration_dataset --fixture FIXTURE.yaml --output DATASET_DIR\n"
    "Creates an SDK-independent evidence-only calibration dataset. Existing output is never "
    "overwritten.\n";
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
    if (argument == "--fixture") {
      options.fixture = value("--fixture");
    } else if (argument == "--output") {
      options.output = value("--output");
    } else if (argument == "--help" || argument == "-h") {
      usage();
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (options.fixture.empty() || options.output.empty()) {
    throw std::invalid_argument("--fixture and --output are required");
  }
  if (!options.fixture.is_absolute() || !options.output.is_absolute()) {
    throw std::invalid_argument("--fixture and --output must be absolute paths");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    try {
      const auto request =
        auto_aim_tools::calibration_dataset::load_offline_fixture(options.fixture);
      const auto result = auto_aim_tools::calibration_dataset::build_dataset(
        request, options.output, AUTO_AIM_DATASET_GIT_COMMIT);
      std::cout << "profile=evidence_only production_ready=false camera_sdk_status=not_used\n";
      std::cout << "status=" << (result.accepted ? "accepted" : "rejected") <<
        " manifest=" << result.manifest_path <<
        " dataset_manifest_sha256=" << result.manifest_sha256 << '\n';
      for (const auto & reason : result.rejection_reasons) {
        std::cerr << "  rejection=" << reason << '\n';
      }
      return auto_aim_tools::calibration_dataset::dataset_exit_code(result);
    } catch (const std::exception & error) {
      if (std::filesystem::exists(options.output)) {
        throw;
      }
      const auto result =
        auto_aim_tools::calibration_dataset::write_input_failure_manifest(
        options.output, "offline_fixture", AUTO_AIM_DATASET_GIT_COMMIT, {error.what()});
      std::cerr << "auto_aim_calibration_dataset: rejected evidence manifest written: " <<
        error.what() << '\n';
      return auto_aim_tools::calibration_dataset::dataset_exit_code(result);
    }
  } catch (const std::exception & error) {
    std::cerr << "auto_aim_calibration_dataset: " << error.what() << '\n';
    usage();
    return 1;
  }
}
