#include "orin_hardware_evidence/orin_environment_preflight.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#endif

namespace
{

void print_usage(std::ostream & output)
{
  output <<
    "Usage: orin_environment_preflight [--repo-root PATH] "
    "[--camera-device PATH] [--serial-device PATH]\n"
    "\n"
    "Read-only metadata preflight. Device paths are never opened. When a device path is\n"
    "provided, only existence and process permission metadata are queried.\n";
}

orin_hardware_evidence::Options parse_options(const int argc, char ** argv)
{
  orin_hardware_evidence::Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      print_usage(std::cout);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--repo-root") {
      options.repo_root = value;
    } else if (argument == "--camera-device") {
      options.camera_device = value;
    } else if (argument == "--serial-device") {
      options.serial_device = value;
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  return options;
}

}  // namespace

int main(const int argc, char ** argv)
{
#if defined(_WIN32)
  SetConsoleOutputCP(CP_UTF8);
#endif
  try {
    const auto options = parse_options(argc, argv);
    const auto snapshot = orin_hardware_evidence::collect_environment(options.repo_root);
    const auto evaluation = orin_hardware_evidence::evaluate(snapshot);
    const auto camera = orin_hardware_evidence::inspect_device_permissions(
      "camera", options.camera_device);
    const auto serial = orin_hardware_evidence::inspect_device_permissions(
      "serial", options.serial_device);
    orin_hardware_evidence::print_report(std::cout, snapshot, evaluation, camera, serial);
    return evaluation.exit_code;
  } catch (const std::exception & error) {
    std::cerr << "preflight.error=" << error.what() << '\n';
    return 64;
  }
}
