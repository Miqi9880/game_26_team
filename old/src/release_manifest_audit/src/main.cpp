#include "release_manifest_audit/audit.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char ** argv)
{
  std::string config;
  std::string output;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if ((argument == "--config" || argument == "--output-dir") && index + 1 < argc) {
      (argument == "--config" ? config : output) = argv[++index];
    } else if (argument == "--help") {
      std::cout << "Usage: release_manifest_audit --config INPUT.json --output-dir NEW_DIRECTORY\n";
      return 0;
    } else {
      std::cerr << "release_manifest_audit: unknown or incomplete argument: " << argument << '\n';
      return 1;
    }
  }
  if (config.empty() || output.empty()) {
    std::cerr << "release_manifest_audit: --config and --output-dir are required\n";
    return 1;
  }
  try {
    return release_manifest_audit::run(config, output, std::cout, std::cerr);
  } catch (const std::exception & exception) {
    std::cerr << "release_manifest_audit: " << exception.what() << '\n';
    return 1;
  }
}
