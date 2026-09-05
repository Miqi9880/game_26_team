#include "auto_aim_release_smoke/release_smoke.hpp"

#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{

void usage(std::ostream & stream)
{
  stream <<
    "Usage: auto_aim_release_smoke --install-base PATH --output-dir NEW_PATH "
    "--baseline SHA --commit SHA [options]\n"
    "Options:\n"
    "  --orin-preflight PATH        standalone Orin preflight executable\n"
    "  --rosdep-status STATUS       PASS, FAIL, UNAVAILABLE, or NOT_RUN\n"
    "  --help                       show this message without running checks\n";
}

auto_aim_release_smoke::Options parse_options(int argc, char ** argv)
{
  auto_aim_release_smoke::Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto require_value = [&](const char * name) -> std::string {
        if (index + 1 >= argc) {
          throw std::invalid_argument(std::string(name) + " requires a value");
        }
        return argv[++index];
      };

    if (argument == "--install-base") {
      options.install_base = require_value("--install-base");
    } else if (argument == "--output-dir") {
      options.output_dir = require_value("--output-dir");
    } else if (argument == "--baseline") {
      options.baseline = require_value("--baseline");
    } else if (argument == "--commit") {
      options.commit = require_value("--commit");
    } else if (argument == "--orin-preflight") {
      options.orin_preflight = require_value("--orin-preflight");
    } else if (argument == "--rosdep-status") {
      options.rosdep_status = auto_aim_release_smoke::parse_status(
        require_value("--rosdep-status"));
    } else if (argument == "--help" || argument == "-h") {
      usage(std::cout);
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }

  if (options.install_base.empty() || options.output_dir.empty() ||
    options.baseline.empty() || options.commit.empty())
  {
    throw std::invalid_argument(
            "--install-base, --output-dir, --baseline, and --commit are required");
  }
  if (!auto_aim_release_smoke::valid_git_sha(options.baseline) ||
    !auto_aim_release_smoke::valid_git_sha(options.commit))
  {
    throw std::invalid_argument("baseline and commit must be full 40-character Git SHAs");
  }
  return options;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return auto_aim_release_smoke::run(parse_options(argc, argv), std::cout, std::cerr);
  } catch (const std::exception & exception) {
    std::cerr << "auto_aim_release_smoke: " << exception.what() << '\n';
    usage(std::cerr);
    return 2;
  }
}
