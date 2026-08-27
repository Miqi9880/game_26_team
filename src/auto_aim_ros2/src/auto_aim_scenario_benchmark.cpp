#include "auto_aim_ros2/offline_scenario_benchmark.hpp"

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace
{

struct Options
{
  std::string scenario;
  std::uint64_t seed{0};
  std::filesystem::path output_dir;
};

void usage(std::ostream & stream)
{
  stream <<
    "Usage: auto_aim_scenario_benchmark --scenario <name|all> --seed <uint64> "
    "--output-dir <new-path>\n"
    "\n"
    "Runs a deterministic, software-only synthetic benchmark. It does not open a "
    "camera, model, ROS, serial, gimbal, or hardware resource.\n"
    "\n"
    "Options:\n"
    "  --scenario NAME      one of: ";

  const auto scenarios = rm_auto_aim::offline::offline_scenario_benchmark_scenarios();
  for (std::size_t index = 0; index < scenarios.size(); ++index) {
    if (index > 0U) {
      stream << ", ";
    }
    stream << scenarios[index];
  }
  stream << "\n"
    "  --seed UINT64        fixed deterministic fixture seed\n"
    "  --output-dir PATH    a new, non-symlink output directory\n"
    "  --help               show this message\n";
}

std::uint64_t parse_seed(const std::string & text)
{
  if (text.empty()) {
    throw std::invalid_argument("--seed requires an unsigned 64-bit integer");
  }

  std::uint64_t result = 0;
  const auto parse_result = std::from_chars(
    text.data(), text.data() + text.size(), result, 10);
  if (parse_result.ec != std::errc{} || parse_result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("--seed must be an unsigned 64-bit integer");
  }
  return result;
}

std::string require_value(int & index, int argc, char ** argv, const char * option)
{
  if (index + 1 >= argc || std::string(argv[index + 1]).rfind("--", 0) == 0U) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return argv[++index];
}

Options parse_options(int argc, char ** argv)
{
  std::optional<std::string> scenario;
  std::optional<std::uint64_t> seed;
  std::optional<std::filesystem::path> output_dir;

  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--scenario") {
      if (scenario.has_value()) {
        throw std::invalid_argument("--scenario may only be specified once");
      }
      scenario = require_value(index, argc, argv, "--scenario");
    } else if (argument == "--seed") {
      if (seed.has_value()) {
        throw std::invalid_argument("--seed may only be specified once");
      }
      seed = parse_seed(require_value(index, argc, argv, "--seed"));
    } else if (argument == "--output-dir") {
      if (output_dir.has_value()) {
        throw std::invalid_argument("--output-dir may only be specified once");
      }
      output_dir = require_value(index, argc, argv, "--output-dir");
    } else if (argument == "--help") {
      throw std::invalid_argument("--help must be used alone");
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }

  if (!scenario.has_value() || !seed.has_value() || !output_dir.has_value()) {
    throw std::invalid_argument("--scenario, --seed, and --output-dir are required");
  }
  if (!rm_auto_aim::offline::is_offline_scenario_benchmark_scenario(*scenario)) {
    throw std::invalid_argument("unknown scenario: " + *scenario);
  }
  if (output_dir->empty()) {
    throw std::invalid_argument("--output-dir must not be empty");
  }

  Options result{};
  result.scenario = std::move(*scenario);
  result.seed = *seed;
  result.output_dir = std::move(*output_dir);
  return result;
}

bool safe_test_only_result(const rm_auto_aim::offline::OfflineScenarioBenchmarkResult & result)
{
  if (!result.diagnostics_enabled || result.records.empty()) {
    return false;
  }

  for (const auto & record : result.records) {
    if (!record.synthetic || !record.test_only || record.production_ready ||
      record.serial_enabled || !record.dry_run || record.allow_fire ||
      record.fire_command != 0 || record.yaw_vel_rad_s != 0.0F ||
      record.pitch_vel_rad_s != 0.0F || record.yaw_acc_rad_s2 != 0.0F ||
      record.pitch_acc_rad_s2 != 0.0F)
    {
      return false;
    }
  }
  return true;
}

void print_success(const Options & options)
{
  std::cout <<
    "software_only_synthetic_benchmark=true\n"
    "synthetic=true\n"
    "test_only=true\n"
    "production_ready=false\n"
    "diagnostics_enabled=true\n"
    "scenario=" << options.scenario << '\n' <<
    "seed=" << options.seed << '\n' <<
    "serial_enabled=false\n"
    "dry_run=true\n"
    "allow_fire=false\n"
    "fire_command=0\n"
    "yaw_vel=0\n"
    "pitch_vel=0\n"
    "yaw_acc=0\n"
    "pitch_acc=0\n"
    "artifacts=benchmark.csv,summary.json,summary.md\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    usage(std::cout);
    return 0;
  }

  try {
    const auto options = parse_options(argc, argv);
    const rm_auto_aim::offline::OfflineScenarioBenchmarkConfig config{
      options.scenario,
      options.seed,
      true};
    const auto result = rm_auto_aim::offline::run_offline_scenario_benchmark(config);
    if (!safe_test_only_result(result)) {
      throw std::runtime_error(
              "scenario benchmark safety invariant failed; no artifacts were written");
    }
    rm_auto_aim::offline::write_offline_scenario_benchmark_outputs(result, options.output_dir);
    print_success(options);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << '\n';
  }

  usage(std::cerr);
  return 2;
}
