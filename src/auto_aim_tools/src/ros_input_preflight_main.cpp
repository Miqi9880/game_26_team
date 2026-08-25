#include "auto_aim_tools/preflight_analyzer.hpp"
#include "auto_aim_tools/ros_input_preflight_node.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

#include <chrono>
#include <csignal>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
volatile std::sig_atomic_t caught_signal = 0;

void handle_signal(int signal_number)
{
  caught_signal = signal_number;
}

struct Arguments
{
  double duration_s{5.0};
  double timeout_s{1.0};
  std::string format{"text"};
  std::string vehicle_profile{"unselected"};
  bool shared_clock_domain{false};
  double sync_tolerance_ms{50.0};
};

double finite_number(const std::string & value, const std::string & option, bool positive)
{
  std::size_t consumed = 0U;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed) ||
    (positive ? parsed <= 0.0 : parsed < 0.0))
  {
    throw std::invalid_argument(option + " has an invalid numeric value");
  }
  return parsed;
}

std::string next_value(
  const std::vector<std::string> & arguments, std::size_t * index,
  const std::string & option)
{
  if (*index + 1U >= arguments.size()) {
    throw std::invalid_argument(option + " requires a value");
  }
  return arguments.at(++(*index));
}

void print_usage(std::ostream & stream)
{
  stream <<
    "Usage: ros_input_preflight [options] [--ros-args ...]\n"
    "  --duration SECONDS              observation duration (default: 5)\n"
    "  --timeout SECONDS               Image/Vision timeout (default: 1)\n"
    "  --format text|json              report format (default: text)\n"
    "  --vehicle-profile PROFILE       unselected|new_turtle|dog_leg\n"
    "  --assume-shared-clock-domain    explicitly allow cross-topic comparison\n"
    "  --sync-tolerance-ms MILLISECONDS diagnostic delta tolerance (default: 50)\n";
}

Arguments parse_arguments(const std::vector<std::string> & arguments)
{
  Arguments result;
  for (std::size_t index = 1U; index < arguments.size(); ++index) {
    const auto & option = arguments[index];
    if (option == "--help" || option == "-h") {
      print_usage(std::cout);
      std::exit(0);
    } else if (option == "--duration") {
      result.duration_s = finite_number(next_value(arguments, &index, option), option, true);
    } else if (option == "--timeout") {
      result.timeout_s = finite_number(next_value(arguments, &index, option), option, true);
    } else if (option == "--format") {
      result.format = next_value(arguments, &index, option);
      if (result.format != "text" && result.format != "json") {
        throw std::invalid_argument("--format must be text or json");
      }
    } else if (option == "--vehicle-profile") {
      result.vehicle_profile = next_value(arguments, &index, option);
      if (result.vehicle_profile != "unselected" &&
        result.vehicle_profile != "new_turtle" && result.vehicle_profile != "dog_leg")
      {
        throw std::invalid_argument(
                "--vehicle-profile must be unselected, new_turtle, or dog_leg");
      }
    } else if (option == "--assume-shared-clock-domain") {
      result.shared_clock_domain = true;
    } else if (option == "--sync-tolerance-ms") {
      result.sync_tolerance_ms = finite_number(
        next_value(arguments, &index, option), option, false);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  return result;
}

}  // namespace

int main(int argc, char ** argv)
{
  bool initialized = false;
  try {
    rclcpp::init(
      argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    initialized = true;
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    const auto arguments = parse_arguments(rclcpp::remove_ros_arguments(argc, argv));
    const double start_s = auto_aim_tools::monotonic_seconds();
    auto analyzer = std::make_shared<auto_aim_tools::PreflightAnalyzer>(
      auto_aim_tools::PreflightConfig{
      arguments.timeout_s, arguments.vehicle_profile, arguments.shared_clock_domain,
      arguments.sync_tolerance_ms,
    },
      start_s);
    auto node = std::make_shared<auto_aim_tools::RosInputPreflightNode>(analyzer);
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const double deadline_s = start_s + arguments.duration_s;
    while (
      rclcpp::ok() && caught_signal == 0 &&
      auto_aim_tools::monotonic_seconds() < deadline_s)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto report = analyzer->build_report(auto_aim_tools::monotonic_seconds());
    std::cout << (arguments.format == "json" ?
    auto_aim_tools::format_report_json(report) :
    auto_aim_tools::format_report_text(report));
    executor.remove_node(node);
    node.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (caught_signal != 0) {
      return 128 + caught_signal;
    }
    return report.overall == auto_aim_tools::Status::Fail ? 2 : 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: preflight runtime error: " << error.what() << '\n';
    if (initialized && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
