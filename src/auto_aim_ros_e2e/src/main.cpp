#include "auto_aim_ros_e2e/report.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <sys/wait.h>
#include <unistd.h>

#include <auto_aim_interfaces/msg/robot_ctrl.hpp>
#include <auto_aim_interfaces/msg/vision.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace
{
namespace fs = std::filesystem;
using namespace std::chrono_literals;
using auto_aim_ros_e2e::Artifact;
using auto_aim_ros_e2e::CaseResult;
using auto_aim_ros_e2e::Status;

constexpr std::int8_t kTargetLocked = 49;
constexpr std::int8_t kTargetUnlocked = 50;
volatile std::sig_atomic_t caught_signal = 0;
std::array<volatile std::sig_atomic_t, 8> active_process_groups{};

void handle_signal(int signal_number)
{
  caught_signal = signal_number;
  for (const auto group : active_process_groups) {
    if (group > 0) {
      ::kill(-static_cast<pid_t>(group), SIGTERM);
    }
  }
}

void register_process_group(pid_t group)
{
  for (auto & slot : active_process_groups) {
    if (slot == 0) {
      slot = static_cast<std::sig_atomic_t>(group);
      return;
    }
  }
  throw std::runtime_error("too many active child process groups");
}

void unregister_process_group(pid_t group) noexcept
{
  for (auto & slot : active_process_groups) {
    if (slot == static_cast<std::sig_atomic_t>(group)) {
      slot = 0;
    }
  }
}

void throw_if_interrupted()
{
  if (caught_signal != 0) {
    throw std::runtime_error("interrupted by signal " + std::to_string(caught_signal));
  }
}

struct Options
{
  fs::path install_base;
  fs::path output_dir;
  std::string baseline;
  std::string commit;
  std::size_t rounds{5U};
  std::uint32_t seed{260033U};
};

struct Process
{
  pid_t pid{-1};
  fs::path log_path;
  std::vector<std::string> command;
  std::optional<int> exit_code;
};

struct Topics
{
  std::string image;
  std::string camera_info;
  std::string vision;
  std::string control;
};

enum class Scenario
{
  ValidNull,
  ValidMock,
  MissingCameraInfo,
  CameraDimensions,
  CameraNanK,
  CameraInfD,
  CameraZeroFocal,
  EmptyFrame,
  ZeroDimensions,
  WrongEncoding,
  ShortStride,
  ShortData,
  MissingTimestamp,
  StampMismatch,
  StampRollback,
  StampDuplicate,
  WrongTopic,
  WrongQos,
  ReorderedValid,
  TemporaryOcclusion,
  ContinuousInterruption,
  NoInput,
};

struct ScenarioSpec
{
  const char * id;
  Scenario scenario;
  const char * input_summary;
  const char * expected;
  bool preflight_fail;
  bool use_mock;
};

std::string read_text(const fs::path & path)
{
  std::ifstream input(path, std::ios::binary);
  std::ostringstream stream;
  stream << input.rdbuf();
  return stream.str();
}

std::string command_text(const std::vector<std::string> & command)
{
  std::ostringstream stream;
  for (std::size_t index = 0U; index < command.size(); ++index) {
    if (index != 0U) {
      stream << ' ';
    }
    const auto & argument = command[index];
    if (argument.find_first_of(" \t\n\"'") == std::string::npos) {
      stream << argument;
    } else {
      stream << std::quoted(argument, '\'');
    }
  }
  return stream.str();
}

bool process_group_has_live_member(pid_t group)
{
  std::error_code error;
  for (const auto & entry : fs::directory_iterator("/proc", error)) {
    const auto name = entry.path().filename().string();
    if (error || name.empty() ||
      !std::all_of(name.begin(), name.end(), [](unsigned char value) {
        return std::isdigit(value) != 0;
      }))
    {
      continue;
    }
    std::ifstream stat(entry.path() / "stat");
    std::string line;
    std::getline(stat, line);
    const auto close = line.rfind(')');
    if (close == std::string::npos) {
      continue;
    }
    std::istringstream fields(line.substr(close + 2U));
    char state = '\0';
    pid_t parent = 0;
    pid_t process_group = 0;
    fields >> state >> parent >> process_group;
    if (fields && process_group == group && state != 'Z' && state != 'X') {
      return true;
    }
  }
  return false;
}

bool wait_for_group_exit(pid_t group, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (!process_group_has_live_member(group)) {
      return true;
    }
    std::this_thread::sleep_for(20ms);
  }
  return !process_group_has_live_member(group);
}

int wait_status_code(int status)
{
  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
  }
  if (WIFSIGNALED(status)) {
    return 128 + WTERMSIG(status);
  }
  return -1;
}

Process spawn_process(const std::vector<std::string> & command, const fs::path & log_path)
{
  if (command.empty()) {
    throw std::invalid_argument("cannot spawn an empty command");
  }
  const int descriptor = ::open(
    log_path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC,
    S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
  if (descriptor < 0) {
    throw std::runtime_error("cannot create process log: " + log_path.string());
  }
  const pid_t child = ::fork();
  if (child < 0) {
    ::close(descriptor);
    throw std::runtime_error("fork failed");
  }
  if (child == 0) {
    ::setpgid(0, 0);
    ::dup2(descriptor, STDOUT_FILENO);
    ::dup2(descriptor, STDERR_FILENO);
    ::close(descriptor);
    std::vector<char *> arguments;
    for (const auto & value : command) {
      arguments.push_back(const_cast<char *>(value.c_str()));
    }
    arguments.push_back(nullptr);
    ::execv(arguments.front(), arguments.data());
    ::dprintf(STDERR_FILENO, "exec failed: %s\n", command.front().c_str());
    ::_exit(127);
  }
  ::close(descriptor);
  ::setpgid(child, child);
  register_process_group(child);
  return Process{child, log_path, command, std::nullopt};
}

bool poll_process(Process * process)
{
  if (process->exit_code.has_value()) {
    return true;
  }
  int status = 0;
  const pid_t waited = ::waitpid(process->pid, &status, WNOHANG);
  if (waited == process->pid) {
    process->exit_code = wait_status_code(status);
    return true;
  }
  if (waited < 0) {
    throw std::runtime_error("waitpid failed for: " + command_text(process->command));
  }
  return false;
}

bool wait_process(Process * process, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (poll_process(process)) {
      return true;
    }
    if (caught_signal != 0) {
      return false;
    }
    std::this_thread::sleep_for(20ms);
  }
  return poll_process(process);
}

bool stop_process(Process * process, bool repeated = false, bool force = false)
{
  if (!poll_process(process)) {
    const int signal_number = force ? SIGKILL : SIGINT;
    ::kill(-process->pid, signal_number);
    if (repeated) {
      ::kill(-process->pid, signal_number);
    }
    if (!wait_process(process, force ? 2s : 4s)) {
      ::kill(-process->pid, SIGTERM);
      if (!wait_process(process, 1s)) {
        ::kill(-process->pid, SIGKILL);
        wait_process(process, 2s);
      }
    }
  }
  if (process_group_has_live_member(process->pid)) {
    ::kill(-process->pid, SIGKILL);
    wait_for_group_exit(process->pid, 2s);
  }
  const bool stopped = process->exit_code.has_value() &&
    !process_group_has_live_member(process->pid);
  if (stopped) {
    unregister_process_group(process->pid);
  }
  return stopped;
}

fs::path package_prefix(const fs::path & install_base, const std::string & package)
{
  return fs::is_directory(install_base / package) ? install_base / package : install_base;
}

fs::path executable_path(
  const fs::path & install_base, const std::string & package, const std::string & executable)
{
  return package_prefix(install_base, package) / "lib" / package / executable;
}

std::string sanitize_id(std::string value)
{
  std::replace_if(value.begin(), value.end(), [](unsigned char character) {
      return std::isalnum(character) == 0;
    }, '_');
  return value;
}

class FixtureProbe final : public rclcpp::Node
{
public:
  FixtureProbe(
    const std::string & name, const Topics & topics, bool wrong_topic, bool wrong_qos)
  : Node(name), topics_(topics)
  {
    const auto image_topic = wrong_topic ? topics.image + "_wrong" : topics.image;
    const auto camera_topic = wrong_topic ? topics.camera_info + "_wrong" : topics.camera_info;
    const auto qos = wrong_qos ? rclcpp::QoS(rclcpp::KeepLast(10)).reliable() :
      rclcpp::SensorDataQoS();
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
    camera_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_topic, qos);
    vision_publisher_ = create_publisher<auto_aim_interfaces::msg::Vision>(
      topics.vision, 10);
    control_subscription_ = create_subscription<auto_aim_interfaces::msg::RobotCtrl>(
      topics.control, 10, [this](auto_aim_interfaces::msg::RobotCtrl::ConstSharedPtr message) {
        controls_.push_back(*message);
      });
  }

  void publish(
    const sensor_msgs::msg::Image & image,
    const std::optional<sensor_msgs::msg::CameraInfo> & camera_info,
    const auto_aim_interfaces::msg::Vision & vision)
  {
    image_publisher_->publish(image);
    if (camera_info.has_value()) {
      camera_publisher_->publish(*camera_info);
    }
    vision_publisher_->publish(vision);
  }

  void publish_image(const sensor_msgs::msg::Image & image)
  {
    image_publisher_->publish(image);
  }

  void publish_camera_info(const sensor_msgs::msg::CameraInfo & camera_info)
  {
    camera_publisher_->publish(camera_info);
  }

  void publish_vision(const auto_aim_interfaces::msg::Vision & vision)
  {
    vision_publisher_->publish(vision);
  }

  const std::vector<auto_aim_interfaces::msg::RobotCtrl> & controls() const noexcept
  {
    return controls_;
  }

  std::string publisher_observation(const std::string & auto_node_name) const
  {
    const auto endpoints = get_publishers_info_by_topic(topics_.control);
    bool auto_node_found = false;
    bool preflight_found = false;
    std::ostringstream result;
    result << "control_publishers=" << endpoints.size() << " [";
    for (std::size_t index = 0U; index < endpoints.size(); ++index) {
      if (index != 0U) {
        result << ',';
      }
      result << endpoints[index].node_namespace() << '/' << endpoints[index].node_name();
      auto_node_found = auto_node_found || endpoints[index].node_name() == auto_node_name;
      preflight_found = preflight_found ||
        endpoints[index].node_name().find("preflight") != std::string::npos;
    }
    result << "]; auto_node_found=" << (auto_node_found ? "true" : "false")
           << "; preflight_control_publisher=" << (preflight_found ? "true" : "false")
           << "; unremapped_control_publishers=" << count_publishers("/Robot_ctrl_data");
    return result.str();
  }

  bool graph_ready(bool wrong_topic) const
  {
    if (wrong_topic) {
      return count_publishers(topics_.control) == 1U;
    }
    return image_publisher_->get_subscription_count() >= 2U &&
           camera_publisher_->get_subscription_count() >= 2U &&
           vision_publisher_->get_subscription_count() >= 2U &&
           count_publishers(topics_.control) == 1U;
  }

private:
  Topics topics_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_publisher_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr vision_publisher_;
  rclcpp::Subscription<auto_aim_interfaces::msg::RobotCtrl>::SharedPtr control_subscription_;
  std::vector<auto_aim_interfaces::msg::RobotCtrl> controls_;
};

bool spin_until(
  const std::shared_ptr<FixtureProbe> & probe, const std::function<bool()> & predicate,
  std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(probe);
    if (predicate()) {
      return true;
    }
    if (caught_signal != 0) {
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }
  return predicate();
}

sensor_msgs::msg::Image valid_image(
  std::int32_t sec, std::uint32_t nanosec, std::uint32_t seed)
{
  sensor_msgs::msg::Image image;
  image.header.stamp.sec = sec;
  image.header.stamp.nanosec = nanosec;
  image.header.frame_id = "camera_optical_frame";
  image.width = 4U;
  image.height = 3U;
  image.encoding = "rgb8";
  image.step = 12U;
  image.data.resize(36U);
  for (std::size_t index = 0U; index < image.data.size(); ++index) {
    image.data[index] = static_cast<std::uint8_t>(
      (index * 17U + static_cast<std::size_t>(seed)) % 256U);
  }
  return image;
}

sensor_msgs::msg::CameraInfo valid_camera_info(std::int32_t sec, std::uint32_t nanosec)
{
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp.sec = sec;
  info.header.stamp.nanosec = nanosec;
  info.header.frame_id = "camera_optical_frame";
  info.width = 4U;
  info.height = 3U;
  info.distortion_model = "plumb_bob";
  info.k = {100.0, 0.0, 2.0, 0.0, 100.0, 1.5, 0.0, 0.0, 1.0};
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  return info;
}

auto_aim_interfaces::msg::Vision valid_vision(std::int32_t sec, std::uint32_t nanosec)
{
  auto_aim_interfaces::msg::Vision vision;
  vision.header.stamp.sec = sec;
  vision.header.stamp.nanosec = nanosec;
  vision.header.frame_id = "vision_header_clock";
  vision.yaw = 0.0F;
  vision.yaw_vel = 0.0F;
  vision.pitch = 0.0F;
  vision.pitch_vel = 0.0F;
  vision.roll = 0.0F;
  vision.quaternion = {1.0F, 0.0F, 0.0F, 0.0F};
  vision.shoot_speed = 0.0F;
  vision.bullet_count = 0U;
  vision.game_progress = 0U;
  return vision;
}

void mutate_fixture(
  Scenario scenario, sensor_msgs::msg::Image * image,
  std::optional<sensor_msgs::msg::CameraInfo> * camera)
{
  switch (scenario) {
    case Scenario::MissingCameraInfo:
      camera->reset();
      break;
    case Scenario::CameraDimensions:
      (*camera)->width = image->width + 1U;
      break;
    case Scenario::CameraNanK:
      (*camera)->k[0] = std::numeric_limits<double>::quiet_NaN();
      break;
    case Scenario::CameraInfD:
      (*camera)->d[0] = std::numeric_limits<double>::infinity();
      break;
    case Scenario::CameraZeroFocal:
      (*camera)->k[0] = 0.0;
      break;
    case Scenario::EmptyFrame:
      image->data.clear();
      break;
    case Scenario::ZeroDimensions:
      image->width = 0U;
      image->height = 0U;
      image->step = 0U;
      image->data.clear();
      break;
    case Scenario::WrongEncoding:
      image->encoding = "jpeg";
      break;
    case Scenario::ShortStride:
      image->step = 11U;
      image->data.resize(33U);
      break;
    case Scenario::ShortData:
      image->data.resize(35U);
      break;
    case Scenario::MissingTimestamp:
      image->header.stamp.sec = 0;
      image->header.stamp.nanosec = 0U;
      (*camera)->header.stamp = image->header.stamp;
      break;
    case Scenario::StampMismatch:
      (*camera)->header.stamp.nanosec += 1U;
      break;
    default:
      break;
  }
}

bool controls_have_safe_fields(
  const std::vector<auto_aim_interfaces::msg::RobotCtrl> & controls)
{
  return !controls.empty() && std::all_of(
    controls.begin(), controls.end(), [](const auto & control) {
      return control.fire_command == 0 && control.yaw_vel == 0.0F &&
             control.pitch_vel == 0.0F && control.yaw_acc == 0.0F &&
             control.pitch_acc == 0.0F;
    });
}

std::string target_lock_summary(
  const std::vector<auto_aim_interfaces::msg::RobotCtrl> & controls)
{
  const auto locked = std::count_if(controls.begin(), controls.end(), [](const auto & control) {
      return control.target_lock == kTargetLocked;
    });
  const auto unlocked = std::count_if(controls.begin(), controls.end(), [](const auto & control) {
      return control.target_lock == kTargetUnlocked;
    });
  return "locked=" + std::to_string(locked) + "; unlocked=" + std::to_string(unlocked);
}

bool group_has_serial_fd(pid_t group, std::string * detail)
{
  std::error_code error;
  for (const auto & entry : fs::directory_iterator("/proc", error)) {
    const auto name = entry.path().filename().string();
    if (name.empty() || !std::all_of(name.begin(), name.end(), ::isdigit)) {
      continue;
    }
    std::ifstream stat(entry.path() / "stat");
    std::string line;
    std::getline(stat, line);
    const auto close = line.rfind(')');
    if (close == std::string::npos) {
      continue;
    }
    std::istringstream fields(line.substr(close + 2U));
    char state = '\0';
    pid_t parent = 0;
    pid_t process_group = 0;
    fields >> state >> parent >> process_group;
    if (!fields || process_group != group) {
      continue;
    }
    for (const auto & fd : fs::directory_iterator(entry.path() / "fd", error)) {
      const auto target = fs::read_symlink(fd.path(), error).string();
      if (!error && (target.rfind("/dev/tty", 0) == 0 ||
        target.rfind("/dev/serial", 0) == 0))
      {
        *detail = target;
        return true;
      }
      error.clear();
    }
  }
  return false;
}

std::vector<std::string> node_command(
  const fs::path & executable, const Topics & topics, const std::string & node_name,
  const fs::path & csv_path, bool use_mock)
{
  return {
    executable.string(), "--ros-args",
    "-p", std::string("backend:=") + (use_mock ? "mock" : "null"),
    "-p", "dry_run:=true",
    "-p", "serial_enabled:=false",
    "-p", "allow_fire:=false",
    "-p", std::string("mock_target:=") + (use_mock ? "true" : "false"),
    "-p", "mock_yaw_rad:=0.25",
    "-p", "mock_pitch_rad:=0.0",
    "-p", "mock_fire_request:=true",
    "-p", "require_camera_info:=true",
    "-p", "input_timeout_ms:=120",
    "-p", "output_hz:=100.0",
    "-p", "vehicle_profile:=new_turtle",
    "-p", "vision_time_alignment_assume_shared_ros_clock:=true",
    "-p", "vision_time_alignment_tolerance_ns:=100000000",
    "-p", "csv_path:=" + csv_path.string(),
    "-r", "/image_raw:=" + topics.image,
    "-r", "/camera_info:=" + topics.camera_info,
    "-r", "/Vision_data:=" + topics.vision,
    "-r", "/Robot_ctrl_data:=" + topics.control,
    "-r", "__node:=" + node_name,
  };
}

std::vector<std::string> preflight_command(
  const fs::path & executable, const Topics & topics, const std::string & node_name,
  const fs::path & report_path)
{
  return {
    executable.string(), "--duration", "0.65", "--timeout", "1.0",
    "--format", "json", "--output", report_path.string(),
    "--vehicle-profile", "new_turtle", "--assume-shared-clock-domain",
    "--sync-tolerance-ms", "100", "--expected-frame-id", "camera_optical_frame",
    "--ros-args",
    "-r", "/image_raw:=" + topics.image,
    "-r", "/camera_info:=" + topics.camera_info,
    "-r", "/Vision_data:=" + topics.vision,
    "-r", "__node:=" + node_name,
  };
}

Artifact artifact_for(
  const fs::path & output_root, const fs::path & path, const std::string & role)
{
  return Artifact{
    fs::relative(path, output_root).generic_string(), role,
    auto_aim_ros_e2e::sha256_file(path)};
}

std::string fixture_summary(
  const ScenarioSpec & spec, const Topics & topics, const std::string & run_id)
{
  std::ostringstream stream;
  stream << "run_id=" << run_id << '\n'
         << "case=" << spec.id << '\n'
         << "summary=" << spec.input_summary << '\n'
         << "image_topic=" << topics.image << '\n'
         << "camera_info_topic=" << topics.camera_info << '\n'
         << "vision_topic=" << topics.vision << '\n'
         << "control_topic=" << topics.control << '\n'
         << "qos=SensorDataQoS(best_effort,volatile,keep_last,depth=5)\n"
         << "encoding=rgb8\nwidth=4\nheight=3\nstep=12\ndata_size=36\n"
         << "frame_id=camera_optical_frame\n"
         << "timestamp_source=fixed synthetic sec/nanosec sequence\n"
         << "fixture_identity=test-only C++ publisher and RobotCtrl subscriber\n"
         << "real_sender_connected=false\n";
  return stream.str();
}

CaseResult run_message_case(
  const Options & options, const fs::path & node_executable,
  const fs::path & preflight_executable, const ScenarioSpec & spec,
  std::size_t round, const std::string & suite_run_id, bool rerun = false)
{
  const std::string run_id = suite_run_id + "-r" + std::to_string(round) + "-" + spec.id +
    (rerun ? "-rerun" : "");
  const auto case_dir = options.output_dir /
    (rerun ? ("reruns/round-" + std::to_string(round)) :
    ("round-" + std::to_string(round))) / spec.id;
  if (fs::exists(case_dir)) {
    throw std::runtime_error("refusing to overwrite case directory: " + case_dir.string());
  }
  fs::create_directories(case_dir);
  const std::string root = "/issue33/" + sanitize_id(run_id);
  const Topics topics{
    root + "/image_raw", root + "/camera_info", root + "/Vision_data",
    root + "/Robot_ctrl_data"};
  const auto fixture_path = case_dir / "fixture.txt";
  auto_aim_ros_e2e::write_new_file(fixture_path, fixture_summary(spec, topics, run_id));

  const bool wrong_topic = spec.scenario == Scenario::WrongTopic;
  const bool wrong_qos = spec.scenario == Scenario::WrongQos;
  const std::string probe_name = "issue33_fixture_" + sanitize_id(run_id);
  const std::string auto_node_name = "issue33_auto_aim_" + sanitize_id(run_id);
  const std::string preflight_name = "issue33_preflight_" + sanitize_id(run_id);
  auto probe = std::make_shared<FixtureProbe>(probe_name, topics, wrong_topic, wrong_qos);
  Process node = spawn_process(
    node_command(
      node_executable, topics, auto_node_name, case_dir / "auto_aim_backend.csv",
      spec.use_mock),
    case_dir / "auto_aim_node.log");
  Process preflight = spawn_process(
    preflight_command(
      preflight_executable, topics, preflight_name, case_dir / "preflight.json"),
    case_dir / "preflight.log");

  bool graph_ready = false;
  bool serial_fd = false;
  std::string serial_detail;
  bool phase_locked = false;
  bool phase_unlocked = false;
  try {
    graph_ready = spin_until(probe, [&]() {return probe->graph_ready(wrong_topic);}, 3500ms);
    serial_fd = group_has_serial_fd(node.pid, &serial_detail);
    const auto publish_once = [&](std::int32_t sec, std::uint32_t nanosec) {
        auto image = valid_image(sec, nanosec, options.seed);
        std::optional<sensor_msgs::msg::CameraInfo> camera = valid_camera_info(sec, nanosec);
        mutate_fixture(spec.scenario, &image, &camera);
        probe->publish(image, camera, valid_vision(sec, nanosec));
        rclcpp::spin_some(probe);
      };

    if (spec.scenario == Scenario::NoInput || wrong_topic) {
      spin_until(probe, []() {return false;}, 500ms);
    } else if (spec.scenario == Scenario::TemporaryOcclusion) {
      for (int index = 0; index < 6; ++index) {
        publish_once(100 + index, 1U);
        std::this_thread::sleep_for(25ms);
      }
      spin_until(probe, [&]() {
        phase_locked = std::any_of(
          probe->controls().begin(), probe->controls().end(), [](const auto & control) {
            return control.target_lock == kTargetLocked;
          });
        phase_unlocked = !probe->controls().empty() &&
          probe->controls().back().target_lock == kTargetUnlocked;
        return phase_locked && phase_unlocked;
      }, 350ms);
      for (int index = 0; index < 6; ++index) {
        publish_once(200 + index, 1U);
        std::this_thread::sleep_for(25ms);
      }
    } else if (spec.scenario == Scenario::ContinuousInterruption) {
      for (int index = 0; index < 6; ++index) {
        publish_once(100 + index, 1U);
        std::this_thread::sleep_for(25ms);
      }
      spin_until(probe, [&]() {
        phase_locked = std::any_of(
          probe->controls().begin(), probe->controls().end(), [](const auto & control) {
            return control.target_lock == kTargetLocked;
          });
        phase_unlocked = !probe->controls().empty() &&
          probe->controls().back().target_lock == kTargetUnlocked;
        return phase_locked && phase_unlocked;
      }, 400ms);
    } else if (spec.scenario == Scenario::ReorderedValid) {
      for (int index = 0; index < 12; ++index) {
        const auto sec = 100 + index;
        const auto camera = valid_camera_info(sec, 1U);
        probe->publish_camera_info(camera);
        std::this_thread::sleep_for(3ms);
        probe->publish_vision(valid_vision(sec, 1U));
        probe->publish_image(valid_image(sec, 1U, options.seed));
        rclcpp::spin_some(probe);
        std::this_thread::sleep_for(30ms);
      }
    } else {
      for (int index = 0; index < 16; ++index) {
        std::int32_t sec = 100 + index;
        if (spec.scenario == Scenario::StampRollback) {
          sec = index % 2 == 0 ? 101 : 100;
        } else if (spec.scenario == Scenario::StampDuplicate) {
          sec = 100;
        }
        publish_once(sec, 1U);
        std::this_thread::sleep_for(30ms);
      }
    }
    spin_until(probe, [&]() {return !probe->controls().empty();}, 800ms);
    if (!wait_process(&preflight, 2500ms)) {
      stop_process(&preflight);
    }
  } catch (...) {
    stop_process(&preflight);
    stop_process(&node);
    probe.reset();
    throw;
  }

  const std::string publishers = probe->publisher_observation(auto_node_name);
  const auto controls = probe->controls();
  const bool fields_safe = controls_have_safe_fields(controls);
  const bool has_locked = std::any_of(controls.begin(), controls.end(), [](const auto & control) {
      return control.target_lock == kTargetLocked;
    });
  const bool all_unlocked = !controls.empty() &&
    std::all_of(controls.begin(), controls.end(), [](const auto & control) {
      return control.target_lock == kTargetUnlocked;
    });
  const bool final_unlocked = !controls.empty() && controls.back().target_lock == kTargetUnlocked;
  const bool preflight_cleanup = stop_process(&preflight);
  const bool node_cleanup = stop_process(&node);
  const bool cleanup_ok = preflight_cleanup && node_cleanup;
  probe.reset();

  const auto preflight_report = read_text(case_dir / "preflight.json");
  const bool preflight_reported_fail =
    preflight_report.find("\"overall\": \"FAIL\"") != std::string::npos;
  const bool preflight_status_ok = spec.preflight_fail ?
    preflight_reported_fail && preflight.exit_code == 2 :
    !preflight_reported_fail && preflight.exit_code == 0;
  bool lock_ok = true;
  if (spec.scenario == Scenario::ValidMock || spec.scenario == Scenario::ReorderedValid) {
    lock_ok = has_locked;
  } else if (spec.scenario == Scenario::TemporaryOcclusion) {
    lock_ok = phase_locked && phase_unlocked && has_locked;
  } else if (spec.scenario == Scenario::ContinuousInterruption) {
    lock_ok = phase_locked && phase_unlocked && final_unlocked;
  } else {
    lock_ok = all_unlocked;
  }
  const bool publisher_ok = publishers.find("control_publishers=1") != std::string::npos &&
    publishers.find("auto_node_found=true") != std::string::npos &&
    publishers.find("preflight_control_publisher=false") != std::string::npos &&
    publishers.find("unremapped_control_publishers=0") != std::string::npos;
  const bool passed = graph_ready && !serial_fd && fields_safe && lock_ok &&
    preflight_status_ok && publisher_ok && cleanup_ok;

  CaseResult result;
  result.round = round;
  result.id = spec.id;
  result.run_id = run_id;
  result.status = passed ? Status::Pass : Status::Fail;
  result.input_summary = spec.input_summary;
  result.expected = spec.expected;
  result.actual = std::string("graph=") + (graph_ready ? "ready" : "not_ready") +
    "; preflight=" + (preflight_reported_fail ? "FAIL" : "PASS/WARN") +
    "; safe_fields=" + (fields_safe ? "true" : "false") +
    "; " + target_lock_summary(controls);
  result.diagnostic = std::string("serial_fd=") + (serial_fd ? serial_detail : "none") +
    "; preflight_expectation=" + (preflight_status_ok ? "matched" : "mismatch") +
    "; publisher_contract=" + (publisher_ok ? "matched" : "mismatch");
  result.node_exit_code = node.exit_code.value_or(-1);
  result.preflight_exit_code = preflight.exit_code.value_or(-1);
  result.topics = "image=" + topics.image + "; camera_info=" + topics.camera_info +
    "; vision=" + topics.vision + "; control=" + topics.control;
  result.publishers = publishers;
  result.control_messages = controls.size();
  result.safety_fields_ok = fields_safe;
  result.target_lock = target_lock_summary(controls);
  result.cleanup_ok = cleanup_ok;
  result.rerun = rerun;
  for (const auto & item : std::array<std::pair<fs::path, const char *>, 5>{
      std::pair{fixture_path, "synthetic input summary"},
      std::pair{case_dir / "auto_aim_node.log", "AutoAimNode log"},
      std::pair{case_dir / "preflight.log", "preflight process log"},
      std::pair{case_dir / "preflight.json", "preflight JSON"},
      std::pair{case_dir / "auto_aim_backend.csv", "existing backend diagnostic CSV"},
    })
  {
    if (fs::is_regular_file(item.first)) {
      result.artifacts.push_back(artifact_for(options.output_dir, item.first, item.second));
    }
  }
  return result;
}

CaseResult run_expected_failure_case(
  const Options & options, const std::string & id, const std::string & run_id,
  const std::string & input_summary, const std::string & expected,
  const std::vector<std::string> & command, const std::string & required_diagnostic,
  std::size_t round, bool rerun = false)
{
  const auto case_dir = options.output_dir /
    (rerun ? ("reruns/round-" + std::to_string(round)) :
    ("round-" + std::to_string(round))) / id;
  if (fs::exists(case_dir)) {
    throw std::runtime_error("refusing to overwrite case directory: " + case_dir.string());
  }
  fs::create_directories(case_dir);
  const auto input_path = case_dir / "input.txt";
  auto_aim_ros_e2e::write_new_file(
    input_path, "run_id=" + run_id + "\ncase=" + id + "\ncommand=" +
    command_text(command) + "\nserial_enabled=false\ndry_run=true\nallow_fire=false\n");
  Process process = spawn_process(command, case_dir / "process.log");
  bool timed_out = !wait_process(&process, 5s);
  const bool cleanup_ok = stop_process(&process);
  const auto output = read_text(case_dir / "process.log");
  const bool diagnostic_ok = required_diagnostic.empty() ||
    output.find(required_diagnostic) != std::string::npos;
  const bool passed = !timed_out && process.exit_code.value_or(0) != 0 &&
    diagnostic_ok && cleanup_ok;

  CaseResult result;
  result.round = round;
  result.id = id;
  result.run_id = run_id;
  result.status = passed ? Status::Pass : Status::Fail;
  result.input_summary = input_summary;
  result.expected = expected;
  result.actual = std::string("exit=") + std::to_string(process.exit_code.value_or(-1)) +
    "; timed_out=" + (timed_out ? "true" : "false");
  result.diagnostic = diagnostic_ok ? "expected fail-closed diagnostic observed" :
    "expected diagnostic missing";
  result.node_exit_code = process.exit_code.value_or(-1);
  result.preflight_exit_code = -1;
  result.publishers = "startup failed before any approved control path was connected";
  result.control_messages = 0U;
  result.safety_fields_ok = true;
  result.target_lock = "no control publisher/message; startup failed closed";
  result.cleanup_ok = cleanup_ok;
  result.rerun = rerun;
  result.artifacts.push_back(artifact_for(options.output_dir, input_path, "failure fixture input"));
  result.artifacts.push_back(
    artifact_for(options.output_dir, case_dir / "process.log", "failure process log"));
  return result;
}

enum class LifecycleMode
{
  Natural,
  RepeatedStop,
  Timeout,
  Abnormal,
};

CaseResult run_lifecycle_case(
  const Options & options, const fs::path & preflight_executable,
  LifecycleMode mode, std::size_t round, const std::string & suite_run_id,
  bool rerun = false)
{
  const std::string id = mode == LifecycleMode::Natural ? "process_normal_stop" :
    mode == LifecycleMode::RepeatedStop ? "process_repeated_stop" :
    mode == LifecycleMode::Timeout ? "process_timeout_cleanup" :
    "process_abnormal_exit";
  const std::string run_id = suite_run_id + "-r" + std::to_string(round) + "-" + id +
    (rerun ? "-rerun" : "");
  const auto case_dir = options.output_dir /
    (rerun ? ("reruns/round-" + std::to_string(round)) :
    ("round-" + std::to_string(round))) / id;
  if (fs::exists(case_dir)) {
    throw std::runtime_error("refusing to overwrite case directory: " + case_dir.string());
  }
  fs::create_directories(case_dir);
  std::vector<std::string> command{
    preflight_executable.string(), "--duration",
    mode == LifecycleMode::Natural ? "0.15" : "30", "--timeout", "0.05",
    "--format", "json", "--output", (case_dir / "preflight.json").string(),
  };
  const auto input_path = case_dir / "input.txt";
  auto_aim_ros_e2e::write_new_file(
    input_path, "run_id=" + run_id + "\ncase=" + id + "\ncommand=" +
    command_text(command) + "\nfixture=no ROS input\n");
  Process process = spawn_process(command, case_dir / "process.log");
  bool cleanup_ok = false;
  bool timeout_triggered = false;
  if (mode == LifecycleMode::Natural) {
    cleanup_ok = wait_process(&process, 3s) && stop_process(&process);
  } else {
    std::this_thread::sleep_for(300ms);
    if (mode == LifecycleMode::RepeatedStop) {
      cleanup_ok = stop_process(&process, true, false);
    } else if (mode == LifecycleMode::Abnormal) {
      cleanup_ok = stop_process(&process, false, true);
    } else {
      timeout_triggered = !wait_process(&process, 250ms);
      cleanup_ok = stop_process(&process);
    }
  }
  const int exit_code = process.exit_code.value_or(-1);
  const bool exit_ok = mode == LifecycleMode::Natural ? exit_code == 2 :
    mode == LifecycleMode::RepeatedStop ? exit_code == 130 :
    mode == LifecycleMode::Abnormal ? exit_code == 137 : timeout_triggered && exit_code != 0;
  const bool passed = exit_ok && cleanup_ok;

  CaseResult result;
  result.round = round;
  result.id = id;
  result.run_id = run_id;
  result.status = passed ? Status::Pass : Status::Fail;
  result.input_summary = "No-input preflight lifecycle fixture";
  result.expected = mode == LifecycleMode::Natural ?
    "natural fail-closed exit=2" : mode == LifecycleMode::RepeatedStop ?
    "two SIGINT requests are idempotent; exit=130" : mode == LifecycleMode::Abnormal ?
    "SIGKILL exit=137 and process group reaped" :
    "runner timeout triggers bounded cleanup and leaves no process";
  result.actual = "exit=" + std::to_string(exit_code) +
    "; timeout_triggered=" + (timeout_triggered ? "true" : "false");
  result.diagnostic = cleanup_ok ? "process group fully reaped" : "residual process group";
  result.node_exit_code = -1;
  result.preflight_exit_code = exit_code;
  result.publishers = "read-only preflight; no RobotCtrl publisher";
  result.control_messages = 0U;
  result.safety_fields_ok = true;
  result.target_lock = "not applicable: preflight is subscription-only";
  result.cleanup_ok = cleanup_ok;
  result.rerun = rerun;
  result.artifacts.push_back(artifact_for(options.output_dir, input_path, "lifecycle input"));
  result.artifacts.push_back(
    artifact_for(options.output_dir, case_dir / "process.log", "lifecycle process log"));
  if (fs::is_regular_file(case_dir / "preflight.json")) {
    result.artifacts.push_back(
      artifact_for(options.output_dir, case_dir / "preflight.json", "lifecycle preflight JSON"));
  }
  return result;
}

CaseResult unavailable_offline_case(
  std::size_t round, const std::string & suite_run_id)
{
  CaseResult result;
  result.round = round;
  result.id = "offline_reference_valid_artifacts";
  result.run_id = suite_run_id + "-r" + std::to_string(round) + "-offline-unavailable";
  result.status = Status::Unavailable;
  result.input_summary = "Valid formal XML/BIN, model profile, and calibration were requested";
  result.expected = "Run only when reviewed artifacts are available";
  result.actual = "Repository intentionally contains no formal model/calibration bundle";
  result.diagnostic = "missing dependency remains UNAVAILABLE, never PASS";
  result.publishers = "not started";
  result.safety_fields_ok = true;
  result.target_lock = "no output";
  result.cleanup_ok = true;
  return result;
}

void usage(std::ostream & stream)
{
  stream <<
    "Usage: auto_aim_ros_e2e --install-base PATH --output-dir NEW_PATH "
    "--baseline SHA --commit SHA [--rounds N] [--seed N]\n";
}

std::size_t positive_size(const std::string & value, const std::string & option)
{
  std::size_t consumed = 0U;
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size() || parsed == 0U || parsed > 100U) {
    throw std::invalid_argument(option + " must be in [1, 100]");
  }
  return static_cast<std::size_t>(parsed);
}

std::uint32_t parse_seed(const std::string & value)
{
  std::size_t consumed = 0U;
  const auto parsed = std::stoull(value, &consumed);
  if (consumed != value.size() || parsed == 0U ||
    parsed > std::numeric_limits<std::uint32_t>::max())
  {
    throw std::invalid_argument("--seed must be a positive uint32");
  }
  return static_cast<std::uint32_t>(parsed);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto value = [&](const char * name) -> std::string {
        if (index + 1 >= argc) {
          throw std::invalid_argument(std::string(name) + " requires a value");
        }
        return argv[++index];
      };
    if (argument == "--install-base") {
      options.install_base = value("--install-base");
    } else if (argument == "--output-dir") {
      options.output_dir = value("--output-dir");
    } else if (argument == "--baseline") {
      options.baseline = value("--baseline");
    } else if (argument == "--commit") {
      options.commit = value("--commit");
    } else if (argument == "--rounds") {
      options.rounds = positive_size(value("--rounds"), "--rounds");
    } else if (argument == "--seed") {
      options.seed = parse_seed(value("--seed"));
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
  if (!auto_aim_ros_e2e::valid_git_sha(options.baseline) ||
    !auto_aim_ros_e2e::valid_git_sha(options.commit))
  {
    throw std::invalid_argument("baseline and commit must be full Git SHAs");
  }
  return options;
}

std::string environment_description()
{
  struct utsname details {};
  std::ostringstream stream;
  if (::uname(&details) == 0) {
    stream << details.sysname << ' ' << details.release << ' ' << details.machine;
  }
  stream << "; ROS_DISTRO=" << (std::getenv("ROS_DISTRO") == nullptr ?
    "UNSET" : std::getenv("ROS_DISTRO"));
  std::ifstream release("/etc/os-release");
  std::string line;
  while (std::getline(release, line)) {
    if (line.rfind("PRETTY_NAME=", 0) == 0) {
      stream << "; " << line.substr(12U);
      break;
    }
  }
  return stream.str();
}

std::vector<ScenarioSpec> message_scenarios()
{
  return {
    {"valid_rgb8_null", Scenario::ValidNull,
      "Valid rgb8 Image plus matching finite CameraInfo; null backend",
      "preflight accepts input; existing null backend publishes unlocked safe controls", false, false},
    {"valid_rgb8_mock", Scenario::ValidMock,
      "Valid rgb8 Image plus matching finite CameraInfo; mock backend",
      "preflight accepts input; mock may lock but fire and motion derivatives remain zero", false, true},
    {"missing_camera_info", Scenario::MissingCameraInfo,
      "Valid Image with no CameraInfo", "preflight fails and mock remains unlocked", true, true},
    {"camera_dimensions_mismatch", Scenario::CameraDimensions,
      "Image and CameraInfo dimensions differ", "preflight fails and mock remains unlocked", true, true},
    {"camera_k_nan", Scenario::CameraNanK,
      "CameraInfo K contains NaN", "preflight fails and mock remains unlocked", true, true},
    {"camera_d_inf", Scenario::CameraInfD,
      "CameraInfo D contains Inf", "preflight fails and mock remains unlocked", true, true},
    {"camera_zero_focal", Scenario::CameraZeroFocal,
      "CameraInfo fx is zero", "preflight fails and mock remains unlocked", true, true},
    {"image_empty_frame", Scenario::EmptyFrame,
      "Image data is empty", "preflight and adapter reject; mock remains unlocked", true, true},
    {"image_zero_dimensions", Scenario::ZeroDimensions,
      "Image width and height are zero", "preflight and adapter reject; mock remains unlocked", true, true},
    {"image_wrong_encoding", Scenario::WrongEncoding,
      "Image encoding is unsupported jpeg", "preflight and adapter reject; mock remains unlocked", true, true},
    {"image_stride_mismatch", Scenario::ShortStride,
      "Image step is smaller than rgb8 row width", "preflight and adapter reject; mock remains unlocked", true, true},
    {"image_data_short", Scenario::ShortData,
      "Image data is shorter than height*step", "preflight and adapter reject; mock remains unlocked", true, true},
    {"timestamp_missing", Scenario::MissingTimestamp,
      "Image and CameraInfo timestamps are zero/unset", "preflight and adapter reject; mock remains unlocked", true, true},
    {"timestamp_pair_mismatch", Scenario::StampMismatch,
      "Image and CameraInfo timestamps differ by 1 ns", "preflight fails; null output remains unlocked", true, false},
    {"timestamp_rollback", Scenario::StampRollback,
      "Paired timestamps alternate backwards", "preflight reports rollback; null output remains unlocked", true, false},
    {"timestamp_duplicate", Scenario::StampDuplicate,
      "Paired timestamps repeat", "preflight records duplicate warning; null output remains unlocked", false, false},
    {"wrong_topic", Scenario::WrongTopic,
      "Fixture publishes Image/CameraInfo to incorrect topic names", "preflight receives no input; node times out safely", true, false},
    {"wrong_qos", Scenario::WrongQos,
      "Fixture uses reliable keep_last depth 10 instead of SensorDataQoS",
      "preflight flags QoS contract; null output remains unlocked", true, false},
    {"valid_reordered_delivery", Scenario::ReorderedValid,
      "CameraInfo arrives before same-stamp Image", "pairing remains valid and mock output stays fire-inhibited", false, true},
    {"temporary_occlusion", Scenario::TemporaryOcclusion,
      "Valid frames, pause beyond input watchdog, then resume",
      "mock lock transitions through unlocked safe hold and can reacquire", false, true},
    {"continuous_frame_interruption", Scenario::ContinuousInterruption,
      "Valid frames stop continuously beyond input watchdog",
      "final control is unlocked safe hold", false, true},
    {"node_start_no_input", Scenario::NoInput,
      "Node starts and receives no input", "preflight fails; controls remain unlocked safe hold", true, false},
  };
}

std::vector<std::string> offline_node_failure_command(
  const fs::path & node_executable, const std::string & node_name,
  const fs::path & model, const fs::path & profile, const fs::path & pnp,
  bool invalid_backend = false)
{
  std::vector<std::string> command{
    node_executable.string(), "--ros-args",
    "-p", std::string("backend:=") + (invalid_backend ? "invalid" : "offline_reference"),
    "-p", "dry_run:=true", "-p", "serial_enabled:=false", "-p", "allow_fire:=false",
    "-p", "vehicle_profile:=new_turtle", "-p", "allow_test_only:=true",
  };
  if (!invalid_backend) {
    if (!model.empty()) {
      command.insert(command.end(), {"-p", "offline_model_path:=" + model.string()});
    }
    if (!profile.empty()) {
      command.insert(command.end(), {"-p", "offline_model_profile:=" + profile.string()});
    }
    if (!pnp.empty()) {
      command.insert(command.end(), {"-p", "offline_pnp_config:=" + pnp.string()});
    }
  }
  command.insert(command.end(), {"-r", "__node:=" + node_name});
  return command;
}

int run_suite(const Options & options)
{
  if (!fs::is_directory(options.install_base)) {
    throw std::invalid_argument("install base is not a directory: " + options.install_base.string());
  }
  if (fs::exists(options.output_dir)) {
    throw std::invalid_argument(
            "output directory must not already exist: " + options.output_dir.string());
  }
  const char * domain = std::getenv("ROS_DOMAIN_ID");
  if (domain == nullptr || std::string(domain).empty()) {
    throw std::invalid_argument("ROS_DOMAIN_ID must be explicitly set for isolation");
  }
  fs::create_directories(options.output_dir / "runtime" / "ros_home");
  fs::create_directories(options.output_dir / "runtime" / "ros_logs");
  ::setenv("ROS2CLI_NO_DAEMON", "1", 1);
  ::setenv("ROS_LOCALHOST_ONLY", "1", 1);
  ::setenv("ROS_HOME", (options.output_dir / "runtime" / "ros_home").c_str(), 1);
  ::setenv("ROS_LOG_DIR", (options.output_dir / "runtime" / "ros_logs").c_str(), 1);

  const auto node_executable = executable_path(
    options.install_base, "auto_aim_ros2", "auto_aim_node");
  const auto preflight_executable = executable_path(
    options.install_base, "auto_aim_tools", "ros_input_preflight");
  const auto offline_executable = executable_path(
    options.install_base, "auto_aim_ros2", "auto_aim_offline");
  for (const auto & executable : {node_executable, preflight_executable, offline_executable}) {
    if (!fs::is_regular_file(executable)) {
      throw std::invalid_argument("installed executable is missing: " + executable.string());
    }
  }
  const auto fixture_dir = package_prefix(options.install_base, "auto_aim_ros_e2e") /
    "share" / "auto_aim_ros_e2e" / "fixtures";
  const auto profile = fixture_dir / "model_profile_test.yaml";
  const auto pnp = fixture_dir / "pnp_test_config.yaml";
  if (!fs::is_regular_file(profile) || !fs::is_regular_file(pnp)) {
    throw std::invalid_argument("installed test-only profile/PnP fixtures are missing");
  }

  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const std::string suite_run_id = "issue33-" + std::to_string(::getpid()) + "-" +
    std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  std::vector<CaseResult> results;
  const auto scenarios = message_scenarios();
  bool initialized = false;
  try {
    int argc = 1;
    char name[] = "auto_aim_ros_e2e";
    char * argv[] = {name, nullptr};
    rclcpp::init(
      argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    initialized = true;
    for (std::size_t round = 1U; round <= options.rounds; ++round) {
      for (const auto & scenario : scenarios) {
        throw_if_interrupted();
        auto result = run_message_case(
          options, node_executable, preflight_executable, scenario, round, suite_run_id);
        const bool failed = result.status == Status::Fail;
        results.push_back(std::move(result));
        if (failed) {
          results.push_back(run_message_case(
            options, node_executable, preflight_executable, scenario, round,
            suite_run_id, true));
        }
      }
      for (const auto mode : {
          LifecycleMode::Natural, LifecycleMode::RepeatedStop,
          LifecycleMode::Timeout, LifecycleMode::Abnormal})
      {
        throw_if_interrupted();
        auto result = run_lifecycle_case(
          options, preflight_executable, mode, round, suite_run_id);
        const bool failed = result.status == Status::Fail;
        results.push_back(std::move(result));
        if (failed) {
          results.push_back(run_lifecycle_case(
            options, preflight_executable, mode, round, suite_run_id, true));
        }
      }

      struct FailureSpec
      {
        std::string id;
        std::string summary;
        std::string expected;
        std::vector<std::string> command;
        std::string diagnostic;
      };
      const auto missing_root = options.output_dir / "missing-inputs";
      std::vector<FailureSpec> failures;
      failures.push_back({
          "wrong_launch_parameter", "Unsupported backend launch parameter",
          "AutoAimNode rejects startup without creating a control path",
          offline_node_failure_command(
            node_executable, "issue33_invalid_backend_" + std::to_string(round),
            {}, {}, {}, true),
          "backend must be one of"});
      failures.push_back({
          "offline_missing_pnp", "offline_reference uses a missing PnP/calibration file",
          "startup fails closed before inference",
          offline_node_failure_command(
            node_executable, "issue33_missing_pnp_" + std::to_string(round),
            missing_root / "model.xml", profile, missing_root / "pnp.yaml"),
          "pnp"});
      failures.push_back({
          "offline_missing_profile", "offline_reference has no model profile",
          "startup fails closed before inference",
          offline_node_failure_command(
            node_executable, "issue33_missing_profile_" + std::to_string(round),
            missing_root / "model.xml", {}, pnp),
          "offline_model_profile"});
      failures.push_back({
          "offline_missing_model", "offline_reference model XML/BIN is absent",
          "startup fails closed; missing runtime/artifact is not a successful inference",
          offline_node_failure_command(
            node_executable, "issue33_missing_model_" + std::to_string(round),
            missing_root / "model.xml", profile, pnp),
          ""});
      failures.push_back({
          "offline_missing_input_file", "offline CLI video input is absent",
          "offline CLI rejects the path before opening model, camera, or serial resources",
          {offline_executable.string(), "--model", (missing_root / "model.xml").string(),
            "--model-profile", profile.string(), "--video",
            (missing_root / "input.avi").string(), "--pnp-config", pnp.string(),
            "--allow-test-profile", "--allow-test-config", "--csv",
            (missing_root / "output.csv").string()},
          "video input"});
      for (const auto & failure : failures) {
        throw_if_interrupted();
        const std::string run_id = suite_run_id + "-r" + std::to_string(round) + "-" + failure.id;
        auto result = run_expected_failure_case(
          options, failure.id, run_id, failure.summary, failure.expected,
          failure.command, failure.diagnostic, round);
        const bool failed = result.status == Status::Fail;
        results.push_back(std::move(result));
        if (failed) {
          results.push_back(run_expected_failure_case(
            options, failure.id, run_id + "-rerun", failure.summary, failure.expected,
            failure.command, failure.diagnostic, round, true));
        }
      }
      results.push_back(unavailable_offline_case(round, suite_run_id));
    }
    rclcpp::shutdown();
    initialized = false;
  } catch (...) {
    if (initialized && rclcpp::ok()) {
      rclcpp::shutdown();
    }
    throw;
  }

  std::map<std::string, std::tuple<Status, bool, bool>> signatures;
  std::map<std::string, bool> deterministic;
  for (const auto & result : results) {
    if (result.rerun) {
      continue;
    }
    const auto signature = std::make_tuple(
      result.status, result.safety_fields_ok, result.cleanup_ok);
    const auto inserted = signatures.emplace(result.id, signature);
    if (!inserted.second && inserted.first->second != signature) {
      deterministic[result.id] = false;
    } else if (deterministic.find(result.id) == deterministic.end()) {
      deterministic[result.id] = true;
    }
  }
  for (auto & result : results) {
    result.deterministic = deterministic[result.id];
    if (!result.deterministic && !result.rerun) {
      result.status = Status::Fail;
      result.diagnostic += "; five-round status summary was inconsistent";
    }
  }
  for (auto & rerun : results) {
    if (!rerun.rerun) {
      continue;
    }
    const auto original = std::find_if(results.begin(), results.end(), [&](const CaseResult & item) {
        return !item.rerun && item.round == rerun.round && item.id == rerun.id;
      });
    if (original != results.end() && original->status != rerun.status) {
      original->flaky = true;
      rerun.flaky = true;
      original->diagnostic += "; rerun status changed: flaky=true";
      rerun.diagnostic += "; differs from first failure: flaky=true";
    }
  }

  for (const auto & item : std::array<std::pair<const char *, const char *>, 8>{
      std::pair{"real_camera", "no real camera or MVS SDK path was run"},
      std::pair{"orin", "not executed on an Orin target"},
      std::pair{"formal_model_calibration", "formal model, K/D, and extrinsics were absent"},
      std::pair{"cdc_serial", "no CDC serial sender was started or opened"},
      std::pair{"robot", "no robot was connected"},
      std::pair{"gimbal_motion", "no gimbal motion was requested"},
      std::pair{"firing", "firing was inhibited and not exercised"},
      std::pair{"competition_performance", "accuracy, latency, and match performance were not measured"},
    })
  {
    CaseResult result;
    result.id = std::string("hardware_") + item.first;
    result.run_id = suite_run_id + "-" + result.id;
    result.status = Status::NotVerified;
    result.input_summary = "hardware/formal evidence boundary";
    result.expected = "must remain explicitly NOT_VERIFIED";
    result.actual = item.second;
    result.diagnostic = "message-level dry-run cannot establish this claim";
    result.publishers = "not started";
    result.safety_fields_ok = true;
    result.target_lock = "not applicable";
    result.cleanup_ok = true;
    results.push_back(std::move(result));
  }

  std::vector<Artifact> artifacts;
  std::error_code error;
  for (const auto & entry : fs::recursive_directory_iterator(options.output_dir, error)) {
    if (!error && entry.is_regular_file()) {
      artifacts.push_back(artifact_for(options.output_dir, entry.path(), "case/runtime artifact"));
    }
  }
  std::sort(artifacts.begin(), artifacts.end(), [](const Artifact & left, const Artifact & right) {
      return left.path < right.path;
    });
  const auto metadata = auto_aim_ros_e2e::ReportMetadata{
    options.baseline, options.commit, suite_run_id, environment_description(),
    domain, std::to_string(options.seed), options.rounds};
  const auto json_path = options.output_dir / "ros-message-e2e-report.json";
  const auto markdown_path = options.output_dir / "ros-message-e2e-report.md";
  auto_aim_ros_e2e::write_new_file(
    json_path, auto_aim_ros_e2e::render_json(metadata, results, artifacts));
  auto_aim_ros_e2e::write_new_file(
    markdown_path, auto_aim_ros_e2e::render_markdown(metadata, results, artifacts));

  std::vector<fs::path> checksum_files;
  for (const auto & entry : fs::recursive_directory_iterator(options.output_dir)) {
    if (entry.is_regular_file() && entry.path().filename() != "SHA256SUMS") {
      checksum_files.push_back(entry.path());
    }
  }
  std::sort(checksum_files.begin(), checksum_files.end());
  std::ostringstream checksums;
  for (const auto & path : checksum_files) {
    checksums << auto_aim_ros_e2e::sha256_file(path) << "  "
              << fs::relative(path, options.output_dir).generic_string() << '\n';
  }
  auto_aim_ros_e2e::write_new_file(options.output_dir / "SHA256SUMS", checksums.str());

  const bool failed = std::any_of(results.begin(), results.end(), [](const CaseResult & result) {
      return result.status == Status::Fail;
    });
  std::cout << "status=" << (failed ? "FAIL" : "PASS") << '\n'
            << "run_id=" << suite_run_id << '\n'
            << "json=" << json_path << '\n'
            << "markdown=" << markdown_path << '\n'
            << "sha256=" << (options.output_dir / "SHA256SUMS") << '\n';
  return failed ? 1 : 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  try {
    return run_suite(parse_options(argc, argv));
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << "auto_aim_ros_e2e: " << error.what() << '\n';
    usage(std::cerr);
    return caught_signal == 0 ? 2 : 128 + caught_signal;
  }
}
