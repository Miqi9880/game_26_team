#include <gtest/gtest.h>

#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "auto_aim_interfaces/msg/vision.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace auto_aim_tools
{
namespace
{
using namespace std::chrono_literals;

class TemporaryFile
{
public:
  explicit TemporaryFile(const std::string & label)
  : path_(
      "/tmp/auto_aim_tools_" + label + "_" + std::to_string(getpid()) + "_" +
      std::to_string(counter_++) + ".json")
  {
    std::ofstream(path_, std::ios::trunc).close();
  }

  ~TemporaryFile()
  {
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }

  const std::string & path() const {return path_;}

private:
  static std::uint32_t counter_;
  std::string path_;
};

std::uint32_t TemporaryFile::counter_ = 0U;

pid_t launch_process(
  const std::vector<std::string> & arguments, const std::string & stdout_path = "",
  const std::string & stderr_path = "")
{
  const pid_t pid = fork();
  if (pid != 0) {
    return pid;
  }

  if (!stdout_path.empty()) {
    const int output = open(stdout_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (output < 0 || dup2(output, STDOUT_FILENO) < 0) {
      _exit(126);
    }
    close(output);
  }
  if (!stderr_path.empty()) {
    const int output = open(stderr_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
    if (output < 0 || dup2(output, STDERR_FILENO) < 0) {
      _exit(126);
    }
    close(output);
  }
  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1U);
  for (const auto & argument : arguments) {
    argv.push_back(const_cast<char *>(argument.c_str()));
  }
  argv.push_back(nullptr);
  execvp(argv.front(), argv.data());
  _exit(127);
}

int wait_for_process(pid_t pid, std::chrono::seconds timeout = 10s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  int status = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    if (waitpid(pid, &status, WNOHANG) == pid) {
      if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
      }
      if (WIFSIGNALED(status)) {
        return 128 + WTERMSIG(status);
      }
    }
    std::this_thread::sleep_for(10ms);
  }
  kill(pid, SIGKILL);
  waitpid(pid, &status, 0);
  return -1;
}

bool parses_as_json(const std::string & path)
{
  const pid_t parser = launch_process(
    {"python3", "-m", "json.tool", path}, "/dev/null");
  return parser > 0 && wait_for_process(parser) == 0;
}

std::string read_file(const std::string & path)
{
  std::ifstream input(path, std::ios::binary);
  return std::string(
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

bool json_has_overall(const std::string & path, const std::string & status)
{
  const pid_t parser = launch_process(
    {"python3", "-c",
      "import json,sys; data=json.load(open(sys.argv[1], encoding='utf-8')); "
      "assert data['overall'] == sys.argv[2]",
      path, status},
    "/dev/null");
  return parser > 0 && wait_for_process(parser) == 0;
}

bool json_has_failing_finding(
  const std::string & path, const std::string & check,
  const std::string & reason_fragment)
{
  const pid_t parser = launch_process(
    {"python3", "-c",
      "import json,sys; data=json.load(open(sys.argv[1], encoding='utf-8')); "
      "assert any(item.get('check') == sys.argv[2] and item.get('status') == 'FAIL' "
      "and sys.argv[3] in item.get('reason', '') for item in data['findings'])",
      path, check, reason_fragment},
    "/dev/null");
  return parser > 0 && wait_for_process(parser) == 0;
}

std::vector<std::string> direct_arguments(
  const std::string & output, const std::string & duration)
{
  return {
    PREFLIGHT_EXECUTABLE_PATH, "--duration", duration, "--timeout", "1.0",
    "--vehicle-profile", "new_turtle", "--assume-shared-clock-domain",
    "--format", "json", "--output", output,
  };
}

enum class InputScenario : std::uint8_t
{
  Valid,
  InvalidImage,
  InvalidCameraInfo,
  InvalidVision,
};

class ContractPublisher final : public rclcpp::Node
{
public:
  explicit ContractPublisher(InputScenario scenario)
  : Node("preflight_process_contract_publisher"), scenario_(scenario)
  {
    image_ = create_publisher<sensor_msgs::msg::Image>("/image_raw", rclcpp::SensorDataQoS());
    camera_info_ = create_publisher<sensor_msgs::msg::CameraInfo>(
      "/camera_info", rclcpp::SensorDataQoS());
    vision_ = create_publisher<auto_aim_interfaces::msg::Vision>(
      "/Vision_data", rclcpp::SensorDataQoS());
  }

  void publish(std::int32_t stamp)
  {
    sensor_msgs::msg::Image image;
    image.header.stamp.sec = stamp;
    image.header.frame_id = "camera_optical_frame";
    image.width = 2U;
    image.height = 2U;
    image.encoding = "rgb8";
    image.step = 6U;
    image.data.resize(12U);
    if (scenario_ == InputScenario::InvalidImage) {
      image.height = 1U;
      image.encoding = "64UC1152921504606846976";
      image.step = 1U;
      image.data.resize(1U);
    }
    image_->publish(image);

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp.sec = stamp;
    info.header.frame_id = "camera_optical_frame";
    info.width = 2U;
    info.height = 2U;
    info.distortion_model = "plumb_bob";
    info.d.assign(5U, 0.0);
    info.k = {{100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0}};
    if (scenario_ == InputScenario::InvalidCameraInfo) {
      info.k[0] = std::numeric_limits<double>::quiet_NaN();
    }
    camera_info_->publish(info);

    auto_aim_interfaces::msg::Vision vision;
    vision.header.stamp.sec = stamp;
    vision.yaw = 10.0F;
    vision.yaw_vel = 1.0F;
    vision.pitch = 5.0F;
    vision.pitch_vel = 1.0F;
    vision.roll = 0.0F;
    vision.quaternion = {{1.0F, 0.0F, 0.0F, 0.0F}};
    vision.shoot_speed = 20.0F;
    if (scenario_ == InputScenario::InvalidVision) {
      vision.yaw = 181.0F;
    }
    vision_->publish(vision);
  }

  bool subscribers_ready() const
  {
    return image_->get_subscription_count() > 0U &&
           camera_info_->get_subscription_count() > 0U &&
           vision_->get_subscription_count() > 0U;
  }

private:
  InputScenario scenario_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr vision_;
};

int run_with_publishers(
  InputScenario scenario, const std::string & output_path, bool verify_graph = false)
{
  static int domain_id = 187;
  const std::string domain = std::to_string(domain_id++);
  if (setenv("ROS_DOMAIN_ID", domain.c_str(), 1) != 0) {
    return -1;
  }
  const pid_t process = launch_process(direct_arguments(output_path, "1.2"));
  if (process <= 0) {
    return -1;
  }

  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto publisher = std::make_shared<ContractPublisher>(scenario);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);

  bool graph_checked = !verify_graph;
  std::int32_t stamp = 1;
  int status = 0;
  const auto publish_deadline = std::chrono::steady_clock::now() + 900ms;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
    waitpid(process, &status, WNOHANG) == 0)
  {
    if (publisher->subscribers_ready() &&
      std::chrono::steady_clock::now() < publish_deadline)
    {
      publisher->publish(stamp++);
    }
    executor.spin_some();
    if (verify_graph) {
      try {
        const auto subscriptions = publisher->get_node_graph_interface()->
          get_subscriber_names_and_types_by_node("ros_input_preflight", "/");
        if (subscriptions.size() > 3U) {
          ADD_FAILURE() << "preflight node exposed more than three subscriptions";
        } else if (subscriptions.size() == 3U) {
          EXPECT_EQ(subscriptions.count("/image_raw"), 1U);
          EXPECT_EQ(subscriptions.count("/camera_info"), 1U);
          EXPECT_EQ(subscriptions.count("/Vision_data"), 1U);
          EXPECT_TRUE(
            publisher->get_node_graph_interface()->get_publisher_names_and_types_by_node(
              "ros_input_preflight", "/").empty());
          EXPECT_TRUE(
            publisher->get_node_graph_interface()->get_service_names_and_types_by_node(
              "ros_input_preflight", "/").empty());
          EXPECT_TRUE(
            publisher->get_node_graph_interface()->get_client_names_and_types_by_node(
              "ros_input_preflight", "/").empty());
          graph_checked = true;
        }
      } catch (const rclcpp::exceptions::RCLError &) {
        // DDS discovery can briefly change while the child node is starting.
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  executor.remove_node(publisher);
  rclcpp::shutdown();

  EXPECT_TRUE(graph_checked);
  if (!WIFEXITED(status)) {
    kill(process, SIGKILL);
    waitpid(process, &status, 0);
    return -1;
  }
  return WEXITSTATUS(status);
}

TEST(ProcessContractTest, DirectExecutableFailIsValidJsonAndReturnsTwo)
{
  TemporaryFile output("direct_fail");
  const pid_t process = launch_process(direct_arguments(output.path(), "0.1"));
  ASSERT_GT(process, 0);
  EXPECT_EQ(wait_for_process(process), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "FAIL"));
  EXPECT_TRUE(
    json_has_failing_finding(
      output.path(), "topic.received",
      "No messages were received"));
}

TEST(ProcessContractTest, Ros2RunFailKeepsOutputFileAsValidJson)
{
  TemporaryFile output("ros2_run_fail");
  TemporaryFile wrapper_stdout("ros2_run_stdout");
  const pid_t process = launch_process(
    {"ros2", "run", "auto_aim_tools", "ros_input_preflight", "--duration", "0.1",
      "--format", "json", "--output", output.path()},
    wrapper_stdout.path());
  ASSERT_GT(process, 0);
  EXPECT_EQ(wait_for_process(process), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "FAIL"));
}

TEST(ProcessContractTest, SigintReturnsOneThirtyAndStillWritesJson)
{
  TemporaryFile output("sigint");
  const pid_t process = launch_process(direct_arguments(output.path(), "30"));
  ASSERT_GT(process, 0);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(kill(process, SIGINT), 0);
  EXPECT_EQ(wait_for_process(process), 130);
  EXPECT_TRUE(parses_as_json(output.path()));
}

TEST(ProcessContractTest, InvalidParameterReturnsTwoWithReadableDiagnostic)
{
  TemporaryFile error_output("invalid_parameter");
  const pid_t process = launch_process(
    {PREFLIGHT_EXECUTABLE_PATH, "--duration", "0"}, "", error_output.path());
  ASSERT_GT(process, 0);
  EXPECT_EQ(wait_for_process(process), 2);
  EXPECT_NE(
    read_file(error_output.path()).find("--duration has an invalid numeric value"),
    std::string::npos);
}

TEST(ProcessContractTest, RecorderResourceLimitsRejectInvalidPositiveIntegers)
{
  const std::array<std::string, 3> invalid_values = {
    "-1", "1trailing", std::to_string(std::numeric_limits<std::size_t>::max()) + "0"};
  const std::array<std::string, 2> options = {
    "--max-frames", "--max-buffered-image-bytes"};

  std::size_t case_index = 0U;
  for (const auto & option : options) {
    for (const auto & invalid_value : invalid_values) {
      const std::filesystem::path output =
        "/tmp/auto_aim_tools_recorder_invalid_limit_" + std::to_string(getpid()) + "_" +
        std::to_string(case_index++);
      TemporaryFile error_output("recorder_invalid_limit");
      std::vector<std::string> arguments = {
        CALIBRATION_DATASET_RECORDER_EXECUTABLE_PATH,
        "--config", "/does/not/need/to/exist.yaml",
        "--output", output.string(),
        "--max-frames", "1",
        "--max-buffered-image-bytes", "1",
        "--timeout-s", "0.1",
      };
      for (std::size_t index = 0U; index < arguments.size(); ++index) {
        if (arguments[index] == option) {
          arguments[index + 1U] = invalid_value;
          break;
        }
      }

      const pid_t process = launch_process(arguments, "", error_output.path());
      ASSERT_GT(process, 0);
      EXPECT_EQ(wait_for_process(process), 1) << option << '=' << invalid_value;
      EXPECT_NE(
        read_file(error_output.path()).find(
          option + " must be a positive decimal integer"),
        std::string::npos) << option << '=' << invalid_value;
      EXPECT_FALSE(std::filesystem::exists(output)) << output;
    }
  }
}

TEST(ProcessContractTest, SigtermWritesReportAndLeavesNoProcess)
{
  TemporaryFile output("sigterm");
  const pid_t process = launch_process(direct_arguments(output.path(), "30"));
  ASSERT_GT(process, 0);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(kill(process, SIGTERM), 0);
  EXPECT_EQ(wait_for_process(process), 143);
  EXPECT_TRUE(parses_as_json(output.path()));
  errno = 0;
  EXPECT_EQ(kill(process, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(ProcessContractTest, RepeatedStopIsIdempotentAndLeavesNoProcess)
{
  TemporaryFile output("repeated_stop");
  const pid_t process = launch_process(direct_arguments(output.path(), "30"));
  ASSERT_GT(process, 0);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(kill(process, SIGINT), 0);
  ASSERT_EQ(kill(process, SIGINT), 0);
  EXPECT_EQ(wait_for_process(process), 130);
  errno = 0;
  EXPECT_EQ(kill(process, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(ProcessContractTest, ForcedExitIsReapedAndLeavesNoProcess)
{
  TemporaryFile output("forced_exit");
  const pid_t process = launch_process(direct_arguments(output.path(), "30"));
  ASSERT_GT(process, 0);
  std::this_thread::sleep_for(300ms);
  ASSERT_EQ(kill(process, SIGKILL), 0);
  EXPECT_EQ(wait_for_process(process), 137);
  errno = 0;
  EXPECT_EQ(kill(process, 0), -1);
  EXPECT_EQ(errno, ESRCH);
}

TEST(ProcessContractTest, ValidPublishersReturnZeroAndGraphHasOnlyThreeSubscriptions)
{
  TemporaryFile output("pass");
  EXPECT_EQ(run_with_publishers(InputScenario::Valid, output.path(), true), 0)
    << read_file(output.path());
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "WARN"));
}

TEST(ProcessContractTest, InvalidImageFailsThroughExecutable)
{
  TemporaryFile output("invalid_image");
  EXPECT_EQ(run_with_publishers(InputScenario::InvalidImage, output.path()), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "FAIL"));
  EXPECT_TRUE(
    json_has_failing_finding(output.path(), "image.step", "width * 3"));
}

TEST(ProcessContractTest, InvalidCameraInfoFailsThroughExecutable)
{
  TemporaryFile output("invalid_camera_info");
  EXPECT_EQ(run_with_publishers(InputScenario::InvalidCameraInfo, output.path()), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "FAIL"));
  EXPECT_TRUE(
    json_has_failing_finding(output.path(), "camera_info.K", "must have 9 finite entries"));
}

TEST(ProcessContractTest, InvalidVisionFailsThroughExecutable)
{
  TemporaryFile output("invalid_vision");
  EXPECT_EQ(run_with_publishers(InputScenario::InvalidVision, output.path()), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
  EXPECT_TRUE(json_has_overall(output.path(), "FAIL"));
  EXPECT_TRUE(
    json_has_failing_finding(output.path(), "vision.yaw_range", "outside inclusive"));
}

}  // namespace
}  // namespace auto_aim_tools
