#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
  const std::vector<std::string> & arguments, const std::string & stdout_path = "")
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

std::vector<std::string> direct_arguments(
  const std::string & output, const std::string & duration)
{
  return {
    PREFLIGHT_EXECUTABLE_PATH, "--duration", duration, "--timeout", "0.4",
    "--vehicle-profile", "new_turtle", "--assume-shared-clock-domain",
    "--format", "json", "--output", output,
  };
}

class ContractPublisher final : public rclcpp::Node
{
public:
  ContractPublisher()
  : Node("preflight_process_contract_publisher")
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
    image.width = 2U;
    image.height = 2U;
    image.encoding = "bgr8";
    image.step = 6U;
    image.data.resize(12U);
    image_->publish(image);

    sensor_msgs::msg::CameraInfo info;
    info.header.stamp.sec = stamp;
    info.width = 2U;
    info.height = 2U;
    info.distortion_model = "plumb_bob";
    info.d.assign(5U, 0.0);
    info.k = {{100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0}};
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
    vision_->publish(vision);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Vision>::SharedPtr vision_;
};

TEST(ProcessContractTest, DirectExecutableFailIsValidJsonAndReturnsTwo)
{
  TemporaryFile output("direct_fail");
  const pid_t process = launch_process(direct_arguments(output.path(), "0.1"));
  ASSERT_GT(process, 0);
  EXPECT_EQ(wait_for_process(process), 2);
  EXPECT_TRUE(parses_as_json(output.path()));
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

TEST(ProcessContractTest, ValidPublishersReturnZeroAndGraphHasOnlyThreeSubscriptions)
{
  TemporaryFile output("pass");
  ASSERT_EQ(setenv("ROS_DOMAIN_ID", "187", 1), 0);
  const pid_t process = launch_process(direct_arguments(output.path(), "1.2"));
  ASSERT_GT(process, 0);

  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto publisher = std::make_shared<ContractPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(publisher);

  bool graph_checked = false;
  std::int32_t stamp = 1;
  int status = 0;
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline &&
    waitpid(process, &status, WNOHANG) == 0)
  {
    publisher->publish(stamp++);
    executor.spin_some();
    try {
      const auto subscriptions = publisher->get_node_graph_interface()->
        get_subscriber_names_and_types_by_node("ros_input_preflight", "/");
      if (!subscriptions.empty()) {
        EXPECT_EQ(subscriptions.size(), 3U);
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
    std::this_thread::sleep_for(20ms);
  }
  executor.remove_node(publisher);
  rclcpp::shutdown();

  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 0);
  EXPECT_TRUE(graph_checked);
  EXPECT_TRUE(parses_as_json(output.path()));
}

}  // namespace
}  // namespace auto_aim_tools
