#include "auto_aim_ros2/auto_aim_core.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>

namespace
{
struct Options
{
  int frames{30};
  bool mock_target{false};
  std::string csv_path{""};
};

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--mock-target") {
      options.mock_target = true;
    } else if (arg == "--frames" && i + 1 < argc) {
      options.frames = std::atoi(argv[++i]);
    } else if (arg == "--csv" && i + 1 < argc) {
      options.csv_path = argv[++i];
    } else {
      std::cerr << "Usage: auto_aim_dry_run [--frames N] [--mock-target] [--csv PATH]\n";
      std::exit(2);
    }
  }
  if (options.frames <= 0) {
    std::cerr << "--frames must be positive\n";
    std::exit(2);
  }
  return options;
}
}  // namespace

int main(int argc, char ** argv)
{
  const auto options = parse_options(argc, argv);
  rm_auto_aim::pipeline::CoreConfig config{};  // dry-run always disables fire
  std::optional<rm_auto_aim::pipeline::Detection> detection;
  if (options.mock_target) {
    detection = rm_auto_aim::pipeline::Detection{};
    detection->valid = true;
    detection->yaw_rad = 0.15F;
    detection->pitch_rad = -0.05F;
  }

  std::unique_ptr<rm_auto_aim::pipeline::YoloStage> yolo;
  if (options.mock_target) {
    yolo = std::make_unique<rm_auto_aim::pipeline::MockYoloStage>(detection);
  } else {
    yolo = std::make_unique<rm_auto_aim::pipeline::NullYoloStage>();
  }

  rm_auto_aim::pipeline::AutoAimPipeline pipeline(
    std::move(yolo),
    std::make_unique<rm_auto_aim::pipeline::PassThroughArmorStage>(),
    std::make_unique<rm_auto_aim::pipeline::LatestTargetTracker>(),
    std::make_unique<rm_auto_aim::pipeline::FirstTargetStage>(),
    std::make_unique<rm_auto_aim::pipeline::CommandAimer>(), config);

  std::ofstream csv;
  if (!options.csv_path.empty()) {
    csv.open(options.csv_path, std::ios::out | std::ios::trunc);
    if (!csv) {
      std::cerr << "Cannot open CSV path: " << options.csv_path << '\n';
      return 1;
    }
  }
  const auto start = std::chrono::steady_clock::now();
  if (csv) {
    csv << "frame,target_lock,yaw_rad_internal,pitch_rad_internal,fire_command\n";
  }
  std::cout << "frame,target_lock,yaw_rad_internal,pitch_rad_internal,fire_command\n";
  for (int frame_index = 0; frame_index < options.frames; ++frame_index) {
    rm_auto_aim::pipeline::ImageFrame frame{};
    frame.stamp_ns = frame_index * 10'000'000LL;
    frame.width = 640;
    frame.height = 480;
    frame.encoding = "rgb8";
    const auto command = pipeline.process(frame, start + std::chrono::milliseconds(frame_index * 10));
    const auto fire = static_cast<int>(rm_auto_aim::pipeline::kFireNone);  // hard dry-run inhibit
    std::cout << frame_index << ',' << static_cast<int>(command.target_lock) << ','
              << command.yaw_rad << ',' << command.pitch_rad << ',' << fire << '\n';
    if (csv) {
      csv << frame_index << ',' << static_cast<int>(command.target_lock) << ','
          << command.yaw_rad << ',' << command.pitch_rad << ',' << fire << '\n';
    }
  }
  return 0;
}
