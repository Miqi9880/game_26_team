#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>

namespace
{
struct Options
{
  std::string model_path;
  std::string model_profile_path;
  std::string video_path;
  std::string csv_path;
  std::string annotated_dir;
  std::string device{"CPU"};
  int frames{5};
  bool allow_test_profile{false};
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_detector_smoke --model MODEL.xml --video VIDEO.avi [options]\n"
    "Options:\n"
    "  --frames N             frames to process (default 5)\n"
    "  --model-profile PATH   explicit versioned model profile YAML\n"
    "  --allow-test-profile   allow profile: test_only\n"
    "  --csv PATH             write raw detections as CSV\n"
    "  --annotated-dir PATH  write annotated BGR frames\n"
    "  --device DEVICE       OpenVINO device (default CPU)\n";
}

Options parse_options(int argc, char ** argv)
{
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    auto require_value = [&](const char * name) -> std::string {
      if (index + 1 >= argc) {
        throw std::invalid_argument(std::string(name) + " requires a value");
      }
      return argv[++index];
    };
    if (argument == "--model") {
      result.model_path = require_value("--model");
    } else if (argument == "--model-profile") {
      result.model_profile_path = require_value("--model-profile");
    } else if (argument == "--allow-test-profile") {
      result.allow_test_profile = true;
    } else if (argument == "--video") {
      result.video_path = require_value("--video");
    } else if (argument == "--frames") {
      result.frames = std::stoi(require_value("--frames"));
    } else if (argument == "--csv") {
      result.csv_path = require_value("--csv");
    } else if (argument == "--annotated-dir") {
      result.annotated_dir = require_value("--annotated-dir");
    } else if (argument == "--device") {
      result.device = require_value("--device");
    } else if (argument == "--help" || argument == "-h") {
      usage();
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + argument);
    }
  }
  if (result.model_path.empty() || result.video_path.empty()) {
    throw std::invalid_argument("--model and --video are required");
  }
  if (result.frames <= 0) {
    throw std::invalid_argument("--frames must be positive");
  }
  return result;
}

void write_csv_header(std::ofstream & csv)
{
  csv << "frame,stamp_ns,detection_index,class_id,color_id,confidence,bbox_x,bbox_y,bbox_w,bbox_h"
         ",p0x,p0y,p1x,p1y,p2x,p2y,p3x,p3y\n";
}

void write_csv_row(
  std::ofstream & csv,
  int frame_index,
  std::int64_t stamp_ns,
  std::size_t detection_index,
  const rm_auto_aim::detector::RawArmorDetection & detection)
{
  csv << frame_index << ',' << stamp_ns << ',' << detection_index << ',' << detection.class_id << ','
      << detection.color_id << ',' << std::setprecision(8) << detection.confidence << ','
      << detection.bbox.x << ',' << detection.bbox.y << ',' << detection.bbox.width << ','
      << detection.bbox.height;
  for (const auto & point : detection.keypoints) {
    csv << ',' << point.x << ',' << point.y;
  }
  csv << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    rm_auto_aim::detector::DetectorConfig config{};
    std::string profile_label = "unprofiled_legacy_reference";
    if (!options.model_profile_path.empty()) {
      rm_auto_aim::detector::ModelProfileLoadOptions profile_options{};
      profile_options.allow_test_only = options.allow_test_profile;
      const auto profile = rm_auto_aim::detector::load_model_profile(
        options.model_profile_path, profile_options);
      profile_label = profile.model_id + "@" + profile.version;
      config = rm_auto_aim::detector::detector_config_from_model_profile(
        profile, options.model_path, options.device);
    } else {
      config.model_path = options.model_path;
      config.device = options.device;
    }

    rm_auto_aim::detector::OpenVinoYoloDetector detector(config);
    const auto & info = detector.model_info();
    std::cout << "dry_run=true allow_fire=false serial_enabled=false fire_command=0\n";
    std::cout << "model=" << options.model_path << " profile=" << profile_label <<
      " device=" << options.device << '\n';
    std::cout << "input_shape=";
    for (const auto value : info.input_shape) {
      std::cout << value << ' ';
    }
    std::cout << "type=" << info.input_element_type << " layout=" << info.input_layout <<
      " output_shape=";
    for (const auto value : info.output_shape) {
      std::cout << value << ' ';
    }
    std::cout << "type=" << info.output_element_type << " layout=" << info.output_layout << '\n';

    cv::VideoCapture video(options.video_path);
    if (!video.isOpened()) {
      throw std::runtime_error("cannot open video: " + options.video_path);
    }
    double fps = video.get(cv::CAP_PROP_FPS);
    if (!std::isfinite(fps) || fps <= 0.0) {
      fps = 0.0;
    }

    std::ofstream csv;
    if (!options.csv_path.empty()) {
      csv.open(options.csv_path, std::ios::out | std::ios::trunc);
      if (!csv) {
        throw std::runtime_error("cannot open CSV path: " + options.csv_path);
      }
      write_csv_header(csv);
    }
    if (!options.annotated_dir.empty()) {
      std::filesystem::create_directories(options.annotated_dir);
    }

    std::cout << "frame,stamp_ns,detections\n";
    cv::Mat image;
    int processed = 0;
    while (processed < options.frames && video.read(image)) {
      rm_auto_aim::pipeline::ImageFrame frame{};
      frame.stamp_ns = fps > 0.0 ? static_cast<std::int64_t>(processed * 1'000'000'000.0 / fps) : 0;
      frame.width = static_cast<std::uint32_t>(image.cols);
      frame.height = static_cast<std::uint32_t>(image.rows);
      frame.encoding = "bgr8";
      frame.bgr_image = image.clone();

      const auto detections = detector.detect(frame);
      std::cout << processed << ',' << frame.stamp_ns << ',' << detections.size() << '\n';
      for (std::size_t index = 0; index < detections.size(); ++index) {
        if (csv) {
          write_csv_row(csv, processed, frame.stamp_ns, index, detections[index]);
        }
      }
      if (!options.annotated_dir.empty()) {
        const auto annotated = rm_auto_aim::detector::OpenVinoYoloDetector::annotate(
          image, detections);
        const auto output_path = std::filesystem::path(options.annotated_dir) /
          ("frame_" + std::to_string(processed) + ".png");
        if (annotated.empty() || !cv::imwrite(output_path.string(), annotated)) {
          throw std::runtime_error("failed to write annotated frame: " + output_path.string());
        }
      }
      ++processed;
    }
    if (processed == 0) {
      throw std::runtime_error("video produced no readable frames");
    }
    std::cout << "processed_frames=" << processed << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "auto_aim_detector_smoke: " << error.what() << '\n';
    usage();
    return 1;
  }
}
