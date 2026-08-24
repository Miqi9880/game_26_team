#include "auto_aim_ros2/pnp_stage.hpp"
#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace
{
struct Options
{
  std::string model_path;
  std::string model_profile_path;
  std::string video_path;
  std::string pnp_config_path;
  std::string csv_path;
  std::string annotated_dir;
  std::string device{"CPU"};
  int frames{5};
  bool allow_test_profile{false};
  bool allow_test_config{false};
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_pnp_smoke --model MODEL.xml --model-profile PROFILE.yaml "
    "--video VIDEO.avi --pnp-config CONFIG.yaml [options]\n"
    "Options:\n"
    "  --model-profile PATH  explicit versioned detector model profile YAML\n"
    "  --allow-test-profile  allow profile: test_only\n"
    "  --allow-test-config  explicitly allow a profile: test_only YAML\n"
    "  --frames N           frames to process (default 5)\n"
    "  --csv PATH           write RawArmorDetection and PnP rows as CSV\n"
    "  --annotated-dir PATH write PnP annotated BGR frames\n"
    "  --device DEVICE      OpenVINO device (default CPU)\n";
}

Options parse_options(int argc, char ** argv)
{
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    const auto require_value = [&](const char * name) -> std::string {
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
    } else if (argument == "--pnp-config") {
      result.pnp_config_path = require_value("--pnp-config");
    } else if (argument == "--allow-test-config") {
      result.allow_test_config = true;
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
  if (result.model_path.empty() || result.model_profile_path.empty() ||
    result.video_path.empty() || result.pnp_config_path.empty())
  {
    throw std::invalid_argument(
            "--model, --model-profile, --video, and --pnp-config are required");
  }
  if (result.frames <= 0) {
    throw std::invalid_argument("--frames must be positive");
  }
  return result;
}

void write_header(std::ofstream & csv)
{
  csv << "frame,stamp_ns,detection_index,class_id,color_id,confidence,pnp_valid,pnp_failure,"
         "bbox_x,bbox_y,bbox_w,bbox_h,p0x,p0y,p1x,p1y,p2x,p2y,p3x,p3y,"
         "camera_x_m,camera_y_m,camera_z_m,reprojection_error_px,gimbal_pose_available,"
         "gimbal_x_m,gimbal_y_m,gimbal_z_m,relative_yaw_rad,relative_pitch_rad\n";
}

void write_optional_number(std::ofstream & csv, const std::optional<double> & value)
{
  if (value.has_value()) {
    csv << std::setprecision(10) << *value;
  }
}

void write_row(
  std::ofstream & csv,
  int frame_index,
  std::int64_t stamp_ns,
  std::size_t detection_index,
  const rm_auto_aim::pnp::PoseObservation & observation)
{
  const auto & detection = observation.raw_detection;
  csv << frame_index << ',' << stamp_ns << ',' << detection_index << ',' << detection.class_id << ',' <<
    detection.color_id << ',' << std::setprecision(8) << detection.confidence << ',' <<
    (observation.valid ? 1 : 0) << ',' << rm_auto_aim::pnp::pose_failure_name(observation.failure) << ',' <<
    detection.bbox.x << ',' << detection.bbox.y << ',' << detection.bbox.width << ',' <<
    detection.bbox.height;
  for (const auto & point : detection.keypoints) {
    csv << ',' << point.x << ',' << point.y;
  }
  csv << ',';
  write_optional_number(
    csv, observation.valid ? std::optional<double>(observation.translation_in_camera_m[0]) : std::nullopt);
  csv << ',';
  write_optional_number(
    csv, observation.valid ? std::optional<double>(observation.translation_in_camera_m[1]) : std::nullopt);
  csv << ',';
  write_optional_number(
    csv, observation.valid ? std::optional<double>(observation.translation_in_camera_m[2]) : std::nullopt);
  csv << ',';
  write_optional_number(
    csv, std::isfinite(observation.reprojection_error_px) && observation.reprojection_error_px > 0.0 ?
    std::optional<double>(observation.reprojection_error_px) : std::nullopt);
  csv << ',' << (observation.translation_in_gimbal_m.has_value() ? 1 : 0) << ',';
  if (observation.translation_in_gimbal_m.has_value()) {
    csv << std::setprecision(10) << (*observation.translation_in_gimbal_m)[0] << ',' <<
      (*observation.translation_in_gimbal_m)[1] << ',' <<
      (*observation.translation_in_gimbal_m)[2];
  } else {
    csv << ",,";
  }
  csv << ',';
  if (observation.relative_angles_in_gimbal.has_value()) {
    csv << std::setprecision(10) << observation.relative_angles_in_gimbal->relative_yaw_rad << ',' <<
      observation.relative_angles_in_gimbal->relative_pitch_rad;
  } else {
    csv << ',';
  }
  csv << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    rm_auto_aim::pnp::ConfigLoadOptions load_options{};
    load_options.allow_test_only = options.allow_test_config;
    rm_auto_aim::pnp::PnpStage pnp_stage(
      rm_auto_aim::pnp::load_pnp_configuration(options.pnp_config_path, load_options));

    rm_auto_aim::detector::ModelProfileLoadOptions profile_options{};
    profile_options.allow_test_only = options.allow_test_profile;
    const auto model_profile = rm_auto_aim::detector::load_model_profile(
      options.model_profile_path, profile_options);
    auto detector_config = rm_auto_aim::detector::detector_config_from_model_profile(
      model_profile, options.model_path, options.device);
    rm_auto_aim::detector::OpenVinoYoloDetector detector(std::move(detector_config));

    std::cout << "dry_run=true allow_fire=false serial_enabled=false fire_command=0\n";
    std::cout << "pnp_profile=" << (pnp_stage.config().test_only ? "test_only" : "production") <<
      " model_profile=" << model_profile.model_id << "@" << model_profile.version <<
      " model_profile_kind=" << (model_profile.test_only ? "test_only" : "production") <<
      " effective_test_only=" << (pnp_stage.config().test_only || model_profile.test_only ? "true" : "false") <<
      " camera_frame=" << pnp_stage.config().camera.coordinate_frame <<
      " gimbal_extrinsic_configured=" <<
      (pnp_stage.config().camera_to_gimbal.configured ? "true" : "false") << '\n';
    if (pnp_stage.config().test_only) {
      std::cout << "warning=test_only calibration/geometry/extrinsic; no physical accuracy claim\n";
    }

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
      write_header(csv);
    }
    if (!options.annotated_dir.empty()) {
      std::filesystem::create_directories(options.annotated_dir);
    }

    std::cout << "frame,stamp_ns,detections,valid_pnp\n";
    cv::Mat image;
    int processed = 0;
    while (processed < options.frames && video.read(image)) {
      if (image.cols != pnp_stage.config().camera.image_width ||
        image.rows != pnp_stage.config().camera.image_height)
      {
        throw std::runtime_error(
                "video frame dimensions do not match PnP calibration: got " +
                std::to_string(image.cols) + "x" + std::to_string(image.rows) + ", expected " +
                std::to_string(pnp_stage.config().camera.image_width) + "x" +
                std::to_string(pnp_stage.config().camera.image_height));
      }

      rm_auto_aim::pipeline::ImageFrame frame{};
      frame.stamp_ns = fps > 0.0 ?
        static_cast<std::int64_t>(processed * 1'000'000'000.0 / fps) : 0;
      frame.width = static_cast<std::uint32_t>(image.cols);
      frame.height = static_cast<std::uint32_t>(image.rows);
      frame.encoding = "bgr8";
      frame.bgr_image = image.clone();
      const auto detections = detector.detect(frame);

      std::vector<rm_auto_aim::pnp::PoseObservation> observations;
      observations.reserve(detections.size());
      std::size_t valid_count = 0;
      for (const auto & detection : detections) {
        auto observation = pnp_stage.solve(detection, image.cols, image.rows);
        if (observation.valid) {
          ++valid_count;
        }
        observations.push_back(std::move(observation));
      }
      std::cout << processed << ',' << frame.stamp_ns << ',' << detections.size() << ',' << valid_count << '\n';

      for (std::size_t index = 0; index < observations.size(); ++index) {
        const auto & observation = observations[index];
        std::cout << "  detection=" << index << " class_id=" << observation.raw_detection.class_id <<
          " color_id=" << observation.raw_detection.color_id <<
          " confidence=" << observation.raw_detection.confidence <<
          " pnp_valid=" << (observation.valid ? "true" : "false") <<
          " failure=" << rm_auto_aim::pnp::pose_failure_name(observation.failure);
        if (observation.valid) {
          std::cout << " camera_xyz_m=" << observation.translation_in_camera_m[0] << ',' <<
            observation.translation_in_camera_m[1] << ',' << observation.translation_in_camera_m[2] <<
            " reprojection_error_px=" << observation.reprojection_error_px;
        }
        if (observation.translation_in_gimbal_m.has_value()) {
          std::cout << " gimbal_xyz_m=" << (*observation.translation_in_gimbal_m)[0] << ',' <<
            (*observation.translation_in_gimbal_m)[1] << ',' <<
            (*observation.translation_in_gimbal_m)[2];
        }
        if (observation.relative_angles_in_gimbal.has_value()) {
          std::cout << " relative_yaw_pitch_rad=" <<
            observation.relative_angles_in_gimbal->relative_yaw_rad << ',' <<
            observation.relative_angles_in_gimbal->relative_pitch_rad;
        }
        std::cout << '\n';
        if (csv) {
          write_row(csv, processed, frame.stamp_ns, index, observation);
        }
      }

      if (!options.annotated_dir.empty()) {
        const auto annotated = rm_auto_aim::pnp::PnpStage::annotate(image, observations);
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
    std::cerr << "auto_aim_pnp_smoke: " << error.what() << '\n';
    usage();
    return 1;
  }
}
