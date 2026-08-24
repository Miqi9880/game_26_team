#include "auto_aim_ros2/offline_pipeline.hpp"
#include "auto_aim_ros2/pnp_stage.hpp"
#include "auto_aim_ros2/raw_armor_detector.hpp"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

namespace
{
using rm_auto_aim::offline::AimerMode;

struct Options
{
  std::string model_path;
  std::string model_profile_path;
  std::string video_path;
  std::string pnp_config_path;
  std::string csv_path{"/tmp/game26_auto_aim_offline.csv"};
  std::string annotated_dir{"/tmp/game26_auto_aim_offline_annotated"};
  std::string device{"CPU"};
  int frames{10};
  bool allow_test_profile{false};
  bool allow_test_config{false};
  double fps_override{0.0};
  double frame_period_ms{0.0};
  AimerMode aimer_mode{AimerMode::RelativeDebug};
  std::optional<double> test_zero_yaw_degree;
  std::optional<double> test_zero_pitch_degree;
  rm_auto_aim::offline::TrackerConfig tracker_config{};
};

void usage()
{
  std::cerr <<
    "Usage: auto_aim_offline --model MODEL.xml --model-profile PROFILE.yaml "
    "--video VIDEO.avi --pnp-config CONFIG.yaml [options]\n"
    "Options:\n"
    "  --model-profile PATH    explicit versioned detector model profile YAML\n"
    "  --allow-test-profile    allow profile: test_only\n"
    "  --allow-test-config       explicitly allow profile: test_only YAML\n"
    "  --frames N                frames to process (default 10)\n"
    "  --fps FPS                 override video FPS when metadata is invalid\n"
    "  --frame-period-ms MS      explicit frame period; mutually exclusive with --fps\n"
    "  --csv PATH                output CSV (default /tmp/game26_auto_aim_offline.csv)\n"
    "  --annotated-dir PATH      output annotated PNG directory\n"
    "  --device DEVICE           OpenVINO device (default CPU)\n"
    "  --aimer-mode MODE         relative_debug or test_absolute_zero\n"
    "  --test-zero-yaw-degree D  test-only yaw zero for test_absolute_zero\n"
    "  --test-zero-pitch-degree D test-only pitch zero for test_absolute_zero\n"
    "  --min-detect-count N      tracker lock threshold (default 2)\n"
    "  --max-temp-lost-ms MS     tracker temporary-loss window (default 100)\n"
    "  --max-position-jump-m M   tracker association gate (default 0.75)\n"
    "  --max-angle-jump-rad R    tracker angle gate (default 0.75)\n";
}

double parse_double(const std::string & value, const char * name)
{
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
  return parsed;
}

int parse_int(const std::string & value, const char * name)
{
  std::size_t consumed = 0;
  const long parsed = std::stol(value, &consumed);
  if (consumed != value.size() || parsed < std::numeric_limits<int>::min() ||
    parsed > std::numeric_limits<int>::max())
  {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
  return static_cast<int>(parsed);
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
      result.frames = parse_int(require_value("--frames"), "--frames");
    } else if (argument == "--fps") {
      result.fps_override = parse_double(require_value("--fps"), "--fps");
    } else if (argument == "--frame-period-ms") {
      result.frame_period_ms = parse_double(require_value("--frame-period-ms"), "--frame-period-ms");
    } else if (argument == "--csv") {
      result.csv_path = require_value("--csv");
    } else if (argument == "--annotated-dir") {
      result.annotated_dir = require_value("--annotated-dir");
    } else if (argument == "--device") {
      result.device = require_value("--device");
    } else if (argument == "--aimer-mode") {
      const auto mode = require_value("--aimer-mode");
      if (mode == "relative_debug") {
        result.aimer_mode = AimerMode::RelativeDebug;
      } else if (mode == "test_absolute_zero") {
        result.aimer_mode = AimerMode::TestAbsoluteZero;
      } else {
        throw std::invalid_argument("--aimer-mode must be relative_debug or test_absolute_zero");
      }
    } else if (argument == "--test-zero-yaw-degree") {
      result.test_zero_yaw_degree = parse_double(
        require_value("--test-zero-yaw-degree"), "--test-zero-yaw-degree");
    } else if (argument == "--test-zero-pitch-degree") {
      result.test_zero_pitch_degree = parse_double(
        require_value("--test-zero-pitch-degree"), "--test-zero-pitch-degree");
    } else if (argument == "--min-detect-count") {
      result.tracker_config.min_detect_count = parse_int(
        require_value("--min-detect-count"), "--min-detect-count");
    } else if (argument == "--max-temp-lost-ms") {
      result.tracker_config.max_temp_lost_ms = parse_int(
        require_value("--max-temp-lost-ms"), "--max-temp-lost-ms");
    } else if (argument == "--max-position-jump-m") {
      result.tracker_config.max_position_jump_m = parse_double(
        require_value("--max-position-jump-m"), "--max-position-jump-m");
    } else if (argument == "--max-angle-jump-rad") {
      result.tracker_config.max_angle_jump_rad = parse_double(
        require_value("--max-angle-jump-rad"), "--max-angle-jump-rad");
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
  if (result.fps_override < 0.0 || result.frame_period_ms < 0.0) {
    throw std::invalid_argument("--fps and --frame-period-ms must not be negative");
  }
  if (result.fps_override > 0.0 && result.frame_period_ms > 0.0) {
    throw std::invalid_argument("--fps and --frame-period-ms are mutually exclusive");
  }
  if (result.test_zero_yaw_degree.has_value() != result.test_zero_pitch_degree.has_value()) {
    throw std::invalid_argument("test zero yaw and pitch must be provided together");
  }
  return result;
}

std::int64_t frame_period_ns(const Options & options, double video_fps)
{
  double period_ms = options.frame_period_ms;
  if (period_ms <= 0.0) {
    const double fps = options.fps_override > 0.0 ? options.fps_override : video_fps;
    if (!std::isfinite(fps) || fps <= 0.0) {
      throw std::runtime_error(
              "video FPS is invalid; provide --fps or --frame-period-ms for reproducible timestamps");
    }
    period_ms = 1000.0 / fps;
  }
  const auto period_ns = static_cast<std::int64_t>(std::llround(period_ms * 1'000'000.0));
  if (period_ns <= 0) {
    throw std::runtime_error("frame period is too small to represent in nanoseconds");
  }
  return period_ns;
}

void write_optional(std::ofstream & csv, const std::optional<double> & value)
{
  if (value.has_value()) {
    csv << std::setprecision(10) << *value;
  }
}

const rm_auto_aim::offline::TrackedTarget * find_track(
  const rm_auto_aim::offline::TrackerUpdate & update,
  std::size_t detection_index,
  std::int64_t stamp_ns)
{
  for (const auto & track : update.tracks) {
    if (track.observation.valid && track.observation.detection_index == detection_index &&
      track.observation.stamp_ns == stamp_ns)
    {
      return &track;
    }
  }
  return nullptr;
}

void write_header(std::ofstream & csv)
{
  csv <<
    "frame,stamp_ns,detection_count,valid_pnp_count,detection_index,class_id,armor_size,confidence,"
    "pnp_valid,pnp_failure,reprojection_error_px,camera_x_m,camera_y_m,camera_z_m,"
    "gimbal_x_m,gimbal_y_m,gimbal_z_m,relative_yaw_rad,relative_pitch_rad,track_id,"
    "tracking_state,consecutive_valid,selected,target_lock,absolute_command_valid,"
    "command_yaw_rad,command_pitch_rad,command_yaw_degree,command_pitch_degree,fire_command,test_only\n";
}

void write_row(
  std::ofstream & csv,
  int frame_index,
  std::int64_t stamp_ns,
  std::size_t detection_count,
  std::size_t valid_pnp_count,
  std::size_t detection_index,
  const rm_auto_aim::pnp::PoseObservation * pose,
  const rm_auto_aim::offline::TrackedTarget * track,
  bool selected,
  const rm_auto_aim::offline::AimerOutput & aimed)
{
  csv << frame_index << ',' << stamp_ns << ',' << detection_count << ',' << valid_pnp_count << ',';
  if (pose == nullptr) {
    // Empty fields 5..23: detection metadata, PnP evidence and track data.
    for (int index = 0; index < 19; ++index) {
      csv << ',';
    }
    csv << static_cast<int>(aimed.target_lock) << ",,,,,," <<
      static_cast<int>(aimed.fire_command) << ",1\n";
    return;
  }

  const auto & detection = pose->raw_detection;
  csv << detection_index << ',' << detection.class_id << ',' <<
    rm_auto_aim::pnp::armor_size_name(pose->armor_size) << ',' << std::setprecision(8) <<
    detection.confidence << ',' << (pose->valid ? 1 : 0) << ',' <<
    rm_auto_aim::pnp::pose_failure_name(pose->failure) << ',';
  if (pose->valid) {
    csv << std::setprecision(10) << pose->reprojection_error_px << ',' <<
      pose->translation_in_camera_m[0] << ',' << pose->translation_in_camera_m[1] << ',' <<
      pose->translation_in_camera_m[2] << ',';
  } else {
    csv << ",,,,";
  }
  if (pose->translation_in_gimbal_m.has_value()) {
    csv << (*pose->translation_in_gimbal_m)[0] << ',' << (*pose->translation_in_gimbal_m)[1] << ',' <<
      (*pose->translation_in_gimbal_m)[2] << ',';
  } else {
    csv << ",,,";
  }
  if (pose->relative_angles_in_gimbal.has_value()) {
    csv << pose->relative_angles_in_gimbal->relative_yaw_rad << ',' <<
      pose->relative_angles_in_gimbal->relative_pitch_rad << ',';
  } else {
    csv << ",,";
  }
  if (track != nullptr) {
    csv << track->track_id << ',' << rm_auto_aim::offline::tracking_state_name(track->state) << ',' <<
      track->consecutive_valid << ',' << (selected ? 1 : 0) << ',';
  } else {
    csv << ",lost,0,0,";
  }
  csv << static_cast<int>(aimed.target_lock) << ',' <<
    (aimed.absolute_command_valid ? 1 : 0) << ',';
  write_optional(csv, aimed.command_yaw_rad);
  csv << ',';
  write_optional(csv, aimed.command_pitch_rad);
  csv << ',';
  write_optional(csv, aimed.command_yaw_degree);
  csv << ',';
  write_optional(csv, aimed.command_pitch_degree);
  csv << ',' << static_cast<int>(aimed.fire_command) << ',' << (aimed.test_only ? 1 : 0) << '\n';
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
    rm_auto_aim::offline::OfflineTracker tracker(options.tracker_config);
    rm_auto_aim::offline::TargetSelector selector;

    rm_auto_aim::offline::AimerConfig aimer_config{};
    aimer_config.mode = options.aimer_mode;
    if (options.test_zero_yaw_degree.has_value()) {
      aimer_config.absolute_zero_configured = true;
      aimer_config.yaw_zero_rad = rm_auto_aim::units::degrees_to_radians(
        static_cast<float>(*options.test_zero_yaw_degree));
      aimer_config.pitch_zero_rad = rm_auto_aim::units::degrees_to_radians(
        static_cast<float>(*options.test_zero_pitch_degree));
    }
    rm_auto_aim::offline::SafeOfflineAimer aimer(aimer_config);

    cv::VideoCapture video(options.video_path);
    if (!video.isOpened()) {
      throw std::runtime_error("cannot open video: " + options.video_path);
    }
    const auto period_ns = frame_period_ns(options, video.get(cv::CAP_PROP_FPS));

    std::ofstream csv(options.csv_path, std::ios::out | std::ios::trunc);
    if (!csv) {
      throw std::runtime_error("cannot open CSV path: " + options.csv_path);
    }
    write_header(csv);
    std::filesystem::create_directories(options.annotated_dir);

    std::cout << "dry_run=true allow_fire=false serial_enabled=false fire_command=0\n";
    std::cout << "pipeline=detector->pnp->tracker->selector->aimer\n";
    std::cout << "pnp_profile=" << (pnp_stage.config().test_only ? "test_only" : "production") <<
      " model_profile=" << model_profile.model_id << "@" << model_profile.version <<
      " model_profile_kind=" << (model_profile.test_only ? "test_only" : "production") <<
      " effective_test_only=" << (pnp_stage.config().test_only || model_profile.test_only ? "true" : "false") <<
      " aimer_mode=" << rm_auto_aim::offline::aimer_mode_name(options.aimer_mode) <<
      " frame_period_ns=" << period_ns << '\n';
    if (pnp_stage.config().test_only) {
      std::cout << "warning=test_only calibration/geometry/extrinsic; no physical accuracy claim\n";
    }

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

      const auto stamp_ns = static_cast<std::int64_t>(processed) * period_ns;
      rm_auto_aim::pipeline::ImageFrame frame{};
      frame.stamp_ns = stamp_ns;
      frame.width = static_cast<std::uint32_t>(image.cols);
      frame.height = static_cast<std::uint32_t>(image.rows);
      frame.encoding = "bgr8";
      frame.bgr_image = image.clone();
      const auto detections = detector.detect(frame);

      std::vector<rm_auto_aim::pnp::PoseObservation> poses;
      std::vector<rm_auto_aim::offline::TargetObservation> target_observations;
      poses.reserve(detections.size());
      target_observations.reserve(detections.size());
      std::size_t valid_pnp_count = 0;
      for (std::size_t index = 0; index < detections.size(); ++index) {
        auto pose = pnp_stage.solve(detections[index], image.cols, image.rows);
        if (pose.valid) {
          ++valid_pnp_count;
        }
        target_observations.push_back(
          rm_auto_aim::offline::make_target_observation(pose, stamp_ns, index));
        poses.push_back(std::move(pose));
      }

      const auto update = tracker.update(target_observations, stamp_ns);
      const auto selected = selector.select(update.tracks, image.cols, image.rows);
      // This offline tool has no VisionData input, so shoot speed is
      // explicitly unavailable/zero. It is retained by Aimer for future
      // ballistic work and does not affect the safe output in this phase.
      const auto aimed = aimer.aim(selected, 0.0);

      std::cout << "frame=" << processed << " stamp_ns=" << stamp_ns <<
        " detections=" << detections.size() << " valid_pnp=" << valid_pnp_count <<
        " tracks=" << update.tracks.size() << " state=" <<
        rm_auto_aim::offline::tracking_state_name(update.state) << " target_lock=" <<
        static_cast<int>(aimed.target_lock) << " fire_command=" <<
        static_cast<int>(aimed.fire_command) << '\n';

      if (poses.empty()) {
        write_row(csv, processed, stamp_ns, 0, 0, 0, nullptr, nullptr, false, aimed);
      } else {
        for (std::size_t index = 0; index < poses.size(); ++index) {
          const auto * track = find_track(update, index, stamp_ns);
          const bool is_selected = selected.has_value() && track != nullptr &&
            selected->track_id == track->track_id;
          write_row(csv, processed, stamp_ns, poses.size(), valid_pnp_count, index,
            &poses[index], track, is_selected, aimed);
        }
      }

      const auto annotated = rm_auto_aim::offline::annotate_offline_frame(
        image, poses, update, selected, aimed);
      const auto output_path = std::filesystem::path(options.annotated_dir) /
        ("frame_" + std::to_string(processed) + ".png");
      if (annotated.empty() || !cv::imwrite(output_path.string(), annotated)) {
        throw std::runtime_error("failed to write annotated frame: " + output_path.string());
      }
      ++processed;
    }

    if (processed == 0) {
      throw std::runtime_error("video produced no readable frames");
    }
    std::cout << "processed_frames=" << processed << '\n';
    std::cout << "csv=" << options.csv_path << '\n';
    std::cout << "annotated_dir=" << options.annotated_dir << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "auto_aim_offline: " << error.what() << '\n';
    usage();
    return 1;
  }
}
