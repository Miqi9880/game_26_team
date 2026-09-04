#ifndef AUTO_AIM_ROS2__OFFLINE_SCENARIO_BENCHMARK_HPP_
#define AUTO_AIM_ROS2__OFFLINE_SCENARIO_BENCHMARK_HPP_

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace rm_auto_aim::offline
{

// This schema is intentionally independent of auto_aim_offline CSV and of
// the evidence-report/bundle parsers.  It describes software-only synthetic
// inputs, not camera, hardware, or competition evidence.
constexpr std::uint32_t kOfflineScenarioBenchmarkSchemaVersion = 1;

struct OfflineScenarioBenchmarkConfig
{
  // Accepted names are returned by offline_scenario_benchmark_scenarios().
  // "all" runs every canonical scenario in its documented order.
  std::string scenario;
  std::uint64_t seed{0};
  // Tests may turn diagnostics off to prove that Tracker/Selector/Aimer
  // behavior has no dependency on Predictor or BallisticDiagnostic.
  bool diagnostics_enabled{false};
};

struct OfflineScenarioBenchmarkRecord
{
  std::string scenario;
  std::uint32_t schema_version{kOfflineScenarioBenchmarkSchemaVersion};
  std::uint64_t seed{0};
  std::size_t frame_index{0};
  std::int64_t stamp_ns{-1};
  std::string event;
  std::string input_order;

  bool synthetic{true};
  bool test_only{true};
  bool production_ready{false};

  // Synthetic truth is deliberately separate from TargetObservation's
  // camera-frame association coordinate.  It is an explicit test-only muzzle
  // point in x-forward/y-left/z-up metres, never an inferred external pose.
  bool truth_valid{false};
  std::string truth_id;
  std::optional<cv::Vec3d> truth_muzzle_m;
  std::optional<double> truth_relative_yaw_rad;
  std::optional<double> truth_relative_pitch_rad;

  bool tracker_accepted{false};
  bool tracker_rejected{false};
  std::string tracker_association_result;
  std::string tracker_association_reason;
  std::optional<std::uint64_t> primary_track_id;
  std::string primary_tracking_state;
  std::string primary_association_result;
  std::string primary_association_reason;
  std::optional<std::uint64_t> selected_track_id;
  bool selector_switched{false};
  std::string selector_switch_reason;

  bool predictor_enabled{false};
  bool predictor_valid{false};
  std::string predictor_reason;
  std::optional<std::int64_t> predictor_future_stamp_ns;
  std::optional<double> predictor_yaw_error_rad;
  std::optional<double> predictor_pitch_error_rad;
  std::string predictor_error_reason;

  bool ballistic_enabled{false};
  bool ballistic_valid{false};
  std::string ballistic_reason;
  std::optional<double> ballistic_flight_time_s;
  std::optional<std::int64_t> ballistic_flight_time_ns;
  std::optional<std::int64_t> ballistic_recommended_horizon_ns;
  // This is a benchmark-output label rather than a new production frame enum.
  std::string origin_assumption{"synthetic_muzzle_frame"};

  // Values below are always copied from SafeOfflineAimer::safe_command(),
  // never from a diagnostic command candidate or a Tracker velocity.
  std::int8_t diagnostic_target_lock{0};
  std::int8_t safe_command_target_lock{0};
  std::int8_t fire_command{0};
  float yaw_vel_rad_s{0.0F};
  float pitch_vel_rad_s{0.0F};
  float yaw_acc_rad_s2{0.0F};
  float pitch_acc_rad_s2{0.0F};
  bool serial_enabled{false};
  bool dry_run{true};
  bool allow_fire{false};
};

struct OfflineScenarioBenchmarkResult
{
  std::uint32_t schema_version{kOfflineScenarioBenchmarkSchemaVersion};
  std::string requested_scenario;
  std::uint64_t seed{0};
  bool diagnostics_enabled{true};
  std::vector<OfflineScenarioBenchmarkRecord> records;
};

// Canonical names only; callers should not infer scenario availability from
// the CLI help text or from filenames.
std::vector<std::string> offline_scenario_benchmark_scenarios();
bool is_offline_scenario_benchmark_scenario(const std::string & scenario) noexcept;

// Runs only deterministic, directly-constructed TargetObservation fixtures.
// It opens no video/model/ROS/serial/hardware resource and never sends an
// AimCommand anywhere.
OfflineScenarioBenchmarkResult run_offline_scenario_benchmark(
  const OfflineScenarioBenchmarkConfig & config);

// Stable, locale-independent artifact renderers.  They intentionally do not
// include wall-clock time, host paths, or hardware metadata, so two runs with
// the same scenario and seed have byte-identical content.
std::string render_offline_scenario_benchmark_csv(
  const OfflineScenarioBenchmarkResult & result);
std::string render_offline_scenario_benchmark_json(
  const OfflineScenarioBenchmarkResult & result);
std::string render_offline_scenario_benchmark_markdown(
  const OfflineScenarioBenchmarkResult & result);

// Creates exactly one new output directory with benchmark.csv, summary.json,
// and summary.md.  An existing path, link, non-directory parent, or unsafe
// parent component fails closed before benchmark artifacts are overwritten.
void write_offline_scenario_benchmark_outputs(
  const OfflineScenarioBenchmarkResult & result,
  const std::filesystem::path & output_dir);

}  // namespace rm_auto_aim::offline

#endif  // AUTO_AIM_ROS2__OFFLINE_SCENARIO_BENCHMARK_HPP_
