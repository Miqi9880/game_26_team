#include "auto_aim_ros2/auto_aim_core.hpp"
#include "auto_aim_ros2/offline_scenario_benchmark.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

namespace
{
using rm_auto_aim::offline::OfflineScenarioBenchmarkConfig;
using rm_auto_aim::offline::OfflineScenarioBenchmarkRecord;
using rm_auto_aim::offline::OfflineScenarioBenchmarkResult;

OfflineScenarioBenchmarkResult run_benchmark(
  const std::string & scenario, std::uint64_t seed, bool diagnostics_enabled = true)
{
  OfflineScenarioBenchmarkConfig config{};
  config.scenario = scenario;
  config.seed = seed;
  config.diagnostics_enabled = diagnostics_enabled;
  return rm_auto_aim::offline::run_offline_scenario_benchmark(config);
}

const OfflineScenarioBenchmarkRecord * find_event(
  const OfflineScenarioBenchmarkResult & result, const std::string & event)
{
  const auto found = std::find_if(
    result.records.begin(), result.records.end(), [&](const OfflineScenarioBenchmarkRecord & record) {
      return record.event == event;
    });
  return found == result.records.end() ? nullptr : &*found;
}

void expect_finite(const std::optional<double> & value)
{
  if (value.has_value()) {
    EXPECT_TRUE(std::isfinite(*value));
  }
}

void expect_truth_muzzle(
  const OfflineScenarioBenchmarkRecord & record, double x, double y, double z)
{
  ASSERT_TRUE(record.truth_valid);
  ASSERT_TRUE(record.truth_muzzle_m.has_value());
  EXPECT_NEAR((*record.truth_muzzle_m)[0], x, 1e-12);
  EXPECT_NEAR((*record.truth_muzzle_m)[1], y, 1e-12);
  EXPECT_NEAR((*record.truth_muzzle_m)[2], z, 1e-12);
}

void expect_safe_record(const OfflineScenarioBenchmarkRecord & record)
{
  SCOPED_TRACE(record.scenario + ":" + record.event);
  EXPECT_EQ(
    record.schema_version,
    rm_auto_aim::offline::kOfflineScenarioBenchmarkSchemaVersion);
  EXPECT_TRUE(record.synthetic);
  EXPECT_TRUE(record.test_only);
  EXPECT_FALSE(record.production_ready);
  EXPECT_TRUE(
    record.origin_assumption == "synthetic_muzzle_frame" ||
    record.origin_assumption == "omitted_for_intentional_failure");
  EXPECT_EQ(record.safe_command_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_TRUE(
    record.diagnostic_target_lock == rm_auto_aim::pipeline::kTargetLocked ||
    record.diagnostic_target_lock == rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(record.fire_command, rm_auto_aim::pipeline::kFireNone);
  EXPECT_FLOAT_EQ(record.yaw_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(record.pitch_vel_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(record.yaw_acc_rad_s2, 0.0F);
  EXPECT_FLOAT_EQ(record.pitch_acc_rad_s2, 0.0F);
  EXPECT_FALSE(record.serial_enabled);
  EXPECT_TRUE(record.dry_run);
  EXPECT_FALSE(record.allow_fire);

  if (record.truth_muzzle_m.has_value()) {
    for (std::size_t index = 0; index < 3U; ++index) {
      EXPECT_TRUE(std::isfinite((*record.truth_muzzle_m)[index]));
    }
  }
  expect_finite(record.truth_relative_yaw_rad);
  expect_finite(record.truth_relative_pitch_rad);
  expect_finite(record.predictor_yaw_error_rad);
  expect_finite(record.predictor_pitch_error_rad);
  expect_finite(record.ballistic_flight_time_s);
}

void expect_tracker_selector_and_safe_command_equal(
  const OfflineScenarioBenchmarkRecord & baseline,
  const OfflineScenarioBenchmarkRecord & diagnostic)
{
  SCOPED_TRACE(baseline.scenario + ":" + baseline.event);
  EXPECT_EQ(baseline.scenario, diagnostic.scenario);
  EXPECT_EQ(baseline.seed, diagnostic.seed);
  EXPECT_EQ(baseline.frame_index, diagnostic.frame_index);
  EXPECT_EQ(baseline.stamp_ns, diagnostic.stamp_ns);
  EXPECT_EQ(baseline.event, diagnostic.event);
  EXPECT_EQ(baseline.input_order, diagnostic.input_order);
  EXPECT_EQ(baseline.tracker_accepted, diagnostic.tracker_accepted);
  EXPECT_EQ(baseline.tracker_rejected, diagnostic.tracker_rejected);
  EXPECT_EQ(baseline.tracker_association_result, diagnostic.tracker_association_result);
  EXPECT_EQ(baseline.tracker_association_reason, diagnostic.tracker_association_reason);
  EXPECT_EQ(baseline.primary_track_id, diagnostic.primary_track_id);
  EXPECT_EQ(baseline.primary_tracking_state, diagnostic.primary_tracking_state);
  EXPECT_EQ(baseline.primary_association_result, diagnostic.primary_association_result);
  EXPECT_EQ(baseline.primary_association_reason, diagnostic.primary_association_reason);
  EXPECT_EQ(baseline.selected_track_id, diagnostic.selected_track_id);
  EXPECT_EQ(baseline.selector_switched, diagnostic.selector_switched);
  EXPECT_EQ(baseline.selector_switch_reason, diagnostic.selector_switch_reason);
  EXPECT_EQ(baseline.diagnostic_target_lock, diagnostic.diagnostic_target_lock);
  EXPECT_EQ(baseline.safe_command_target_lock, diagnostic.safe_command_target_lock);
  EXPECT_EQ(baseline.fire_command, diagnostic.fire_command);
  EXPECT_FLOAT_EQ(baseline.yaw_vel_rad_s, diagnostic.yaw_vel_rad_s);
  EXPECT_FLOAT_EQ(baseline.pitch_vel_rad_s, diagnostic.pitch_vel_rad_s);
  EXPECT_FLOAT_EQ(baseline.yaw_acc_rad_s2, diagnostic.yaw_acc_rad_s2);
  EXPECT_FLOAT_EQ(baseline.pitch_acc_rad_s2, diagnostic.pitch_acc_rad_s2);
  EXPECT_EQ(baseline.serial_enabled, diagnostic.serial_enabled);
  EXPECT_EQ(baseline.dry_run, diagnostic.dry_run);
  EXPECT_EQ(baseline.allow_fire, diagnostic.allow_fire);
}

std::filesystem::path make_temporary_directory()
{
  static std::atomic<std::uint64_t> counter{0};
  const auto parent = std::filesystem::temp_directory_path();
  for (std::size_t attempt = 0; attempt < 1000U; ++attempt) {
    const auto name = "offline_scenario_benchmark_test_" +
      std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
    const auto candidate = parent / name;
    std::error_code error;
    if (std::filesystem::create_directory(candidate, error)) {
      return candidate;
    }
    if (error && error != std::errc::file_exists) {
      throw std::runtime_error("unable to create scenario benchmark test directory");
    }
  }
  throw std::runtime_error("unable to allocate unique scenario benchmark test directory");
}

class ScopedTemporaryDirectory final
{
public:
  ScopedTemporaryDirectory() : path_(make_temporary_directory()) {}

  ~ScopedTemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path_, error);
  }

  const std::filesystem::path & path() const noexcept
  {
    return path_;
  }

private:
  std::filesystem::path path_;
};

std::string read_file(const std::filesystem::path & path)
{
  std::ifstream input(path, std::ios::binary);
  return {
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>()};
}

}  // namespace

TEST(OfflineScenarioBenchmark, ScenarioNamesAreExplicitAndUnknownRequestsFailClosed)
{
  const std::vector<std::string> expected{
    "all",
    "static_3m",
    "spin_3m",
    "spin_translate_3m",
    "crossing_permuted",
    "occlusion_reacquisition",
    "invalid_inputs",
    "ballistic_failures",
  };
  EXPECT_EQ(rm_auto_aim::offline::offline_scenario_benchmark_scenarios(), expected);
  for (const auto & scenario : expected) {
    EXPECT_TRUE(rm_auto_aim::offline::is_offline_scenario_benchmark_scenario(scenario));
  }
  EXPECT_FALSE(rm_auto_aim::offline::is_offline_scenario_benchmark_scenario(""));
  EXPECT_FALSE(rm_auto_aim::offline::is_offline_scenario_benchmark_scenario("not_a_scenario"));

  EXPECT_THROW(run_benchmark("", 7), std::invalid_argument);
  EXPECT_THROW(run_benchmark("not_a_scenario", 7), std::invalid_argument);
}

TEST(OfflineScenarioBenchmark, AllFixturesAreSyntheticSafeFiniteAndTestOnly)
{
  const auto result = run_benchmark("all", 0, true);
  EXPECT_EQ(
    result.schema_version,
    rm_auto_aim::offline::kOfflineScenarioBenchmarkSchemaVersion);
  EXPECT_EQ(result.requested_scenario, "all");
  EXPECT_EQ(result.seed, 0U);
  EXPECT_TRUE(result.diagnostics_enabled);
  ASSERT_EQ(result.records.size(), 50U);

  std::size_t diagnostic_locks = 0;
  for (const auto & record : result.records) {
    expect_safe_record(record);
    EXPECT_TRUE(record.predictor_enabled);
    EXPECT_TRUE(record.ballistic_enabled);
    if (record.diagnostic_target_lock == rm_auto_aim::pipeline::kTargetLocked) {
      ++diagnostic_locks;
    }
  }
  EXPECT_GT(diagnostic_locks, 0U);
}

TEST(OfflineScenarioBenchmark, StaticThreeMetreFixtureHasIndependentBallisticGolden)
{
  const auto result = run_benchmark("static_3m", 17, true);
  ASSERT_EQ(result.records.size(), 5U);

  const auto & initial = result.records.front();
  EXPECT_EQ(initial.event, "initial_detection");
  EXPECT_EQ(initial.primary_tracking_state, "detecting");
  EXPECT_FALSE(initial.selected_track_id.has_value());
  EXPECT_FALSE(initial.predictor_valid);
  EXPECT_EQ(initial.predictor_reason, "no_target");
  EXPECT_FALSE(initial.ballistic_valid);
  EXPECT_EQ(initial.ballistic_reason, "no_target");

  const auto & tracking = result.records[1];
  EXPECT_EQ(tracking.event, "stationary_3m_observation");
  EXPECT_EQ(tracking.primary_tracking_state, "tracking");
  ASSERT_TRUE(tracking.selected_track_id.has_value());
  EXPECT_EQ(tracking.diagnostic_target_lock, rm_auto_aim::pipeline::kTargetLocked);
  expect_truth_muzzle(tracking, 3.0, 0.0, 0.0);
  ASSERT_TRUE(tracking.predictor_valid);
  EXPECT_EQ(tracking.predictor_reason, "none");
  ASSERT_TRUE(tracking.predictor_future_stamp_ns.has_value());
  EXPECT_EQ(*tracking.predictor_future_stamp_ns, 20'000'000);
  ASSERT_TRUE(tracking.predictor_yaw_error_rad.has_value());
  ASSERT_TRUE(tracking.predictor_pitch_error_rad.has_value());
  EXPECT_NEAR(*tracking.predictor_yaw_error_rad, 0.0, 1e-15);
  EXPECT_NEAR(*tracking.predictor_pitch_error_rad, 0.0, 1e-15);
  EXPECT_EQ(tracking.predictor_error_reason, "synthetic_future_angle_error");

  ASSERT_TRUE(tracking.ballistic_valid);
  EXPECT_EQ(tracking.ballistic_reason, "none");
  ASSERT_TRUE(tracking.ballistic_flight_time_s.has_value());
  ASSERT_TRUE(tracking.ballistic_flight_time_ns.has_value());
  ASSERT_TRUE(tracking.ballistic_recommended_horizon_ns.has_value());
  EXPECT_NEAR(*tracking.ballistic_flight_time_s, 0.1501017401625011, 1e-14);
  EXPECT_EQ(*tracking.ballistic_flight_time_ns, 150'101'740);
  EXPECT_EQ(*tracking.ballistic_recommended_horizon_ns, 160'101'740);
}

TEST(OfflineScenarioBenchmark, RotationAndRotationTranslationFixturesUseThreeMetreSyntheticTruth)
{
  const auto spin = run_benchmark("spin_3m", 3, true);
  const auto spin_translate = run_benchmark("spin_translate_3m", 3, true);
  ASSERT_EQ(spin.records.size(), 7U);
  ASSERT_EQ(spin_translate.records.size(), 7U);

  const auto & spin_first_tracking = spin.records[1];
  const auto & spin_second_tracking = spin.records[2];
  EXPECT_EQ(spin_first_tracking.event, "synthetic_in_place_rotation");
  EXPECT_EQ(spin_first_tracking.primary_tracking_state, "tracking");
  expect_truth_muzzle(spin_first_tracking, 3.0, 0.0, 0.0);
  ASSERT_TRUE(spin_first_tracking.truth_relative_yaw_rad.has_value());
  ASSERT_TRUE(spin_second_tracking.truth_relative_yaw_rad.has_value());
  EXPECT_NE(
    *spin_first_tracking.truth_relative_yaw_rad,
    *spin_second_tracking.truth_relative_yaw_rad);
  ASSERT_TRUE(spin_first_tracking.predictor_valid);
  EXPECT_EQ(spin_first_tracking.predictor_error_reason, "synthetic_future_angle_error");
  ASSERT_TRUE(spin_first_tracking.predictor_yaw_error_rad.has_value());
  ASSERT_TRUE(spin_first_tracking.predictor_pitch_error_rad.has_value());
  EXPECT_NEAR(*spin_first_tracking.predictor_yaw_error_rad, 0.0, 1e-12);
  EXPECT_NEAR(*spin_first_tracking.predictor_pitch_error_rad, 0.0, 1e-12);

  const auto & translate_first_tracking = spin_translate.records[1];
  const auto & translate_last = spin_translate.records.back();
  EXPECT_EQ(translate_first_tracking.event, "synthetic_rotation_and_translation");
  EXPECT_EQ(translate_first_tracking.primary_tracking_state, "tracking");
  expect_truth_muzzle(translate_first_tracking, 3.025, -0.085, 0.024);
  expect_truth_muzzle(translate_last, 3.15, 0.09, 0.044);
  ASSERT_TRUE(translate_first_tracking.predictor_valid);
  ASSERT_TRUE(translate_first_tracking.predictor_yaw_error_rad.has_value());
  ASSERT_TRUE(translate_first_tracking.predictor_pitch_error_rad.has_value());
  EXPECT_NEAR(*translate_first_tracking.predictor_yaw_error_rad, 0.0, 1e-12);
  EXPECT_NEAR(*translate_first_tracking.predictor_pitch_error_rad, 0.0, 1e-12);
}

TEST(OfflineScenarioBenchmark, CrossingFixtureIsOrderPermutedButSelectionStaysDeterministic)
{
  const auto zero_seed = run_benchmark("crossing_permuted", 0, true);
  const auto one_seed = run_benchmark("crossing_permuted", 1, true);
  ASSERT_EQ(zero_seed.records.size(), 5U);
  ASSERT_EQ(one_seed.records.size(), 5U);

  EXPECT_EQ(zero_seed.records[0].input_order, "cross_a>cross_b");
  EXPECT_EQ(zero_seed.records[1].input_order, "cross_b>cross_a");
  EXPECT_EQ(one_seed.records[2].input_order, "cross_b>cross_a");
  for (std::size_t index = 1; index < zero_seed.records.size(); ++index) {
    const auto & first = zero_seed.records[index];
    const auto & second = one_seed.records[index];
    EXPECT_EQ(first.primary_tracking_state, "tracking");
    EXPECT_EQ(second.primary_tracking_state, "tracking");
    ASSERT_TRUE(first.selected_track_id.has_value());
    ASSERT_TRUE(second.selected_track_id.has_value());
    EXPECT_EQ(first.primary_track_id, second.primary_track_id);
    EXPECT_EQ(first.selected_track_id, second.selected_track_id);
    EXPECT_TRUE(first.truth_valid);
    EXPECT_TRUE(second.truth_valid);
    const auto expected_truth_id = index < 3U ? "cross_a" : "cross_b";
    EXPECT_EQ(first.truth_id, expected_truth_id);
    EXPECT_EQ(second.truth_id, expected_truth_id);
    EXPECT_EQ(first.safe_command_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
    EXPECT_EQ(second.safe_command_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  }
  EXPECT_TRUE(zero_seed.records[3].selector_switched);
  EXPECT_EQ(zero_seed.records[3].selector_switch_reason, "ranked_replacement");
}

TEST(OfflineScenarioBenchmark, OcclusionExpiresAndReacquiresWithoutAStaleLock)
{
  const auto result = run_benchmark("occlusion_reacquisition", 19, true);
  ASSERT_EQ(result.records.size(), 9U);

  const auto * initial = find_event(result, "initial_detection");
  const auto * temporary_loss = find_event(result, "short_occlusion_temp_lost");
  const auto * reacquired = find_event(result, "short_occlusion_reacquired");
  const auto * after_reacquisition = find_event(result, "tracking_after_reacquisition");
  const auto * expired = find_event(result, "temporary_loss_timeout_expired");
  const auto * new_capture = find_event(result, "new_capture_after_timeout");
  ASSERT_NE(initial, nullptr);
  ASSERT_NE(temporary_loss, nullptr);
  ASSERT_NE(reacquired, nullptr);
  ASSERT_NE(after_reacquisition, nullptr);
  ASSERT_NE(expired, nullptr);
  ASSERT_NE(new_capture, nullptr);
  ASSERT_TRUE(initial->primary_track_id.has_value());
  ASSERT_TRUE(new_capture->primary_track_id.has_value());

  EXPECT_EQ(temporary_loss->primary_tracking_state, "temp_lost");
  EXPECT_FALSE(temporary_loss->selected_track_id.has_value());
  EXPECT_EQ(temporary_loss->diagnostic_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
  EXPECT_EQ(reacquired->primary_tracking_state, "detecting");
  EXPECT_EQ(reacquired->primary_association_result, "reacquired");
  EXPECT_FALSE(reacquired->selected_track_id.has_value());
  EXPECT_EQ(after_reacquisition->primary_tracking_state, "tracking");
  ASSERT_TRUE(after_reacquisition->selected_track_id.has_value());
  EXPECT_EQ(expired->primary_tracking_state, "lost");
  EXPECT_EQ(expired->primary_association_result, "expired");
  EXPECT_EQ(new_capture->primary_tracking_state, "detecting");
  EXPECT_NE(*initial->primary_track_id, *new_capture->primary_track_id);
}

TEST(OfflineScenarioBenchmark, InvalidInputsFailClosedWithoutNonFiniteEvidence)
{
  const auto result = run_benchmark("invalid_inputs", 23, true);
  ASSERT_EQ(result.records.size(), 10U);

  for (const auto & event : {
      "timestamp_negative",
      "timestamp_rollback",
      "timestamp_duplicate",
      "nan_camera_position",
      "infinite_relative_yaw",
      "invalid_observation_flag"})
  {
    const auto * record = find_event(result, event);
    ASSERT_NE(record, nullptr);
    EXPECT_TRUE(record->tracker_rejected);
    EXPECT_FALSE(record->selected_track_id.has_value());
    EXPECT_EQ(record->safe_command_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
    EXPECT_EQ(record->fire_command, rm_auto_aim::pipeline::kFireNone);
    EXPECT_FALSE(record->truth_valid);
  }

  EXPECT_EQ(
    find_event(result, "timestamp_negative")->tracker_association_result,
    "rejected_timestamp");
  EXPECT_EQ(
    find_event(result, "timestamp_rollback")->tracker_association_result,
    "rejected_timestamp");
  EXPECT_EQ(
    find_event(result, "timestamp_duplicate")->tracker_association_result,
    "rejected_timestamp");
  EXPECT_EQ(
    find_event(result, "nan_camera_position")->tracker_association_result,
    "rejected_invalid");
  EXPECT_EQ(
    find_event(result, "infinite_relative_yaw")->tracker_association_result,
    "rejected_invalid");
  EXPECT_EQ(
    find_event(result, "invalid_observation_flag")->tracker_association_result,
    "rejected_invalid");

  const auto * recovered = find_event(result, "recovery_tracking");
  ASSERT_NE(recovered, nullptr);
  EXPECT_EQ(recovered->primary_tracking_state, "tracking");
  ASSERT_TRUE(recovered->selected_track_id.has_value());

  const auto csv = rm_auto_aim::offline::render_offline_scenario_benchmark_csv(result);
  EXPECT_EQ(csv.find(",nan,"), std::string::npos);
  EXPECT_EQ(csv.find(",NaN,"), std::string::npos);
  EXPECT_EQ(csv.find(",inf,"), std::string::npos);
  EXPECT_EQ(csv.find(",Inf,"), std::string::npos);
}

TEST(OfflineScenarioBenchmark, BallisticFailuresHaveExplicitReasonsAndNoControlEffect)
{
  const auto result = run_benchmark("ballistic_failures", 29, true);
  ASSERT_EQ(result.records.size(), 7U);

  const auto * nominal = find_event(result, "nominal_ballistic_diagnostic");
  ASSERT_NE(nominal, nullptr);
  ASSERT_TRUE(nominal->ballistic_valid);
  EXPECT_EQ(nominal->ballistic_reason, "none");
  ASSERT_TRUE(nominal->ballistic_flight_time_s.has_value());
  EXPECT_NEAR(*nominal->ballistic_flight_time_s, 0.1501017401625011, 1e-14);

  const std::vector<std::pair<std::string, std::string>> expected_failures{
    {"ballistic_missing_muzzle_transform", "missing_muzzle_transform"},
    {"ballistic_unreachable", "discriminant_negative"},
    {"ballistic_missing_bullet_speed", "missing_bullet_speed"},
    {"ballistic_invalid_bullet_speed", "invalid_bullet_speed"},
    {"ballistic_horizon_exceeds_maximum", "horizon_exceeds_prediction_maximum"},
  };
  for (const auto & expected : expected_failures) {
    const auto * record = find_event(result, expected.first);
    ASSERT_NE(record, nullptr);
    EXPECT_FALSE(record->ballistic_valid);
    EXPECT_EQ(record->ballistic_reason, expected.second);
    if (expected.first == "ballistic_missing_muzzle_transform") {
      EXPECT_EQ(record->origin_assumption, "omitted_for_intentional_failure");
    }
    EXPECT_EQ(record->safe_command_target_lock, rm_auto_aim::pipeline::kTargetUnlocked);
    EXPECT_EQ(record->fire_command, rm_auto_aim::pipeline::kFireNone);
  }
}

TEST(OfflineScenarioBenchmark, DisabledDiagnosticsCannotChangeTrackerSelectorOrSafeCommand)
{
  const auto baseline = run_benchmark("all", 31, false);
  const auto diagnostic = run_benchmark("all", 31, true);
  ASSERT_FALSE(baseline.diagnostics_enabled);
  ASSERT_TRUE(diagnostic.diagnostics_enabled);
  ASSERT_EQ(baseline.records.size(), diagnostic.records.size());

  for (std::size_t index = 0; index < baseline.records.size(); ++index) {
    const auto & baseline_record = baseline.records[index];
    const auto & diagnostic_record = diagnostic.records[index];
    expect_tracker_selector_and_safe_command_equal(baseline_record, diagnostic_record);
    EXPECT_FALSE(baseline_record.predictor_enabled);
    EXPECT_FALSE(baseline_record.predictor_valid);
    EXPECT_EQ(baseline_record.predictor_reason, "disabled_by_benchmark_config");
    EXPECT_EQ(baseline_record.predictor_error_reason, "diagnostics_disabled");
    EXPECT_FALSE(baseline_record.ballistic_enabled);
    EXPECT_FALSE(baseline_record.ballistic_valid);
    EXPECT_EQ(baseline_record.ballistic_reason, "disabled_by_benchmark_config");
    EXPECT_TRUE(diagnostic_record.predictor_enabled);
    EXPECT_TRUE(diagnostic_record.ballistic_enabled);
  }
}

TEST(OfflineScenarioBenchmark, RenderersAndNewOutputDirectoriesAreByteDeterministic)
{
  const auto first = run_benchmark("crossing_permuted", 0, true);
  const auto second = run_benchmark("crossing_permuted", 0, true);
  const auto first_csv = rm_auto_aim::offline::render_offline_scenario_benchmark_csv(first);
  const auto first_json = rm_auto_aim::offline::render_offline_scenario_benchmark_json(first);
  const auto first_markdown = rm_auto_aim::offline::render_offline_scenario_benchmark_markdown(first);
  EXPECT_EQ(first_csv, rm_auto_aim::offline::render_offline_scenario_benchmark_csv(second));
  EXPECT_EQ(first_json, rm_auto_aim::offline::render_offline_scenario_benchmark_json(second));
  EXPECT_EQ(first_markdown, rm_auto_aim::offline::render_offline_scenario_benchmark_markdown(second));
  EXPECT_NE(first_json.find("\"hardware_validation\": false"), std::string::npos);
  EXPECT_NE(first_json.find("\"serial_enabled\": false"), std::string::npos);
  EXPECT_NE(first_markdown.find("software-only synthetic benchmark"), std::string::npos);

  ScopedTemporaryDirectory temporary;
  const auto first_output = temporary.path() / "first";
  const auto second_output = temporary.path() / "second";
  rm_auto_aim::offline::write_offline_scenario_benchmark_outputs(first, first_output);
  rm_auto_aim::offline::write_offline_scenario_benchmark_outputs(second, second_output);
  for (const auto & artifact : {"benchmark.csv", "summary.json", "summary.md"}) {
    const auto first_path = first_output / artifact;
    const auto second_path = second_output / artifact;
    ASSERT_TRUE(std::filesystem::is_regular_file(first_path));
    ASSERT_TRUE(std::filesystem::is_regular_file(second_path));
    EXPECT_EQ(read_file(first_path), read_file(second_path));
  }
  EXPECT_EQ(read_file(first_output / "benchmark.csv"), first_csv);
  EXPECT_EQ(read_file(first_output / "summary.json"), first_json);
  EXPECT_EQ(read_file(first_output / "summary.md"), first_markdown);
}

TEST(OfflineScenarioBenchmark, ExistingOutputDirectoryIsRejectedBeforeSentinelMutation)
{
  const auto result = run_benchmark("static_3m", 37, true);
  ScopedTemporaryDirectory temporary;
  const auto existing_output = temporary.path() / "existing";
  ASSERT_TRUE(std::filesystem::create_directory(existing_output));
  const auto sentinel = existing_output / "sentinel.txt";
  constexpr char kSentinel[] = "preserve this existing output directory";
  {
    std::ofstream output(sentinel, std::ios::binary);
    ASSERT_TRUE(output.is_open());
    output << kSentinel;
    ASSERT_TRUE(output.good());
  }

  EXPECT_THROW(
    rm_auto_aim::offline::write_offline_scenario_benchmark_outputs(result, existing_output),
    std::invalid_argument);
  EXPECT_EQ(read_file(sentinel), kSentinel);
  EXPECT_FALSE(std::filesystem::exists(existing_output / "benchmark.csv"));
  EXPECT_FALSE(std::filesystem::exists(existing_output / "summary.json"));
  EXPECT_FALSE(std::filesystem::exists(existing_output / "summary.md"));
}
