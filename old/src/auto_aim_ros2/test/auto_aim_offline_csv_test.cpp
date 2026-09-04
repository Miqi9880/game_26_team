// This test deliberately includes the CLI implementation so it exercises the
// exact private CSV writer rather than a copied schema.  Rename its program
// entry point so the gtest runner remains the only real main in this target.
#define main auto_aim_offline_embedded_main
#include "../src/auto_aim_offline.cpp"
#undef main

#include <algorithm>
#include <array>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

namespace
{

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> fields;
  std::string field;
  for (const char character : line) {
    if (character == ',') {
      fields.push_back(field);
      field.clear();
    } else {
      field.push_back(character);
    }
  }
  fields.push_back(field);
  return fields;
}

std::unordered_map<std::string, std::size_t> field_indexes(
  const std::vector<std::string> & header)
{
  std::unordered_map<std::string, std::size_t> indexes;
  for (std::size_t index = 0; index < header.size(); ++index) {
    indexes.emplace(header[index], index);
  }
  return indexes;
}

TEST(AutoAimOfflineCsv, AllPredictionAndBallisticPresenceCombinationsMatchTheHeader)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("auto_aim_offline_csv_test_" + std::to_string(std::rand()) + ".csv");
  {
    std::ofstream csv(path);
    ASSERT_TRUE(csv.is_open());
    write_header(csv);

    rm_auto_aim::offline::AimerOutput aimed{};
    rm_auto_aim::offline::PredictionResult prediction{};
    prediction.valid = false;
    prediction.failure_reason = rm_auto_aim::offline::PredictionFailureReason::NoTarget;
    rm_auto_aim::offline::BallisticResult ballistic{};
    ballistic.enabled = true;
    ballistic.valid = false;
    ballistic.failure_reason = rm_auto_aim::offline::BallisticFailureReason::NoTarget;

    write_row(csv, 0, 1, 0, 0, 0, nullptr, nullptr, false, aimed, nullptr, nullptr);
    write_row(csv, 1, 2, 0, 0, 0, nullptr, nullptr, false, aimed, &prediction, nullptr);
    write_row(csv, 2, 3, 0, 0, 0, nullptr, nullptr, false, aimed, nullptr, &ballistic);
    write_row(csv, 3, 4, 0, 0, 0, nullptr, nullptr, false, aimed, &prediction, &ballistic);
  }

  std::ifstream csv(path);
  ASSERT_TRUE(csv.is_open());
  std::string line;
  ASSERT_TRUE(std::getline(csv, line));
  const auto header = split_csv_line(line);
  const auto indexes = field_indexes(header);
  ASSERT_EQ(header.size(), 73U);
  const std::array<std::string, 31> ballistic_suffix{
    "ballistic_enabled", "ballistic_valid", "ballistic_reason", "ballistic_track_id",
    "ballistic_source_stamp_ns", "ballistic_target_x_m", "ballistic_target_y_m",
    "ballistic_target_z_m", "ballistic_horizontal_distance_m",
    "ballistic_geometric_yaw_rad", "ballistic_geometric_pitch_rad", "ballistic_yaw_rad",
    "ballistic_pitch_rad", "ballistic_gravity_pitch_correction_rad",
    "ballistic_flight_time_s", "ballistic_flight_time_ns", "ballistic_system_latency_ns",
    "ballistic_recommended_prediction_horizon_ns", "ballistic_bullet_speed_mps",
    "ballistic_gravity_mps2", "ballistic_origin_assumption", "ballistic_test_only",
    "ballistic_production_ready", "ballistic_control_applied", "serial_enabled", "dry_run",
    "allow_fire", "yaw_vel_rad_s", "pitch_vel_rad_s", "yaw_acc_rad_s2", "pitch_acc_rad_s2"};
  EXPECT_TRUE(std::equal(ballistic_suffix.begin(), ballistic_suffix.end(), header.end() - 31));

  for (int row = 0; row < 4; ++row) {
    ASSERT_TRUE(std::getline(csv, line));
    const auto fields = split_csv_line(line);
    ASSERT_EQ(fields.size(), header.size()) << "row=" << row;
    EXPECT_EQ(fields[indexes.at("ballistic_test_only")], "1");
    EXPECT_EQ(fields[indexes.at("ballistic_production_ready")], "0");
    EXPECT_EQ(fields[indexes.at("ballistic_control_applied")], "0");
    EXPECT_EQ(fields[indexes.at("serial_enabled")], "0");
    EXPECT_EQ(fields[indexes.at("dry_run")], "1");
    EXPECT_EQ(fields[indexes.at("allow_fire")], "0");
    EXPECT_EQ(fields[indexes.at("yaw_vel_rad_s")], "0");
    EXPECT_EQ(fields[indexes.at("pitch_vel_rad_s")], "0");
    EXPECT_EQ(fields[indexes.at("yaw_acc_rad_s2")], "0");
    EXPECT_EQ(fields[indexes.at("pitch_acc_rad_s2")], "0");
  }

  std::error_code error;
  std::filesystem::remove(path, error);
  EXPECT_FALSE(error);
}

}  // namespace
