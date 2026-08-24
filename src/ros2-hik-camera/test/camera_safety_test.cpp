#include "hik_camera/camera_safety.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace hik_camera
{
namespace
{

TEST(CameraSelection, RejectsEmptyDeviceList)
{
  const auto result = selectCameraDevice({}, "");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "no camera was found");
}

TEST(CameraSelection, SelectsOnlyCameraWithoutRequestedSerial)
{
  const auto result = selectCameraDevice({"TEST_SERIAL_A"}, "");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.index, 0U);
}

TEST(CameraSelection, RejectsMultipleCamerasWithoutRequestedSerial)
{
  const auto result =
    selectCameraDevice({"TEST_SERIAL_A", "TEST_SERIAL_B"}, "");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(
    result.reason,
    "multiple cameras were found; camera_serial is required");
}

TEST(CameraSelection, SelectsMatchingSerial)
{
  const auto result =
    selectCameraDevice({"TEST_SERIAL_A", "TEST_SERIAL_B"}, "TEST_SERIAL_B");

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.index, 1U);
}

TEST(CameraSelection, RejectsMissingSerial)
{
  const auto result =
    selectCameraDevice({"TEST_SERIAL_A"}, "TEST_SERIAL_B");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "the requested camera serial was not found");
}

TEST(CameraSelection, RejectsDuplicateSerial)
{
  const auto result =
    selectCameraDevice({"TEST_SERIAL_A", "TEST_SERIAL_A"}, "TEST_SERIAL_A");

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "the requested camera serial is not unique");
}

TEST(Rgb8BufferSize, CalculatesExpectedSize)
{
  std::size_t buffer_size = 0U;

  ASSERT_TRUE(calculateRgb8BufferSize(1920U, 1080U, buffer_size));
  EXPECT_EQ(buffer_size, std::size_t{1920U} * 1080U * 3U);
}

TEST(Rgb8BufferSize, RejectsZeroDimensions)
{
  std::size_t buffer_size = 123U;

  EXPECT_FALSE(calculateRgb8BufferSize(0U, 1080U, buffer_size));
  EXPECT_EQ(buffer_size, 0U);
  EXPECT_FALSE(calculateRgb8BufferSize(1920U, 0U, buffer_size));
  EXPECT_EQ(buffer_size, 0U);
}

TEST(Rgb8BufferSize, RejectsOverflow)
{
  std::size_t buffer_size = 0U;

  EXPECT_FALSE(
    calculateRgb8BufferSize(
      std::numeric_limits<std::uint32_t>::max(),
      std::numeric_limits<std::uint32_t>::max(),
      buffer_size));
  EXPECT_EQ(buffer_size, 0U);
}

TEST(ParameterRange, AcceptsFiniteInclusiveValues)
{
  EXPECT_TRUE(isFiniteInRange(1.0, 1.0, 2.0));
  EXPECT_TRUE(isFiniteInRange(1.5, 1.0, 2.0));
  EXPECT_TRUE(isFiniteInRange(2.0, 1.0, 2.0));
}

TEST(ParameterRange, RejectsInvalidValuesAndRanges)
{
  EXPECT_FALSE(isFiniteInRange(0.5, 1.0, 2.0));
  EXPECT_FALSE(isFiniteInRange(2.5, 1.0, 2.0));
  EXPECT_FALSE(
    isFiniteInRange(
      std::numeric_limits<double>::quiet_NaN(), 1.0, 2.0));
  EXPECT_FALSE(
    isFiniteInRange(
      std::numeric_limits<double>::infinity(), 1.0, 2.0));
  EXPECT_FALSE(isFiniteInRange(1.5, 2.0, 1.0));
}

TEST(SdkStatus, FormatsFixedWidthHexadecimal)
{
  EXPECT_EQ(formatSdkStatus(0), "0x00000000");
  EXPECT_EQ(formatSdkStatus(0x1234), "0x00001234");
  EXPECT_EQ(
    formatSdkStatus(std::numeric_limits<int>::min()),
    "0x80000000");
}

}  // namespace
}  // namespace hik_camera