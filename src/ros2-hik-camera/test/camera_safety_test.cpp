#include "hik_camera/camera_safety.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace hik_camera
{
namespace
{

CameraInfoContractSample validCameraInfo()
{
  CameraInfoContractSample sample;
  sample.width = 1440U;
  sample.height = 1080U;
  sample.distortion_model = "plumb_bob";
  sample.k = {{
    1000.0, 0.0, 720.0,
    0.0, 1000.0, 540.0,
    0.0, 0.0, 1.0,
  }};
  sample.d.assign(5U, 0.0);
  return sample;
}

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
  EXPECT_EQ(buffer_size, std::size_t{1920U} *1080U * 3U);
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

TEST(CameraInfoContract, AcceptsFiniteCalibratedSample)
{
  const auto result = validateCameraInfoContract(validCameraInfo());
  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(CameraInfoContract, RejectsInvalidDimensionsAndMatrix)
{
  auto dimensions = validCameraInfo();
  dimensions.width = 0U;
  EXPECT_FALSE(validateCameraInfoContract(dimensions).valid);

  auto focal_length = validCameraInfo();
  focal_length.k[0] = 0.0;
  EXPECT_FALSE(validateCameraInfoContract(focal_length).valid);

  auto non_finite = validCameraInfo();
  non_finite.k[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateCameraInfoContract(non_finite).valid);
}

TEST(CameraInfoContract, RejectsInvalidDistortion)
{
  auto non_finite = validCameraInfo();
  non_finite.d[0] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateCameraInfoContract(non_finite).valid);

  auto wrong_length = validCameraInfo();
  wrong_length.d.resize(4U);
  EXPECT_FALSE(validateCameraInfoContract(wrong_length).valid);

  auto unsupported = validCameraInfo();
  unsupported.distortion_model = "unreviewed_model";
  EXPECT_FALSE(validateCameraInfoContract(unsupported).valid);
}

TEST(CameraInfoContract, RequiresFrameDimensionsToMatch)
{
  const auto sample = validCameraInfo();
  EXPECT_TRUE(cameraInfoMatchesFrame(sample, 1440U, 1080U));
  EXPECT_FALSE(cameraInfoMatchesFrame(sample, 1280U, 1024U));
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

TEST(BoundedByteString, StopsAtNullTerminator)
{
  const std::array<unsigned char, 8U> bytes{
    {'S', 'E', 'R', 'I', 'A', 'L', '\0', 'X'}};

  EXPECT_EQ(boundedByteString(bytes.data(), bytes.size()), "SERIAL");
}

TEST(BoundedByteString, AcceptsFullNonTerminatedBuffer)
{
  const std::array<unsigned char, 4U> bytes{{'A', 'B', 'C', 'D'}};

  EXPECT_EQ(boundedByteString(bytes.data(), bytes.size()), "ABCD");
}

TEST(BoundedByteString, HonorsSmallerCapacity)
{
  const std::array<unsigned char, 6U> bytes{{'S', 'E', 'R', 'I', 'A', 'L'}};

  EXPECT_EQ(boundedByteString(bytes.data(), 3U), "SER");
}

TEST(BoundedByteString, RejectsMissingStorage)
{
  const std::array<unsigned char, 1U> bytes{{'X'}};

  EXPECT_TRUE(boundedByteString(nullptr, bytes.size()).empty());
  EXPECT_TRUE(boundedByteString(bytes.data(), 0U).empty());
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
