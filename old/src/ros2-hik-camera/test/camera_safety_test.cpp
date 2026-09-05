#include "hik_camera/camera_safety.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <unistd.h>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>

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

std::string readTextFile(const std::string & path)
{
  std::ifstream input(path);
  return std::string(
    std::istreambuf_iterator<char>(input),
    std::istreambuf_iterator<char>());
}

class TemporaryCalibrationFile
{
public:
  TemporaryCalibrationFile()
  {
    char path[] = "/tmp/hik_camera_calibration_XXXXXX";
    const int descriptor = mkstemp(path);
    if (descriptor >= 0) {
      close(descriptor);
      path_ = path;
      std::ofstream output(path_);
      output << "test calibration fixture\n";
    }
  }

  ~TemporaryCalibrationFile()
  {
    if (!hardlink_.empty()) {
      std::remove(hardlink_.c_str());
    }
    if (!alias_.empty()) {
      std::remove(alias_.c_str());
    }
    if (!path_.empty()) {
      std::remove(path_.c_str());
    }
  }

  const std::string & path() const
  {
    return path_;
  }

  std::string fileUrl() const
  {
    return "file://" + path_;
  }

  std::string symlinkUrl()
  {
    alias_ = path_ + ".alias";
    if (symlink(path_.c_str(), alias_.c_str()) != 0) {
      alias_.clear();
    }
    return "file://" + alias_;
  }

  std::string hardlinkUrl()
  {
    hardlink_ = path_ + ".hardlink";
    if (link(path_.c_str(), hardlink_.c_str()) != 0) {
      hardlink_.clear();
    }
    return "file://" + hardlink_;
  }

  std::string dotDotUrl() const
  {
    constexpr const char * temporary_prefix = "/tmp/";
    if (path_.compare(0U, 5U, temporary_prefix) != 0) {
      return {};
    }
    return "file:///tmp/../tmp/" + path_.substr(5U);
  }

private:
  std::string path_;
  std::string alias_;
  std::string hardlink_;
};

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

TEST(CameraInfoUrlProvenance, RejectsResolvedPackageExample)
{
  TemporaryCalibrationFile candidate;
  ASSERT_FALSE(candidate.path().empty());

  const auto result = validateCameraInfoUrlProvenance(
    "package://hik_camera/config/camera_info.yaml",
    candidate.fileUrl(), {candidate.fileUrl()});

  EXPECT_FALSE(result.valid);
  EXPECT_NE(result.reason.find("checked-in unverified"), std::string::npos);
}

TEST(CameraInfoUrlProvenance, RejectsFileAndSymlinkAliases)
{
  TemporaryCalibrationFile candidate;
  ASSERT_FALSE(candidate.path().empty());
  const auto alias_url = candidate.symlinkUrl();
  const auto hardlink_url = candidate.hardlinkUrl();
  const auto dotdot_url = candidate.dotDotUrl();
  ASSERT_NE(alias_url, "file://");
  ASSERT_NE(hardlink_url, "file://");
  ASSERT_FALSE(dotdot_url.empty());

  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      candidate.fileUrl(), candidate.fileUrl(), {candidate.fileUrl()}).valid);
  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      alias_url, alias_url, {candidate.fileUrl()}).valid);
  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      hardlink_url, hardlink_url, {candidate.fileUrl()}).valid);
  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      dotdot_url, dotdot_url, {candidate.fileUrl()}).valid);
}

TEST(CameraInfoUrlProvenance, AcceptsDifferentExternalFile)
{
  TemporaryCalibrationFile candidate;
  TemporaryCalibrationFile verified;
  ASSERT_FALSE(candidate.path().empty());
  ASSERT_FALSE(verified.path().empty());

  const auto result = validateCameraInfoUrlProvenance(
    verified.fileUrl(), verified.fileUrl(), {candidate.fileUrl()});

  EXPECT_TRUE(result.valid) << result.reason;
}

TEST(CameraInfoUrlProvenance, CheckedInFileCannotPassFormalContract)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  auto node = std::make_shared<rclcpp::Node>("unverified_camera_info_file_test");
  camera_info_manager::CameraInfoManager manager(node.get(), "unverified_example");
  const std::string package_url =
    "package://hik_camera/config/camera_info.yaml";
  const auto installed_example_url = manager.resolveURL(
    package_url, "unverified_example");
  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      package_url, installed_example_url, {installed_example_url}).valid);
  EXPECT_FALSE(
    validateCameraInfoUrlProvenance(
      installed_example_url, installed_example_url,
      {installed_example_url}).valid);

  const std::string url =
    std::string("file://") + HIK_CAMERA_UNVERIFIED_EXAMPLE_PATH;
  const bool example_exists = std::ifstream(HIK_CAMERA_UNVERIFIED_EXAMPLE_PATH).good();
  EXPECT_TRUE(example_exists);
  const bool loaded = example_exists && manager.loadCameraInfo(url);
  EXPECT_TRUE(loaded);
  if (!loaded) {
    node.reset();
    rclcpp::shutdown();
    return;
  }
  const auto message = manager.getCameraInfo();
  const auto validation = validateCameraInfoContract(
    CameraInfoContractSample{
        message.width, message.height, message.distortion_model,
        message.k, message.d,
      });
  EXPECT_FALSE(validation.valid);
  node.reset();
  rclcpp::shutdown();
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

TEST(CameraParameterContract, DefaultConfigDoesNotSetManualWhiteBalanceRatios)
{
  const auto config = readTextFile(HIK_CAMERA_DEFAULT_PARAMS_PATH);
  ASSERT_FALSE(config.empty());

  const std::string manual_ratio_prefix = "balance_" "ratio_";
  EXPECT_EQ(config.find(manual_ratio_prefix), std::string::npos);
}

TEST(CameraParameterContract, GenericFloatPolicyDoesNotMapWhiteBalanceRatios)
{
  const auto node_source = readTextFile(HIK_CAMERA_NODE_SOURCE_PATH);
  ASSERT_FALSE(node_source.empty());

  const std::string ros_parameter_prefix = "balance_" "ratio_";
  EXPECT_EQ(node_source.find(ros_parameter_prefix), std::string::npos);

  for (const char channel : {'R', 'G', 'B'}) {
    const std::string sdk_field = "Balance" "Ratio_" + std::string(1U, channel);
    EXPECT_EQ(node_source.find(sdk_field), std::string::npos);
  }
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
