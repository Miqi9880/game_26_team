#ifndef HIK_CAMERA__CAMERA_SAFETY_HPP_
#define HIK_CAMERA__CAMERA_SAFETY_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace hik_camera
{

struct DeviceSelectionResult
{
  bool success{false};
  std::size_t index{0};
  std::string reason;
};

struct CameraInfoContractSample
{
  std::uint32_t width{0U};
  std::uint32_t height{0U};
  std::string distortion_model;
  std::array<double, 9U> k{{}};
  std::vector<double> d;
};

struct ContractValidationResult
{
  bool valid{false};
  std::string reason;
};

DeviceSelectionResult selectCameraDevice(
  const std::vector<std::string> & serial_numbers,
  const std::string & requested_serial);

bool calculateRgb8BufferSize(
  std::uint32_t width,
  std::uint32_t height,
  std::size_t & buffer_size);

ContractValidationResult validateCameraInfoContract(
  const CameraInfoContractSample & sample);

ContractValidationResult validateCameraInfoUrlProvenance(
  const std::string & requested_url,
  const std::string & resolved_url,
  const std::vector<std::string> & unverified_resolved_urls);

bool cameraInfoMatchesFrame(
  const CameraInfoContractSample & sample,
  std::uint32_t frame_width,
  std::uint32_t frame_height);

bool isFiniteInRange(double value, double minimum, double maximum);

std::string boundedByteString(const unsigned char * data, std::size_t capacity);

std::string formatSdkStatus(int status);

}  // namespace hik_camera

#endif  // HIK_CAMERA__CAMERA_SAFETY_HPP_
