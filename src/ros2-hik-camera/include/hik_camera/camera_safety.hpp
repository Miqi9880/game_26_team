#ifndef HIK_CAMERA__CAMERA_SAFETY_HPP_
#define HIK_CAMERA__CAMERA_SAFETY_HPP_

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

DeviceSelectionResult selectCameraDevice(
  const std::vector<std::string> & serial_numbers,
  const std::string & requested_serial);

bool calculateRgb8BufferSize(
  std::uint32_t width,
  std::uint32_t height,
  std::size_t & buffer_size);

bool isFiniteInRange(double value, double minimum, double maximum);

std::string formatSdkStatus(int status);

}  // namespace hik_camera

#endif  // HIK_CAMERA__CAMERA_SAFETY_HPP_