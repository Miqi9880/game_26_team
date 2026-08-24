#include "hik_camera/camera_safety.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>

namespace hik_camera
{

DeviceSelectionResult selectCameraDevice(
  const std::vector<std::string> & serial_numbers,
  const std::string & requested_serial)
{
  DeviceSelectionResult result;

  if (serial_numbers.empty()) {
    result.reason = "no camera was found";
    return result;
  }

  if (requested_serial.empty()) {
    if (serial_numbers.size() != 1U) {
      result.reason = "multiple cameras were found; camera_serial is required";
      return result;
    }

    result.success = true;
    result.index = 0U;
    return result;
  }

  std::size_t match_count = 0U;
  std::size_t match_index = 0U;
  for (std::size_t index = 0U; index < serial_numbers.size(); ++index) {
    if (serial_numbers[index] == requested_serial) {
      ++match_count;
      match_index = index;
    }
  }

  if (match_count == 0U) {
    result.reason = "the requested camera serial was not found";
    return result;
  }

  if (match_count > 1U) {
    result.reason = "the requested camera serial is not unique";
    return result;
  }

  result.success = true;
  result.index = match_index;
  return result;
}

bool calculateRgb8BufferSize(
  std::uint32_t width,
  std::uint32_t height,
  std::size_t & buffer_size)
{
  buffer_size = 0U;
  if (width == 0U || height == 0U) {
    return false;
  }

  const auto max_size = std::numeric_limits<std::size_t>::max();
  const auto width_size = static_cast<std::size_t>(width);
  const auto height_size = static_cast<std::size_t>(height);

  if (width_size > max_size / height_size) {
    return false;
  }

  const auto pixel_count = width_size * height_size;
  constexpr std::size_t channels = 3U;
  if (pixel_count > max_size / channels) {
    return false;
  }

  buffer_size = pixel_count * channels;
  return true;
}

bool isFiniteInRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && std::isfinite(minimum) &&
         std::isfinite(maximum) && minimum <= maximum &&
         value >= minimum && value <= maximum;
}

std::string formatSdkStatus(int status)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setfill('0')
         << std::setw(8) << static_cast<std::uint32_t>(status);
  return stream.str();
}

}  // namespace hik_camera