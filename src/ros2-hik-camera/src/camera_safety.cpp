#include "hik_camera/camera_safety.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <sys/stat.h>

namespace
{

int hexadecimalValue(char character)
{
  if (character >= '0' && character <= '9') {
    return character - '0';
  }
  if (character >= 'a' && character <= 'f') {
    return character - 'a' + 10;
  }
  if (character >= 'A' && character <= 'F') {
    return character - 'A' + 10;
  }
  return -1;
}

bool fileUrlPath(const std::string & url, std::string & path, std::string & reason)
{
  constexpr const char * file_scheme = "file://";
  if (url.compare(0U, std::strlen(file_scheme), file_scheme) != 0) {
    if (url.find("://") != std::string::npos) {
      reason = "resolved camera_info_url must use the file scheme";
      return false;
    }
    path = url;
    return !path.empty();
  }

  std::string encoded = url.substr(std::strlen(file_scheme));
  constexpr const char * localhost_prefix = "localhost/";
  if (encoded.compare(0U, std::strlen(localhost_prefix), localhost_prefix) == 0) {
    encoded.erase(0U, std::strlen(localhost_prefix) - 1U);
  }
  if (encoded.empty() || encoded.front() != '/') {
    reason = "resolved file camera_info_url must contain an absolute path";
    return false;
  }

  path.clear();
  path.reserve(encoded.size());
  for (std::size_t index = 0U; index < encoded.size(); ++index) {
    if (encoded[index] != '%') {
      path.push_back(encoded[index]);
      continue;
    }
    if (index + 2U >= encoded.size()) {
      reason = "resolved camera_info_url contains an incomplete percent escape";
      return false;
    }
    const int high = hexadecimalValue(encoded[index + 1U]);
    const int low = hexadecimalValue(encoded[index + 2U]);
    if (high < 0 || low < 0 || (high == 0 && low == 0)) {
      reason = "resolved camera_info_url contains an invalid percent escape";
      return false;
    }
    path.push_back(static_cast<char>((high << 4) | low));
    index += 2U;
  }
  return true;
}

struct CanonicalFile
{
  std::string path;
  dev_t device{};
  ino_t inode{};
};

bool canonicalFile(
  const std::string & url, CanonicalFile & result, std::string & reason)
{
  std::string path;
  if (!fileUrlPath(url, path, reason)) {
    return false;
  }

  std::unique_ptr<char, decltype(& std::free)> canonical(
    realpath(path.c_str(), nullptr), &std::free);
  if (!canonical) {
    reason = "camera_info_url does not resolve to an existing file: " +
      std::string(std::strerror(errno));
    return false;
  }

  struct stat status {};
  if (stat(canonical.get(), &status) != 0 || !S_ISREG(status.st_mode)) {
    reason = "camera_info_url must resolve to a regular file";
    return false;
  }
  result.path = canonical.get();
  result.device = status.st_dev;
  result.inode = status.st_ino;
  return true;
}

}  // namespace

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

ContractValidationResult validateCameraInfoContract(
  const CameraInfoContractSample & sample)
{
  if (sample.width == 0U || sample.height == 0U) {
    return {false, "CameraInfo width and height must both be positive"};
  }
  if (!std::all_of(
      sample.k.begin(), sample.k.end(), [](double value) {
        return std::isfinite(value);
      }))
  {
    return {false, "CameraInfo K contains a non-finite value"};
  }
  if (!(sample.k[0] > 0.0) || !(sample.k[4] > 0.0) ||
    std::abs(sample.k[8]) <= 1e-12)
  {
    return {false, "CameraInfo K requires positive fx/fy and non-zero K[8]"};
  }
  if (!std::all_of(
      sample.d.begin(), sample.d.end(), [](double value) {
        return std::isfinite(value);
      }))
  {
    return {false, "CameraInfo D contains a non-finite value"};
  }

  std::size_t expected_d_size = 0U;
  if (sample.distortion_model == "plumb_bob") {
    expected_d_size = 5U;
  } else if (sample.distortion_model == "rational_polynomial") {
    expected_d_size = 8U;
  } else if (sample.distortion_model == "equidistant") {
    expected_d_size = 4U;
  } else {
    return {false, "CameraInfo distortion_model is empty or unsupported"};
  }
  if (sample.d.size() != expected_d_size) {
    return {false, "CameraInfo D length does not match distortion_model"};
  }
  return {true, "CameraInfo satisfies the ROS input contract"};
}

ContractValidationResult validateCameraInfoUrlProvenance(
  const std::string & requested_url,
  const std::string & resolved_url,
  const std::vector<std::string> & unverified_resolved_urls)
{
  CanonicalFile requested;
  std::string reason;
  if (!canonicalFile(resolved_url, requested, reason)) {
    return {false, reason};
  }

  for (const auto & unverified_url : unverified_resolved_urls) {
    CanonicalFile unverified;
    std::string ignored_reason;
    if (!canonicalFile(unverified_url, unverified, ignored_reason)) {
      continue;
    }
    const bool same_path = requested.path == unverified.path;
    const bool same_file = requested.device == unverified.device &&
      requested.inode == unverified.inode;
    if (same_path || same_file) {
      return {
        false,
        "camera_info_url resolves to the checked-in unverified format example: " +
        requested_url,
      };
    }
  }
  return {true, "camera_info_url resolves to an external calibration file"};
}

bool cameraInfoMatchesFrame(
  const CameraInfoContractSample & sample,
  std::uint32_t frame_width,
  std::uint32_t frame_height)
{
  return sample.width == frame_width && sample.height == frame_height;
}

bool isFiniteInRange(double value, double minimum, double maximum)
{
  return std::isfinite(value) && std::isfinite(minimum) &&
         std::isfinite(maximum) && minimum <= maximum &&
         value >= minimum && value <= maximum;
}

std::string boundedByteString(const unsigned char * data, std::size_t capacity)
{
  if (data == nullptr || capacity == 0U) {
    return {};
  }

  std::size_t length = 0U;
  while (length < capacity && data[length] != '\0') {
    ++length;
  }

  return std::string(
    reinterpret_cast<const char *>(data), length);
}

std::string formatSdkStatus(int status)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setfill('0')
         << std::setw(8) << static_cast<std::uint32_t>(status);
  return stream.str();
}

}  // namespace hik_camera
