#ifndef AUTO_AIM_ROS2__ANGLE_UNITS_HPP_
#define AUTO_AIM_ROS2__ANGLE_UNITS_HPP_

#include <cmath>

namespace rm_auto_aim::units
{

// ROS/serial positions use degree, velocities use degree/s, and accelerations
// use degree/s^2.  The algorithm uses rad, rad/s, and rad/s^2.  Keep these
// conversions at the ROS/algorithm boundary; the serial bridge must remain a
// byte/field mapping layer with no conversion.
inline constexpr double kPi = 3.141592653589793238462643383279502884;
inline constexpr double kDegreesPerRadian = 180.0 / kPi;
inline constexpr double kRadiansPerDegree = kPi / 180.0;

inline float degrees_to_radians(float degrees) noexcept
{
  return static_cast<float>(static_cast<double>(degrees) * kRadiansPerDegree);
}

inline float radians_to_degrees(float radians) noexcept
{
  return static_cast<float>(static_cast<double>(radians) * kDegreesPerRadian);
}

inline float degrees_per_second_to_radians_per_second(float degrees_per_second) noexcept
{
  return static_cast<float>(static_cast<double>(degrees_per_second) * kRadiansPerDegree);
}

inline float radians_per_second_to_degrees_per_second(float radians_per_second) noexcept
{
  return static_cast<float>(static_cast<double>(radians_per_second) * kDegreesPerRadian);
}

inline float degrees_per_second_squared_to_radians_per_second_squared(
  float degrees_per_second_squared) noexcept
{
  return static_cast<float>(
    static_cast<double>(degrees_per_second_squared) * kRadiansPerDegree);
}

inline float radians_per_second_squared_to_degrees_per_second_squared(
  float radians_per_second_squared) noexcept
{
  return static_cast<float>(
    static_cast<double>(radians_per_second_squared) * kDegreesPerRadian);
}

inline bool finite(float value) noexcept
{
  return std::isfinite(value);
}

}  // namespace rm_auto_aim::units

#endif  // AUTO_AIM_ROS2__ANGLE_UNITS_HPP_
