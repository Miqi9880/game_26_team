#ifndef AUTO_AIM_ROS2__ANGLE_UNITS_HPP_
#define AUTO_AIM_ROS2__ANGLE_UNITS_HPP_

#include <cmath>

namespace rm_auto_aim::units
{

// The ROS/serial position-angle contract is degree.  The algorithm core and
// PnP contract is radian.  Keep these conversions in this one adapter header;
// the serial bridge must remain a byte/field mapping layer with no conversion.
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

inline bool finite(float value) noexcept
{
  return std::isfinite(value);
}

}  // namespace rm_auto_aim::units

#endif  // AUTO_AIM_ROS2__ANGLE_UNITS_HPP_
