#ifndef SERICAL_DEVICE_ROS2__SERIAL_ROS_MAPPING_HPP_
#define SERICAL_DEVICE_ROS2__SERIAL_ROS_MAPPING_HPP_

#include <cstddef>

#include "auto_aim_interfaces/msg/robot_ctrl.hpp"
#include "auto_aim_interfaces/msg/vision.hpp"
#include "protocol_new.hpp"

namespace rm_auto_aim::serial_ros
{

// These helpers are deliberately raw field mappings.  VisionData and
// RobotCtrlData use degree for position angles, and the ROS messages use the
// same external units.  Unit conversion belongs only to auto_aim_ros2's
// algorithm adapter, never in this serial package.
inline auto to_ros_vision(const io::VisionData & data)
  -> auto_aim_interfaces::msg::Vision
{
  auto message = auto_aim_interfaces::msg::Vision{};
  message.id = data.id;
  message.mode = data.mode;
  message.yaw = data.yaw;
  message.yaw_vel = data.yaw_vel;
  message.pitch = data.pitch;
  message.pitch_vel = data.pitch_vel;
  message.roll = data.roll;
  for (std::size_t index = 0; index < 4; ++index) {
    message.quaternion[index] = data.quaternion[index];
  }
  message.shoot_speed = data.shoot_speed;
  message.bullet_count = data.bullet_count;
  message.game_progress = data.game_progress;
  return message;
}

inline auto to_serial_robot_ctrl(const auto_aim_interfaces::msg::RobotCtrl & message)
  -> io::RobotCtrlData
{
  io::RobotCtrlData data{};
  data.yaw = message.yaw;
  data.yaw_vel = message.yaw_vel;
  data.yaw_acc = message.yaw_acc;
  data.pitch = message.pitch;
  data.pitch_vel = message.pitch_vel;
  data.pitch_acc = message.pitch_acc;
  data.target_lock = message.target_lock;
  data.fire_command = message.fire_command;
  return data;
}

}  // namespace rm_auto_aim::serial_ros

#endif  // SERICAL_DEVICE_ROS2__SERIAL_ROS_MAPPING_HPP_
