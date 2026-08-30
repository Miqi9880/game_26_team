#!/bin/bash
# Start the serial RobotCtrl bridge with the serial port enabled
# (default /dev/robomaster; pass another path as $1).
# Run from the repo root with the workspace sourced:
#   bash tools/run_robot_ctrl.sh
#   bash tools/run_robot_ctrl.sh /dev/ttyUSB0
DEV=${1:-/dev/robomaster}
ros2 run serical_device_ros2 robot_ctrl_main --ros-args \
  -p serial_device:=$DEV -p serial_enabled:=true -p dry_run:=false
