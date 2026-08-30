#!/bin/bash
# Start the serial RobotCtrl bridge with the serial port enabled
# (default /dev/robomaster; pass another path as $1).
# Run from the repo root with the workspace sourced:
#   bash tools/run_robot_ctrl.sh
#   bash tools/run_robot_ctrl.sh /dev/ttyUSB0
# vehicle_profile must be new_turtle or dog_leg; "unselected" makes the
# safety layer reject every command (profile_unselected).
PROFILE=${2:-new_turtle}
DEV=${1:-/dev/robomaster}
ros2 run serical_device_ros2 robot_ctrl_main --ros-args \
  -p serial_device:=$DEV -p serial_enabled:=true -p dry_run:=false \
  -p vehicle_profile:=$PROFILE
