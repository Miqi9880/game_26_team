#!/bin/bash
# Launch the Hik camera with the calibrated camera_info file.
# The calibrated CameraInfo is required so the dataset recorder can pair
# images with camera_info (otherwise every frame is rejected).
# Run from the repo root: bash tools/run_camera.sh
ros2 launch hik_camera hik_camera.launch.py \
  camera_info_url:=file:///home/nvidia/game_26_orin_main/src/ros2-hik-camera/config/camera_info_calibrated.yaml
