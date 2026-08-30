#!/bin/bash
# Collect look-at calibration data: for each (yaw, pitch) pose in
# config/lookat_poses.txt, command the gimbal, wait for settle, and record
# ~5 raw camera frames with the calibration dataset recorder.
# Run from the repo root with the workspace sourced:
#   bash tools/collect_lookat_poses.sh
set -e
n=0
while read -r y p; do
  n=$((n + 1))
  python3 tools/cmd_gimbal.py "$y" "$p"
  sleep 2
  ros2 run auto_aim_tools auto_aim_calibration_dataset_recorder \
    --config "$PWD/config/lookat_dataset.yaml" \
    --output "$HOME/calib/pose0$n" --max-frames 5 --timeout-s 15
done < config/lookat_poses.txt
echo "recorded $n poses under $HOME/calib/pose01..pose0$n"
