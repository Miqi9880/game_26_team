#!/bin/bash
# Run auto_aim_pnp_smoke on each recorded pose directory
# ($HOME/calib/pose01..pose06/images/frame_%06d.png) with the production
# model profile and pnp config.  Writes $HOME/calib/poseNN.csv (camera-frame
# pose; gimbal extrinsic is not configured yet, so no gimbal output).
# Run from the repo root with the workspace sourced:
#   bash tools/run_smoke_poses.sh
for n in 01 02 03 04 05 06; do
  ros2 run auto_aim_ros2 auto_aim_pnp_smoke \
    --model /home/nvidia/game_26_orin_main/models/yolov5.xml \
    --model-profile "$PWD/config/model_profile.yaml" \
    --video "$HOME/calib/pose$n/images/frame_%06d.png" \
    --pnp-config "$PWD/config/pnp_config.yaml" \
    --frames 5 --csv "$HOME/calib/pose$n.csv" \
    --annotated-dir "$HOME/calib/pose$n_annotated"
done
echo "smoke CSVs written: $HOME/calib/pose01.csv..pose06.csv"
