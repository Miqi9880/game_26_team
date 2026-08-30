#!/bin/bash
# Merge per-pose smoke CSVs into samples.csv, and make sure
# config/lookat_translation.txt exists (edit it with the measured
# camera->gimbal translation before solving).
# Run from the repo root: bash tools/build_samples.sh
if [ ! -f config/lookat_translation.txt ]; then
  echo "0.000 0.000 0.000" > config/lookat_translation.txt
  echo "created config/lookat_translation.txt; edit the three numbers"
fi
python3 tools/build_lookat_samples.py config/lookat_poses.txt \
  "$HOME/calib/pose0"{1..6}.csv
