#!/usr/bin/env python3
"""Merge per-pose smoke CSVs into samples.csv for solve_extrinsic.py.

Each smoke CSV row is one detection; only pnp_valid=1 rows are kept, and the
camera-frame translation (camera_x_m/y/z_m) is averaged per pose.  The poses
file lists one "yaw_deg pitch_deg" per line in the same order as the CSVs.

Usage:
  python3 tools/build_lookat_samples.py poses.txt smoke1.csv smoke2.csv ...
"""

import csv
import sys


def main() -> None:
    if len(sys.argv) < 3:
        sys.exit("usage: build_lookat_samples.py poses.txt smoke.csv ...")
    poses = []
    with open(sys.argv[1], encoding="utf-8") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            parts = stripped.split()
            if len(parts) == 2:
                poses.append((float(parts[0]), float(parts[1])))
    if len(poses) != len(sys.argv) - 2:
        sys.exit(
            f"poses file has {len(poses)} poses but {len(sys.argv) - 2} CSVs "
            "were given")
    samples = []
    for (yaw, pitch), path in zip(poses, sys.argv[2:]):
        sums = [0.0, 0.0, 0.0]
        count = 0
        with open(path, newline="", encoding="utf-8") as handle:
            for record in csv.DictReader(handle):
                if record.get("pnp_valid") != "1":
                    continue
                for index, name in enumerate(
                        ["camera_x_m", "camera_y_m", "camera_z_m"]):
                    value = record.get(name)
                    if value is None or value == "":
                        sys.exit(f"{path}: missing {name} in a pnp_valid row")
                    sums[index] += float(value)
                count += 1
        if count == 0:
            sys.exit(
                f"{path}: no pnp_valid rows; check the annotated frames and "
                "re-record this pose")
        samples.append([yaw, pitch] + [total / count for total in sums])
        print(f"{path}: {count} valid rows, mean t = "
              f"{samples[-1][2]:.4f},{samples[-1][3]:.4f},{samples[-1][4]:.4f}")
    with open("samples.csv", "w", newline="", encoding="utf-8") as handle:
        csv.writer(handle).writerows(samples)
    print(f"wrote samples.csv with {len(samples)} poses "
          "(yaw_deg pitch_deg tx ty tz)")


if __name__ == "__main__":
    main()
