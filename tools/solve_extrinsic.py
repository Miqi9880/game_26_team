#!/usr/bin/env python3
"""Solve camera->gimbal extrinsic from look-at samples.

Each CSV row: yaw_deg pitch_deg tx ty tz
  yaw_deg / pitch_deg : gimbal absolute aim angles (RobotCtrl convention,
    +yaw left, +pitch up) when the target is near the image center.
  tx ty tz            : target position in the camera frame from PnP
    (x right, y down, z forward), metres.

Direction model: n_gimbal = R_gimbal_from_camera * n_cam, solved by
orthogonal Procrustes (SVD).  Translation is not recoverable from
directions; supply the measured t with --translation-m.

Samples MUST mix yaw and pitch changes (pure-yaw poses are coplanar and
degenerate).  Keep the target within a few degrees of the image center.

Usage:
  python3 tools/solve_extrinsic.py samples.csv \
      --translation-m 0.100,-0.020,0.030
"""

import csv
import math
import sys

import numpy as np


def main() -> None:
    if len(sys.argv) < 2:
        sys.exit("usage: solve_extrinsic.py samples.csv [--translation-m x,y,z]")
    path = sys.argv[1]
    translation: list[float] | None = None
    if "--translation-m" in sys.argv:
        index = sys.argv.index("--translation-m")
        translation = [float(value) for value in sys.argv[index + 1].split(",")]
        if len(translation) != 3:
            sys.exit("--translation-m must have exactly three values")
    rows: list[list[float]] = []
    with open(path, newline="", encoding="utf-8") as handle:
        for record in csv.reader(handle):
            if not record or record[0].lstrip().startswith("#"):
                continue
            values = [float(value) for value in record[:5]]
            if len(values) == 5:
                rows.append(values)
    if len(rows) < 2:
        sys.exit("need at least 2 samples (mix yaw and pitch changes!)")
    camera_directions = []
    gimbal_directions = []
    for yaw_deg, pitch_deg, tx, ty, tz in rows:
        yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
        gimbal_direction = (math.cos(pitch) * math.cos(yaw),
                            math.cos(pitch) * math.sin(yaw),
                            math.sin(pitch))
        camera_direction = np.array([tx, ty, tz], dtype=float)
        norm = np.linalg.norm(camera_direction)
        if norm <= 1e-9:
            sys.exit("camera translation must be non-zero")
        camera_directions.append(camera_direction / norm)
        gimbal_directions.append(np.array(gimbal_direction, dtype=float))
    camera = np.array(camera_directions).T  # 3 x N
    gimbal = np.array(gimbal_directions).T  # 3 x N
    u, _, vt = np.linalg.svd(gimbal @ camera.T)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    residuals_deg = []
    for index in range(len(rows)):
        estimated = rotation @ camera[:, index]
        cosine = max(-1.0, min(1.0, float(np.dot(estimated, gimbal[:, index]))))
        residuals_deg.append(math.degrees(math.acos(cosine)))
    print(f"samples={len(rows)} det(R)={np.linalg.det(rotation):.6f} "
          f"max_err_deg={max(residuals_deg):.4f} "
          f"mean_err_deg={sum(residuals_deg) / len(residuals_deg):.4f}")
    print("R_gimbal_from_camera (row-major):")
    print("  rotation_gimbal_from_camera: [" +
          ", ".join(f"{value:.9f}" for row in rotation for value in row) + "]")
    if translation is not None:
        print("  translation_gimbal_from_camera_m: [" +
              ", ".join(f"{value:.6f}" for value in translation) + "]")


if __name__ == "__main__":
    main()
