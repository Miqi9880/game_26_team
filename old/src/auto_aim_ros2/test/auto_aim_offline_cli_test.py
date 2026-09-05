#!/usr/bin/env python3
"""Fail-closed CLI contract tests for the offline ballistic diagnostic.

Every invocation deliberately uses nonexistent fixture paths.  The assertions
exercise option parsing before model/video/PnP loading, so this test cannot
open hardware, a serial port, ROS, or a real model artifact.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

if len(sys.argv) != 2:
    raise RuntimeError("expected auto_aim_offline executable path")

EXECUTABLE = Path(sys.argv.pop(1))


class OfflineCliBallisticContractTests(unittest.TestCase):
    @classmethod
    def run_tool(cls, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(EXECUTABLE), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )

    @staticmethod
    def base_arguments(video: str = "missing.avi") -> tuple[str, ...]:
        return (
            "--model", "missing.xml",
            "--model-profile", "missing.yaml",
            "--video", video,
            "--pnp-config", "missing_pnp.yaml",
        )

    def test_help_exposes_explicit_test_only_boundary(self) -> None:
        result = self.run_tool("--help")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        for option in (
            "--ballistic-diagnostic",
            "--ballistic-bullet-speed-mps",
            "--ballistic-system-latency-ms",
            "--allow-test-gimbal-origin-as-muzzle",
        ):
            self.assertIn(option, result.stderr)

    def test_ballistic_inputs_require_explicit_enable_speed_latency_and_limits(self) -> None:
        missing_speed = self.run_tool(*self.base_arguments(), "--ballistic-diagnostic")
        self.assertNotEqual(missing_speed.returncode, 0)
        self.assertIn("requires --ballistic-bullet-speed-mps", missing_speed.stderr)

        missing_latency = self.run_tool(
            *self.base_arguments(),
            "--ballistic-diagnostic",
            "--ballistic-bullet-speed-mps", "20",
            "--ballistic-gravity-mps2", "9.81",
            "--ballistic-max-flight-time-ms", "500",
        )
        self.assertNotEqual(missing_latency.returncode, 0)
        self.assertIn("requires --ballistic-system-latency-ms", missing_latency.stderr)

        invalid_speed = self.run_tool(
            *self.base_arguments(),
            "--ballistic-diagnostic",
            "--ballistic-bullet-speed-mps", "0",
            "--ballistic-gravity-mps2", "9.81",
            "--ballistic-system-latency-ms", "0",
            "--ballistic-max-flight-time-ms", "500",
        )
        self.assertNotEqual(invalid_speed.returncode, 0)
        self.assertIn("ballistic speed, gravity, and flight limit must be positive", invalid_speed.stderr)

        implicit_enable = self.run_tool(*self.base_arguments(), "--ballistic-bullet-speed-mps", "20")
        self.assertNotEqual(implicit_enable.returncode, 0)
        self.assertIn("require explicit --ballistic-diagnostic", implicit_enable.stderr)

    def test_explicit_zero_latency_reaches_safe_file_validation_not_defaulting(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            video = Path(directory) / "offline.avi"
            video.write_bytes(b"not a real video")
            result = self.run_tool(
                *self.base_arguments(str(video)),
                "--ballistic-diagnostic",
                "--ballistic-bullet-speed-mps", "20",
                "--ballistic-gravity-mps2", "9.81",
                "--ballistic-system-latency-ms", "0",
                "--ballistic-max-flight-time-ms", "500",
            )
        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("requires --ballistic-system-latency-ms", result.stderr)
        self.assertIn("missing_pnp.yaml", result.stderr)

    def test_nonregular_video_is_rejected_before_any_capture_or_model_load(self) -> None:
        result = self.run_tool(*self.base_arguments("/dev/null"))
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("video input must be an existing regular local file", result.stderr)
        self.assertNotIn("missing_pnp.yaml", result.stderr)

    def test_csv_output_cannot_alias_an_input_path(self) -> None:
        result = self.run_tool(*self.base_arguments(), "--csv", "missing.xml")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CSV output must not alias an input path", result.stderr)

    def test_nonregular_csv_output_is_rejected_without_opening_it(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            fifo = Path(directory) / "report.fifo"
            try:
                fifo_path = str(fifo)
                import os
                os.mkfifo(fifo_path)
            except (AttributeError, OSError) as exc:
                self.skipTest(f"FIFO unavailable: {exc}")
            result = self.run_tool(*self.base_arguments(), "--csv", fifo_path)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("CSV output must be a regular file", result.stderr)


if __name__ == "__main__":
    unittest.main()
