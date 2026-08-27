#!/usr/bin/env python3
"""CLI contract tests for the synthetic offline scenario benchmark.

The executable only receives its own deterministic fixture arguments. These
tests do not provide camera, model, ROS, serial, or hardware input.
"""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

if len(sys.argv) != 2:
    raise RuntimeError("expected auto_aim_scenario_benchmark executable path")

EXECUTABLE = Path(sys.argv.pop(1))


class ScenarioBenchmarkCliTests(unittest.TestCase):
    @classmethod
    def run_tool(cls, *arguments: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(EXECUTABLE), *arguments],
            check=False,
            capture_output=True,
            text=True,
        )

    @staticmethod
    def arguments(scenario: str, seed: str, output_dir: Path) -> tuple[str, ...]:
        return (
            "--scenario", scenario,
            "--seed", seed,
            "--output-dir", str(output_dir),
        )

    def test_help_is_explicit_and_has_no_side_effect(self) -> None:
        result = self.run_tool("--help")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn("--scenario <name|all>", result.stdout)
        self.assertIn("--seed <uint64>", result.stdout)
        self.assertIn("--output-dir <new-path>", result.stdout)
        self.assertIn("software-only synthetic benchmark", result.stdout)

    def test_same_scenario_and_seed_create_byte_identical_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            first = root / "first"
            second = root / "second"
            arguments = self.arguments("all", "18446744073709551615", first)
            first_run = self.run_tool(*arguments)
            self.assertEqual(first_run.returncode, 0, first_run.stdout + first_run.stderr)

            second_run = self.run_tool(*self.arguments(
                "all", "18446744073709551615", second))
            self.assertEqual(second_run.returncode, 0, second_run.stdout + second_run.stderr)

            for text in (
                "software_only_synthetic_benchmark=true",
                "serial_enabled=false",
                "dry_run=true",
                "allow_fire=false",
                "fire_command=0",
                "yaw_vel=0",
                "pitch_vel=0",
                "yaw_acc=0",
                "pitch_acc=0",
            ):
                self.assertIn(text, first_run.stdout)

            for name in ("benchmark.csv", "summary.json", "summary.md"):
                first_bytes = (first / name).read_bytes()
                second_bytes = (second / name).read_bytes()
                self.assertEqual(first_bytes, second_bytes, name)

    def test_unknown_or_missing_required_options_fail_before_writing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            unknown_output = root / "unknown"
            unknown = self.run_tool(*self.arguments("not-a-scenario", "1", unknown_output))
            self.assertEqual(unknown.returncode, 2)
            self.assertIn("unknown scenario", unknown.stderr)
            self.assertFalse(unknown_output.exists())

            missing = self.run_tool("--scenario", "static_3m", "--seed", "1")
            self.assertEqual(missing.returncode, 2)
            self.assertIn("are required", missing.stderr)

            overflow = self.run_tool(*self.arguments(
                "static_3m", "18446744073709551616", root / "overflow"))
            self.assertEqual(overflow.returncode, 2)
            self.assertIn("unsigned 64-bit integer", overflow.stderr)

    def test_existing_output_directory_is_not_mutated(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "existing"
            output.mkdir()
            sentinel = output / "sentinel.txt"
            sentinel.write_text("preserve me", encoding="utf-8")

            result = self.run_tool(*self.arguments("static_3m", "7", output))

            self.assertEqual(result.returncode, 2)
            self.assertIn("must not already exist", result.stderr)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "preserve me")
            self.assertFalse((output / "benchmark.csv").exists())
            self.assertFalse((output / "summary.json").exists())
            self.assertFalse((output / "summary.md").exists())


if __name__ == "__main__":
    unittest.main()
