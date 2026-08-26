"""Regression tests for qualification report output path safety."""

from __future__ import annotations

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
CLI = ROOT / "tools/auto_aim_qualification/auto_aim_qualification.py"
MODEL_PROFILE = ROOT / "src/auto_aim_ros2/test/data/model_profile_test.yaml"
PNP_CONFIG = ROOT / "src/auto_aim_ros2/test/data/pnp_test_config.yaml"


class OutputSafetyTests(unittest.TestCase):
    def _command(self, output_json: Path, output_markdown: Path) -> list[str]:
        return [
            sys.executable,
            str(CLI),
            "--allow-test-only",
            "--model-profile",
            str(MODEL_PROFILE),
            "--pnp-config",
            str(PNP_CONFIG),
            "--output-json",
            str(output_json),
            "--output-markdown",
            str(output_markdown),
        ]

    def test_dangling_json_or_markdown_symlink_is_rejected_without_external_write(self):
        for output_kind in ("json", "markdown"):
            with self.subTest(output_kind=output_kind), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                victim = root / f"outside.{output_kind}"
                output = root / "qualification.json"
                markdown = root / "qualification.md"
                unsafe = output if output_kind == "json" else markdown
                unsafe.symlink_to(victim)

                result = subprocess.run(self._command(output, markdown), check=False, capture_output=True, text=True)

                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("status=FAIL", result.stdout)
                self.assertTrue(unsafe.is_symlink())
                self.assertFalse(victim.exists(), "dangling output symlink must never create its target")
                self.assertFalse(output.exists() and not output.is_symlink())
                self.assertFalse(markdown.exists() and not markdown.is_symlink())

    def test_dangling_parent_symlink_is_rejected_without_external_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outside = root / "outside"
            parent = root / "reports"
            parent.symlink_to(outside, target_is_directory=True)
            output = parent / "qualification.json"
            markdown = root / "qualification.md"

            result = subprocess.run(self._command(output, markdown), check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertFalse((outside / "qualification.json").exists())
            self.assertFalse(markdown.exists())

    def test_live_parent_symlink_is_rejected_without_external_write(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outside = root / "outside"
            outside.mkdir()
            parent = root / "reports"
            parent.symlink_to(outside, target_is_directory=True)
            output = parent / "qualification.json"
            markdown = root / "qualification.md"

            result = subprocess.run(self._command(output, markdown), check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertFalse((outside / "qualification.json").exists())
            self.assertFalse(markdown.exists())

    def test_dangling_intermediate_symlink_is_rejected_before_mkdir(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            outside = root / "outside"
            outside.mkdir()
            parent = root / "reports"
            parent.symlink_to(outside, target_is_directory=True)
            output = parent / "new" / "qualification.json"
            markdown = root / "qualification.md"

            result = subprocess.run(self._command(output, markdown), check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertFalse((outside / "new").exists(), "mkdir must not follow an intermediate symlink")
            self.assertFalse(markdown.exists())

    def test_existing_hardlink_output_is_rejected_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            victim = root / "victim.json"
            victim.write_text("keep", encoding="utf-8")
            output = root / "qualification.json"
            output.hardlink_to(victim)
            markdown = root / "qualification.md"

            result = subprocess.run(self._command(output, markdown), check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertEqual(victim.read_text(encoding="utf-8"), "keep")
            self.assertEqual(output.read_text(encoding="utf-8"), "keep")
            self.assertFalse(markdown.exists())


if __name__ == "__main__":
    unittest.main()
