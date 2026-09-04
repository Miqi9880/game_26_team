"""Regression tests for qualification report output path safety."""

from __future__ import annotations

import hashlib
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

    def _assert_read_only_input_is_protected(self, argument: str, source: Path, *, extra: list[str] | None = None):
        """A report alias must fail before either report can be written."""

        for output_kind in ("json", "markdown"):
            with self.subTest(argument=argument, output_kind=output_kind), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                copied = root / source.name
                copied.write_bytes(source.read_bytes())
                before = hashlib.sha256(copied.read_bytes()).hexdigest()
                output_json = copied if output_kind == "json" else root / "qualification.json"
                output_markdown = copied if output_kind == "markdown" else root / "qualification.md"
                command = self._command(output_json, output_markdown) + [argument, str(copied)]
                if extra:
                    command.extend(extra)

                result = subprocess.run(command, check=False, capture_output=True, text=True)

                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("status=FAIL", result.stdout)
                self.assertEqual(hashlib.sha256(copied.read_bytes()).hexdigest(), before)
                # The aliased output path is the protected source itself; the
                # other report must not be created as a partial side effect.
                other_output = output_markdown if output_json == copied else output_json
                self.assertFalse(other_output.exists() and not other_output.is_symlink())

    def test_all_file_inputs_are_protected_from_json_and_markdown_aliases(self):
        cases = (
            ("--model-profile", MODEL_PROFILE),
            ("--pnp-config", PNP_CONFIG),
            ("--model-bin", MODEL_PROFILE),
            ("--metadata-json", ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"),
            ("--manifest", ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"),
            ("--camera-intrinsic-report", ROOT / "tools/offline_evidence_report/fixtures/bundle/camera_intrinsic_evidence.yaml"),
        )
        for argument, source in cases:
            self._assert_read_only_input_is_protected(argument, source)

    def test_producer_command_input_is_protected(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "producer_command.txt"
            source.write_text("python3 producer.py --dry-run\n", encoding="utf-8")
            self._assert_read_only_input_is_protected("--producer-command-file", source)

    def test_live_hardlink_and_symlink_input_aliases_are_protected(self):
        """samefile aliases must fail before either report is written."""

        for alias_kind in ("hardlink", "symlink"):
            for output_kind in ("json", "markdown"):
                with self.subTest(alias_kind=alias_kind, output_kind=output_kind), tempfile.TemporaryDirectory() as directory:
                    root = Path(directory)
                    source = root / "profile.yaml"
                    source.write_bytes(MODEL_PROFILE.read_bytes())
                    alias = root / "profile-alias.yaml"
                    if alias_kind == "hardlink":
                        alias.hardlink_to(source)
                    else:
                        alias.symlink_to(source)
                    before = hashlib.sha256(source.read_bytes()).hexdigest()
                    output_json = alias if output_kind == "json" else root / "qualification.json"
                    output_markdown = alias if output_kind == "markdown" else root / "qualification.md"

                    result = subprocess.run(
                        self._command(output_json, output_markdown) + ["--model-profile", str(source)],
                        check=False,
                        capture_output=True,
                        text=True,
                    )

                    self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                    self.assertIn("status=FAIL", result.stdout)
                    self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), before)
                    other_output = output_markdown if output_kind == "json" else output_json
                    self.assertFalse(other_output.exists() and not other_output.is_symlink())

    def test_hardlink_and_symlink_aliases_for_each_file_input_are_protected(self):
        cases = (
            ("--model-profile", MODEL_PROFILE),
            ("--pnp-config", PNP_CONFIG),
            ("--model-bin", MODEL_PROFILE),
            ("--metadata-json", ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"),
            ("--manifest", ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"),
            ("--camera-intrinsic-report", ROOT / "tools/offline_evidence_report/fixtures/bundle/camera_intrinsic_evidence.yaml"),
        )
        for argument, source_fixture in cases:
            for alias_kind in ("hardlink", "symlink"):
                for output_kind in ("json", "markdown"):
                    with self.subTest(argument=argument, alias_kind=alias_kind, output_kind=output_kind), tempfile.TemporaryDirectory() as directory:
                        root = Path(directory)
                        source = root / source_fixture.name
                        source.write_bytes(source_fixture.read_bytes())
                        alias = root / f"alias-{source_fixture.name}"
                        if alias_kind == "hardlink":
                            alias.hardlink_to(source)
                        else:
                            alias.symlink_to(source)
                        before = hashlib.sha256(source.read_bytes()).hexdigest()
                        output_json = alias if output_kind == "json" else root / "qualification.json"
                        output_markdown = alias if output_kind == "markdown" else root / "qualification.md"

                        result = subprocess.run(
                            self._command(output_json, output_markdown) + [argument, str(source)],
                            check=False,
                            capture_output=True,
                            text=True,
                        )

                        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                        self.assertIn("status=FAIL", result.stdout)
                        self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), before)
                        other_output = output_markdown if output_kind == "json" else output_json
                        self.assertFalse(other_output.exists() and not other_output.is_symlink())

    def test_model_file_input_is_protected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "model.xml"
            model.write_bytes(b"MODEL_BYTES")
            before = hashlib.sha256(model.read_bytes()).hexdigest()
            output = model
            markdown = root / "qualification.md"

            result = subprocess.run(
                self._command(output, markdown) + ["--model", str(model), "--model-profile", str(MODEL_PROFILE)],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            self.assertEqual(hashlib.sha256(model.read_bytes()).hexdigest(), before)
            self.assertFalse(markdown.exists())

    def test_model_bin_file_input_is_protected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            binary = root / "model.bin"
            binary.write_bytes(b"MODEL_BIN_BYTES")
            before = hashlib.sha256(binary.read_bytes()).hexdigest()
            markdown = root / "qualification.md"

            result = subprocess.run(
                self._command(binary, markdown) + ["--model-bin", str(binary)],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            self.assertEqual(hashlib.sha256(binary.read_bytes()).hexdigest(), before)
            self.assertFalse(markdown.exists())

    def test_annotated_directory_and_evidence_bundle_are_protected_by_containment(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            annotated = root / "annotated"
            annotated.mkdir()
            bundle = root / "bundle"
            bundle.mkdir()
            for source_dir, output in (
                (annotated, annotated / "qualification.json"),
                (bundle, bundle / "qualification.json"),
            ):
                markdown = root / f"{source_dir.name}.md"
                result = subprocess.run(
                    self._command(output, markdown) + [
                        "--annotated-dir" if source_dir == annotated else "--evidence-bundle",
                        str(source_dir),
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("status=FAIL", result.stdout)
                self.assertFalse(output.exists())
                self.assertFalse(markdown.exists())

    def test_manifest_parent_is_protected_when_bundle_root_is_implicit(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            bundle = root / "bundle"
            bundle.mkdir()
            manifest = bundle / "manifest.json"
            manifest.write_text("{}\n", encoding="utf-8")
            output = bundle / "qualification.json"
            markdown = root / "qualification.md"

            result = subprocess.run(
                self._command(output, markdown) + ["--manifest", str(manifest)],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            self.assertEqual(manifest.read_text(encoding="utf-8"), "{}\n")
            self.assertFalse(markdown.exists())

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
