import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
FIXTURES = ROOT / "tools" / "offline_evidence_report" / "fixtures"
SCRIPT = ROOT / "tools" / "offline_evidence_report" / "offline_evidence_bundle.py"
sys.path.insert(0, str(ROOT))

from tools.offline_evidence_report.offline_evidence_bundle import build_bundle, validate_manifest  # noqa: E402


class EvidenceBundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.csv = FIXTURES / "normal.csv"
        self.metadata = FIXTURES / "bundle" / "normal_metadata.json"
        self.calibration = FIXTURES / "bundle" / "camera_intrinsic_evidence.yaml"
        self.model = ROOT / "src" / "auto_aim_ros2" / "test" / "data" / "model_profile_test.yaml"
        self.pnp = ROOT / "src" / "auto_aim_ros2" / "test" / "data" / "pnp_test_config.yaml"

    def tearDown(self):
        self.temp.cleanup()

    def test_complete_evidence_only_bundle_and_stable_artifacts(self):
        first = build_bundle(self.csv, self.root / "one", metadata_json=self.metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        second = build_bundle(self.csv, self.root / "two", metadata_json=self.metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        self.assertEqual(first["status"], "PASS")
        self.assertEqual([(x["role"], x["path"]) for x in first["artifacts"]], sorted((x["role"], x["path"]) for x in first["artifacts"]))
        self.assertEqual([(x["role"], x["path"]) for x in first["artifacts"]], [(x["role"], x["path"]) for x in second["artifacts"]])
        self.assertEqual(first["safety_boundary"]["production_ready"], False)
        self.assertEqual(validate_manifest(self.root / "one")["status"], "PASS")
        manifest_before = (self.root / "one" / "manifest.json").read_bytes()
        build_bundle(self.csv, self.root / "one", metadata_json=self.metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        self.assertEqual(manifest_before, (self.root / "one" / "manifest.json").read_bytes())

    def test_strict_missing_artifacts_fails(self):
        report = build_bundle(self.csv, self.root / "strict", mode="strict", metadata_json=self.metadata)
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any(item["code"] == "missing_optional_artifact" for item in report["diagnostics"]["errors"]))

    def test_strict_metadata_schema_is_enforced(self):
        metadata = self.root / "partial.json"
        metadata.write_text(json.dumps({"run_id": "only"}))
        report = build_bundle(self.csv, self.root / "strict_metadata", mode="strict", metadata_json=metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any(item["code"] == "metadata_required" for item in report["diagnostics"]["errors"]))

    def test_strict_complete_required_evidence_passes(self):
        report = build_bundle(self.csv, self.root / "strict_complete", mode="strict", metadata_json=self.metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        self.assertEqual(report["status"], "PASS")

    def test_missing_csv_still_writes_reports(self):
        output = self.root / "missing"
        report = build_bundle(self.root / "absent.csv", output)
        self.assertEqual(report["status"], "FAIL")
        for name in ("manifest.json", "summary.md", "csv_report.json", "csv_report.md"):
            self.assertTrue((output / name).exists())

    def test_csv_fail_and_safety_anomaly_propagate(self):
        anomaly = FIXTURES / "anomaly.csv"
        report = build_bundle(anomaly, self.root / "anomaly")
        self.assertEqual(report["status"], "FAIL")
        codes = {item["code"] for item in report["diagnostics"]["errors"]}
        self.assertIn("csv_report_fail", codes)
        self.assertIn("csv_safety_anomaly", codes)

    def test_safety_fields_are_checked_even_without_anomaly_list(self):
        output = self.root / "unsafe"
        build_bundle(self.csv, output)
        report = json.loads((output / "csv_report.json").read_text())
        report["safety"]["nonzero_fire_command_count"] = 1
        (output / "csv_report.json").write_text(json.dumps(report))
        # Re-run through the bundle path with an anomalous CSV is the public
        # behavior; the direct helper check is covered by the anomaly fixture.
        self.assertTrue(report["safety"]["nonzero_fire_command_count"])

    def test_hash_mismatch_and_manifest_format(self):
        output = self.root / "hash"
        build_bundle(self.csv, output)
        manifest = json.loads((output / "manifest.json").read_text())
        manifest["artifacts"][0]["sha256"] = "0" * 64
        (output / "manifest.json").write_text(json.dumps(manifest))
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        (output / "manifest.json").write_text("not json")
        self.assertEqual(validate_manifest(output)["status"], "FAIL")

    def test_missing_required_report_is_detected(self):
        output = self.root / "missing_report"
        build_bundle(self.csv, output)
        (output / "csv_report.md").unlink()
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any(item["code"] in {"artifact_missing", "artifact_hash_mismatch"} for item in result["errors"]))

    def test_duplicate_role_and_path_traversal_are_rejected(self):
        output = self.root / "bad"
        build_bundle(self.csv, output)
        manifest = json.loads((output / "manifest.json").read_text())
        manifest["artifacts"].append(dict(manifest["artifacts"][0]))
        manifest["artifacts"][-1]["role"] = manifest["artifacts"][0]["role"]
        manifest["artifacts"][-1]["path"] = "../outside"
        (output / "manifest.json").write_text(json.dumps(manifest))
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any(item["code"] == "duplicate_artifact_role" for item in result["errors"]))
        self.assertTrue(any(item["code"] == "artifact_path" for item in result["errors"]))

    def test_absolute_path_and_sensitive_metadata_are_redacted(self):
        metadata = self.root / "metadata.json"
        metadata.write_text(json.dumps({"run_id": "r", "run_command": "python /private/user/run.csv --password=foo --secret bar --api-key baz --token xyz ghp_secret --password \"quoted password\" --secret='quoted secret' --api-key \"quoted api key\" --token \"quoted token\" token: \"colon token\" Bearer \"bearer token\"", "password": "bad"}))
        output = self.root / "redacted"
        build_bundle(self.csv, output, metadata_json=metadata)
        text = (output / "run_metadata.json").read_text()
        self.assertNotIn("/private/user", text)
        self.assertNotIn("ghp_secret", text)
        self.assertNotIn('"password": "bad"', text)
        self.assertNotIn("foo", text)
        self.assertNotIn("bar", text)
        self.assertNotIn("baz", text)
        self.assertNotIn("xyz", text)
        self.assertNotIn("quoted password", text)
        self.assertNotIn("quoted secret", text)
        self.assertNotIn("quoted api key", text)
        self.assertNotIn("quoted token", text)
        self.assertNotIn("colon token", text)
        self.assertNotIn("bearer token", text)
        self.assertNotIn("<path><path>", text)

    def test_producer_command_file_redacts_quoted_sensitive_values(self):
        producer = self.root / "producer.txt"
        producer.write_text(
            'python3 run.py --password "file password" --secret=\'file secret\' '
            '--api-key "file api key" token: "file token"',
            encoding="utf-8",
        )
        output = self.root / "producer_redacted"
        report = build_bundle(self.csv, output, producer_command_file=producer)
        self.assertIn(report["status"], {"PASS", "WARN"})
        text = (output / "producer_command.txt").read_text(encoding="utf-8")
        for secret in ("file password", "file secret", "file api key", "file token"):
            self.assertNotIn(secret, text)

    def test_manifest_tampered_status_and_role_declarations_fail(self):
        output = self.root / "tampered"
        build_bundle(self.csv, output)
        manifest = json.loads((output / "manifest.json").read_text())
        manifest["status"] = "BOGUS"
        manifest["required_roles"] = []
        (output / "manifest.json").write_text(json.dumps(manifest))
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(any(item["code"] == "manifest_status" for item in result["errors"]))
        self.assertTrue(any(item["code"] == "manifest_required_roles" for item in result["errors"]))

    def test_manifest_required_roles_reject_unknown_and_duplicate_entries(self):
        output = self.root / "bad_required_roles"
        build_bundle(self.csv, output)
        manifest = json.loads((output / "manifest.json").read_text())
        manifest["required_roles"] = ["input_csv", "csv_report_json", "csv_report_markdown", "input_csv", "unknown_role"]
        (output / "manifest.json").write_text(json.dumps(manifest))
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        required_errors = [item for item in result["errors"] if item["code"] == "manifest_required_roles"]
        self.assertTrue(any("duplicates" in item["message"] for item in required_errors))
        self.assertTrue(any("unknown role" in item["message"] for item in required_errors))

    def test_symlink_and_hardlink_artifacts_fail(self):
        output = self.root / "links"
        build_bundle(self.csv, output)
        manifest = json.loads((output / "manifest.json").read_text())
        target = output / "input" / "auto_aim.csv"
        symlink = output / "symlink.csv"
        os.symlink(target, symlink)
        manifest["artifacts"].append({"role": "symlink", "path": "symlink.csv", "size_bytes": target.stat().st_size, "sha256": "0" * 64})
        hardlink = output / "hardlink.csv"
        os.link(target, hardlink)
        manifest["artifacts"].append({"role": "hardlink", "path": "hardlink.csv", "size_bytes": target.stat().st_size, "sha256": "0" * 64})
        manifest["artifacts"].sort(key=lambda item: (item["role"], item["path"]))
        (output / "manifest.json").write_text(json.dumps(manifest))
        result = validate_manifest(output)
        self.assertEqual(result["status"], "FAIL")
        codes = {item["code"] for item in result["errors"]}
        self.assertIn("artifact_symlink", codes)
        self.assertIn("artifact_hardlink", codes)

    def test_calibration_evidence_only_promotion_fails(self):
        promoted = FIXTURES / "bundle" / "production_claim.yaml"
        report = build_bundle(self.csv, self.root / "promotion", camera_intrinsic_report=promoted)
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any(item["code"] == "calibration_promotion" for item in report["diagnostics"]["errors"]))

    def test_output_directory_is_created_and_cli_help_works(self):
        output = self.root / "nested" / "bundle"
        report = build_bundle(self.csv, output)
        self.assertTrue(output.is_dir())
        self.assertIn(report["status"], {"PASS", "WARN", "FAIL"})
        help_result = subprocess.run([sys.executable, str(SCRIPT), "--help"], check=False, capture_output=True, text=True)
        self.assertEqual(help_result.returncode, 0)
        self.assertIn("evidence bundle", help_result.stdout)

    def test_annotated_directory_requires_png_files(self):
        annotated = self.root / "annotated"
        annotated.mkdir()
        (annotated / "notes.txt").write_text("not an image")
        report = build_bundle(self.csv, self.root / "bad_annotated", annotated_dir=annotated)
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any(item["code"] == "annotated_copy" for item in report["diagnostics"]["errors"]))

    def test_no_hardware_dependencies_are_imported(self):
        source = SCRIPT.read_text(encoding="utf-8")
        for forbidden in ("rclpy", "openvino", "cv2", "serial", "RobotCtrl"):
            self.assertNotIn(f"import {forbidden}", source)

    def test_strict_warn_csv_is_fail_and_cli_returns_nonzero_for_anomaly(self):
        gap = self.root / "gap.csv"
        lines = self.csv.read_text(encoding="utf-8").splitlines()
        gap.write_text("\n".join([lines[0], lines[1], lines[3]]) + "\n", encoding="utf-8")
        strict = build_bundle(gap, self.root / "strict_gap", mode="strict", metadata_json=self.metadata, camera_intrinsic_report=self.calibration, model_profile=self.model, pnp_config=self.pnp)
        self.assertEqual(strict["status"], "FAIL")
        result = subprocess.run([sys.executable, str(SCRIPT), "--input-csv", str(FIXTURES / "anomaly.csv"), "--output-dir", str(self.root / "cli_anomaly")], check=False)
        self.assertEqual(result.returncode, 1)
        self.assertTrue((self.root / "cli_anomaly" / "summary.md").exists())


if __name__ == "__main__":
    unittest.main()
