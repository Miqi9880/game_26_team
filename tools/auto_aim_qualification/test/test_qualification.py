import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

from tools.auto_aim_qualification.model_profile_audit import audit_model_profile, audit_pnp_config
from tools.auto_aim_qualification.offline_qualification import qualify_offline, render_markdown


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.model_profile = ROOT / "src/auto_aim_ros2/test/data/model_profile_test.yaml"
        self.pnp_config = ROOT / "src/auto_aim_ros2/test/data/pnp_test_config.yaml"
        self.csv = ROOT / "tools/offline_evidence_report/fixtures/normal.csv"
        self.metadata = ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"

    def test_test_only_requires_explicit_opt_in(self):
        result = audit_model_profile(self.model_profile)
        self.assertEqual(result.status, "FAIL")
        self.assertIn("test_only_not_allowed", {item.code for item in result.findings})
        result = audit_pnp_config(self.pnp_config)
        self.assertEqual(result.status, "FAIL")
        self.assertIn("test_only_not_allowed", {item.code for item in result.findings})

    def test_evidence_only_fixture_is_warn_and_never_production(self):
        report = qualify_offline(
            model_profile=self.model_profile,
            pnp_config=self.pnp_config,
            input_csv=self.csv,
            metadata_json=self.metadata,
            allow_test_only=True,
        )
        self.assertEqual(report["status"], "WARN")
        self.assertFalse(report["production_ready"])
        self.assertFalse(report["hardware_validation"])
        self.assertTrue(any(item["code"] == "test_only_evidence" for item in report["diagnostics"]["warnings"]))

    def test_strict_fixture_fails_closed(self):
        report = qualify_offline(
            model_profile=self.model_profile,
            pnp_config=self.pnp_config,
            input_csv=self.csv,
            mode="strict",
            allow_test_only=True,
            metadata_json=self.metadata,
        )
        self.assertEqual(report["status"], "FAIL")
        codes = {item["code"] for item in report["diagnostics"]["errors"]}
        self.assertIn("strict_requires_production_model", codes)
        self.assertIn("strict_requires_production_pnp", codes)

    def test_model_hash_and_runtime_path_are_checked(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_text("MODEL_BYTES", encoding="utf-8")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8").replace("profile: test_only", "profile: production")
                .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}")
                .replace("version: legacy-reference-v1", "version: reviewed-v1")
                .replace("source: sp_vision_25_assets_yolov5_xml", "source: reviewed-model-source\n  sha256: " + digest),
                encoding="utf-8",
            )
            result = audit_model_profile(profile, model_path=artifact, mode="strict")
            self.assertNotIn("model_path_mismatch", {item.code for item in result.findings})
            self.assertNotIn("model_hash_mismatch", {item.code for item in result.findings})
            artifact.write_text("CHANGED", encoding="utf-8")
            changed = audit_model_profile(profile, model_path=artifact, mode="strict")
            self.assertIn("model_hash_mismatch", {item.code for item in changed.findings})

    def test_pnp_resolution_and_corner_order_fail(self):
        import yaml

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.yaml"
            value = yaml.safe_load(self.pnp_config.read_text(encoding="utf-8"))
            value["camera"]["image_width"] = 1
            value["armor_geometry"]["corner_order"] = ["top_left", "top_left", "bottom_right", "bottom_left"]
            path.write_text(yaml.safe_dump(value), encoding="utf-8")
            result = audit_pnp_config(path, allow_test_only=True, expected_image_size=(1440, 1080))
            codes = {item.code for item in result.findings}
            self.assertIn("pnp_resolution_mismatch", codes)
            self.assertIn("corner_order", codes)

    def test_model_contract_shape_layout_color_output_and_mapping_fail_closed(self):
        import yaml

        mutations = (
            ("input_shape", lambda value: value["input"].__setitem__("shape", [1, 4, 640, 640]), "input_shape"),
            ("input_layout", lambda value: value["input"].__setitem__("layout", "NHWC"), "input_layout"),
            ("input_color", lambda value: value["input"].__setitem__("model_color_order", "BGR"), "color_order"),
            ("output_shape", lambda value: value["output"].__setitem__("shape", [1, 2, 3]), "tensor_range"),
            ("keypoint_order", lambda value: value["postprocess"].__setitem__("keypoint_order", [0, 0, 2, 3]), "keypoint_order"),
            ("mapping", lambda value: value["semantics"]["class_to_armor_type"].pop(8), "class_mapping_incomplete"),
        )
        for name, mutate, expected_code in mutations:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "bad.yaml"
                value = yaml.safe_load(self.model_profile.read_text(encoding="utf-8"))
                mutate(value)
                path.write_text(yaml.safe_dump(value), encoding="utf-8")
                result = audit_model_profile(path, allow_test_only=True)
                self.assertIn(expected_code, {item.code for item in result.findings})

    def test_pnp_missing_intrinsics_geometry_and_production_extrinsic_fail(self):
        import yaml

        mutations = (
            ("camera_matrix", lambda value: value["camera"].pop("camera_matrix"), "matrix_shape"),
            ("small_geometry", lambda value: value["armor_geometry"].pop("small"), "armor_geometry_missing"),
            ("production_extrinsic", lambda value: (value.__setitem__("profile", "production"), value["camera_to_gimbal"].__setitem__("configured", False)), "production_extrinsic_missing"),
        )
        for name, mutate, expected_code in mutations:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / "bad.yaml"
                value = yaml.safe_load(self.pnp_config.read_text(encoding="utf-8"))
                mutate(value)
                path.write_text(yaml.safe_dump(value), encoding="utf-8")
                result = audit_pnp_config(path, allow_test_only=True)
                self.assertIn(expected_code, {item.code for item in result.findings})

    def test_missing_csv_is_explicitly_unavailable(self):
        report = qualify_offline(
            model_profile=self.model_profile,
            pnp_config=self.pnp_config,
            allow_test_only=True,
        )
        self.assertEqual(report["status"], "WARN")
        self.assertTrue(any(item["code"] == "pipeline_execution_unavailable" for item in report["diagnostics"]["warnings"]))
        self.assertFalse(report["pipeline"]["execution_claimed"])

    def test_malformed_metadata_is_fail_closed_in_strict_mode(self):
        with tempfile.TemporaryDirectory() as directory:
            metadata = Path(directory) / "metadata.json"
            metadata.write_text("not-json", encoding="utf-8")
            report = qualify_offline(
                model_profile=self.model_profile,
                pnp_config=self.pnp_config,
                metadata_json=metadata,
                mode="strict",
            )
        self.assertEqual(report["status"], "FAIL")
        self.assertTrue(any(item["code"] == "metadata_error" for item in report["diagnostics"]["errors"]))

    def test_markdown_and_json_are_stable_and_redacted(self):
        report = qualify_offline(
            model_profile=self.model_profile,
            pnp_config=self.pnp_config,
            input_csv=self.csv,
            allow_test_only=True,
            metadata_json=self.metadata,
        )
        text = render_markdown(report)
        self.assertIn("production_ready: false", text)
        encoded = json.dumps(report, ensure_ascii=False)
        self.assertNotIn("/home/ubuntu22", encoded)

    def test_unified_csv_fail_and_manifest_status_are_propagated(self):
        anomaly = ROOT / "tools/offline_evidence_report/fixtures/anomaly.csv"
        with tempfile.TemporaryDirectory() as directory:
            report = qualify_offline(
                model_profile=self.model_profile,
                pnp_config=self.pnp_config,
                input_csv=anomaly,
                evidence_bundle=Path(directory) / "bundle",
                allow_test_only=True,
            )
        self.assertEqual(report["status"], "FAIL")
        self.assertEqual(report["csv_report_status"], "FAIL")
        self.assertTrue(any(item["code"] == "csv_report_fail" for item in report["diagnostics"]["errors"]))

    def test_cli_rejects_output_alias_without_overwriting_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "input.csv"
            original = self.csv.read_bytes()
            output.write_bytes(original)
            command = [
                sys.executable,
                str(ROOT / "tools/auto_aim_qualification/auto_aim_qualification.py"),
                "--allow-test-only",
                "--model-profile", str(self.model_profile),
                "--pnp-config", str(self.pnp_config),
                "--input-csv", str(output),
                "--output-json", str(output),
                "--output-markdown", str(Path(directory) / "report.md"),
            ]
            result = subprocess.run(command, check=False, capture_output=True, text=True)
            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertEqual(output.read_bytes(), original)


if __name__ == "__main__":
    unittest.main()
