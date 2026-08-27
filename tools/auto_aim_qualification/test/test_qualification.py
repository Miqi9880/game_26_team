import hashlib
import json
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[3]

from tools.auto_aim_qualification.model_profile_audit import audit_model_profile, audit_pnp_config
from tools.auto_aim_qualification.offline_qualification import qualify_offline, render_markdown


class QualificationTests(unittest.TestCase):
    def setUp(self):
        self.model_profile = ROOT / "src/auto_aim_ros2/test/data/model_profile_test.yaml"
        self.pnp_config = ROOT / "src/auto_aim_ros2/test/data/pnp_test_config.yaml"
        self.csv = ROOT / "tools/offline_evidence_report/fixtures/normal.csv"
        self.metadata = ROOT / "tools/offline_evidence_report/fixtures/bundle/normal_metadata.json"

    def _schema_v2_profile(
        self,
        root: Path,
        xml: Path,
        binary: Path,
        *,
        profile_kind: str = "test_only",
        xml_digest: object | None = None,
        bin_digest: object | None = None,
    ) -> Path:
        """Build a reviewed two-member IR manifest around temporary bytes."""

        import yaml

        value = yaml.safe_load(self.model_profile.read_text(encoding="utf-8"))
        value["schema_version"] = 2
        value["profile"] = profile_kind
        model = value["model"]
        model.pop("path", None)
        for key in ("sha256", "sha256sum", "artifact_sha256", "hash"):
            model.pop(key, None)
        model.update(
            {
                "id": "reviewed_openvino_ir",
                "source": "reviewed-model-source",
                "version": "reviewed-v2",
                "format": "openvino_ir",
                "artifacts": {
                    "xml": {
                        "path": str(xml),
                        "sha256": hashlib.sha256(xml.read_bytes()).hexdigest() if xml_digest is None else xml_digest,
                    },
                    "bin": {
                        "path": str(binary),
                        "sha256": hashlib.sha256(binary.read_bytes()).hexdigest() if bin_digest is None else bin_digest,
                    },
                },
            }
        )
        if profile_kind == "production":
            value["semantics"]["color_id_to_name"] = [f"color_{index}" for index in range(4)]
            value["semantics"]["armor_class_names"] = [f"armor_{index}" for index in range(9)]
        profile = root / "profile-v2.yaml"
        profile.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
        return profile

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
            xml = root / "model.xml"
            binary = root / "model.bin"
            xml.write_text("MODEL_XML", encoding="utf-8")
            binary.write_bytes(b"MODEL_BIN")
            profile = self._schema_v2_profile(root, xml, binary, profile_kind="production")
            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value={"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"},
            ):
                result = audit_model_profile(profile, model_path=xml, model_bin_path=binary, mode="strict")
            codes = {item.code for item in result.findings}
            self.assertNotIn("model_xml_path_mismatch", codes)
            self.assertNotIn("model_bin_path_mismatch", codes)
            self.assertNotIn("model_xml_hash_mismatch", codes)
            self.assertNotIn("model_bin_hash_mismatch", codes)

            binary.write_bytes(b"CHANGED_BIN")
            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value={"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"},
            ) as inspect:
                changed = audit_model_profile(profile, model_path=xml, model_bin_path=binary, mode="strict")
            self.assertIn("model_bin_hash_mismatch", {item.code for item in changed.findings})
            self.assertIn("model_artifact_unverified", {item.code for item in changed.findings})
            self.assertEqual(changed.values["runtime_contract"]["status"], "model_artifact_unverified")
            inspect.assert_not_called()

    def _artifact_profile(self, root: Path, artifact: Path) -> Path:
        digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
        profile = root / "profile.yaml"
        profile.write_text(
            self.model_profile.read_text(encoding="utf-8")
            .replace("model:\n", "model:\n  sha256: " + digest + "\n")
            .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"),
            encoding="utf-8",
        )
        return profile

    def test_missing_artifact_and_openvino_runtime_are_explicitly_unavailable(self):
        missing = Path(tempfile.gettempdir()) / "game26-does-not-exist-model.xml"
        result = audit_model_profile(self.model_profile, model_path=missing, allow_test_only=True)
        codes = {item.code for item in result.findings}
        self.assertEqual(result.status, "WARN")
        self.assertIn("model_artifact_unavailable", codes)
        self.assertEqual(result.values["runtime_contract"]["status"], "model_artifact_unavailable")
        self.assertFalse(result.values["model_artifact_exists"])

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"not-an-openvino-model")
            profile = self._artifact_profile(root, artifact)
            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value={"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"},
            ):
                runtime_result = audit_model_profile(profile, model_path=artifact, allow_test_only=True)
            self.assertEqual(runtime_result.status, "WARN")
            self.assertIn("runtime_unavailable", {item.code for item in runtime_result.findings})
            self.assertEqual(runtime_result.values["runtime_contract"]["status"], "unavailable")
            self.assertEqual(runtime_result.values["runtime_contract"]["reason"], "ImportError")

    def test_schema_v2_xml_bin_manifest_is_audited_and_passed_to_openvino(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "reviewed.xml"
            binary = root / "reviewed.bin"
            xml.write_bytes(b"REVIEWED_XML")
            binary.write_bytes(b"REVIEWED_BIN")
            profile = self._schema_v2_profile(root, xml, binary)
            unavailable = {"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"}

            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value=unavailable,
            ) as inspect:
                result = audit_model_profile(
                    profile,
                    model_path=xml,
                    model_bin_path=binary,
                    allow_test_only=True,
                )
            inspect.assert_called_once_with(xml, binary)
            self.assertEqual(result.status, "WARN")
            artifacts = result.values["model_artifacts"]
            self.assertEqual(set(artifacts), {"xml", "bin"})
            self.assertTrue(artifacts["xml"]["exists"])
            self.assertTrue(artifacts["bin"]["exists"])
            self.assertTrue(artifacts["xml"]["sha256_matches_profile"])
            self.assertTrue(artifacts["bin"]["sha256_matches_profile"])
            self.assertEqual(artifacts["xml"]["runtime_path"], xml.name)
            self.assertEqual(artifacts["bin"]["runtime_path"], binary.name)

            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value=unavailable,
            ):
                report = qualify_offline(
                    model_profile=profile,
                    model=xml,
                    model_bin=binary,
                    pnp_config=self.pnp_config,
                    input_csv=self.csv,
                    allow_test_only=True,
                )
            self.assertEqual(report["model_artifacts"]["xml"]["actual_sha256"], hashlib.sha256(xml.read_bytes()).hexdigest())
            self.assertEqual(report["model_artifacts"]["bin"]["actual_sha256"], hashlib.sha256(binary.read_bytes()).hexdigest())
            self.assertEqual(report["model_bin_file"], binary.name)
            self.assertEqual(report["model_bin_sha256"], hashlib.sha256(binary.read_bytes()).hexdigest())
            markdown = render_markdown(report)
            self.assertIn("- XML artifact:", markdown)
            self.assertIn("- BIN artifact:", markdown)

    def test_openvino_runtime_reads_schema_v2_with_explicit_xml_and_bin(self):
        """Do not regress to OpenVINO's implicit same-stem BIN discovery."""

        from tools.auto_aim_qualification import model_profile_audit

        calls: list[tuple[str, ...]] = []

        class FakeModel:
            inputs: list[object] = []
            outputs: list[object] = []

        class FakeCore:
            def read_model(self, *paths: str) -> FakeModel:
                calls.append(paths)
                return FakeModel()

        fake_openvino = types.ModuleType("openvino")
        fake_openvino.Core = FakeCore
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "reviewed.xml"
            binary = root / "reviewed.bin"
            xml.write_bytes(b"XML")
            binary.write_bytes(b"BIN")
            with mock.patch.dict(sys.modules, {"openvino": fake_openvino}):
                observed = model_profile_audit._inspect_openvino_runtime(xml, binary)
        self.assertEqual(observed["status"], "checked")
        self.assertEqual(calls, [(str(xml), str(binary))])

    def test_runtime_layout_falls_back_to_openvino_output_node(self):
        from tools.auto_aim_qualification.model_profile_audit import _runtime_layout

        class FakeNode:
            def get_layout(self):
                return "[N,C,H,W]"

        class FakePort:
            def get_node(self):
                return FakeNode()

        self.assertEqual(_runtime_layout(FakePort()), "NCHW")

    def test_schema_v2_missing_bin_is_explicit_for_test_only_and_production(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "reviewed.xml"
            missing_binary = root / "missing.bin"
            xml.write_bytes(b"REVIEWED_XML")
            profile = self._schema_v2_profile(
                root,
                xml,
                missing_binary,
                bin_digest=hashlib.sha256(b"EXPECTED_BIN").hexdigest(),
            )
            test_only = audit_model_profile(
                profile,
                model_path=xml,
                model_bin_path=missing_binary,
                allow_test_only=True,
            )
            self.assertEqual(test_only.status, "WARN")
            test_codes = {item.code for item in test_only.findings}
            self.assertIn("model_bin_artifact_unavailable", test_codes)
            self.assertIn("model_artifact_unavailable", test_codes)

            strict_test_only = audit_model_profile(
                profile,
                model_path=xml,
                model_bin_path=missing_binary,
                mode="strict",
                allow_test_only=True,
            )
            self.assertEqual(strict_test_only.status, "FAIL")
            self.assertIn("model_bin_artifact_unavailable", {item.code for item in strict_test_only.findings})

            production_profile = self._schema_v2_profile(
                root,
                xml,
                missing_binary,
                profile_kind="production",
                bin_digest=hashlib.sha256(b"EXPECTED_BIN").hexdigest(),
            )
            production = audit_model_profile(
                production_profile,
                model_path=xml,
                model_bin_path=missing_binary,
            )
            self.assertEqual(production.status, "FAIL")
            production_codes = {item.code for item in production.findings}
            self.assertIn("model_bin_artifact_unavailable", production_codes)
            self.assertIn("model_artifact_unavailable", production_codes)

    def test_schema_v2_bin_digest_and_paths_fail_closed(self):
        import yaml

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "reviewed.xml"
            binary = root / "reviewed.bin"
            alternate_xml = root / "alternate.xml"
            alternate_binary = root / "alternate.bin"
            xml.write_bytes(b"REVIEWED_XML")
            binary.write_bytes(b"REVIEWED_BIN")
            alternate_xml.write_bytes(xml.read_bytes())
            alternate_binary.write_bytes(binary.read_bytes())
            profile = self._schema_v2_profile(root, xml, binary, profile_kind="production")

            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value={"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"},
            ):
                mismatch = audit_model_profile(
                    profile,
                    model_path=alternate_xml,
                    model_bin_path=alternate_binary,
                )
            mismatch_codes = {item.code for item in mismatch.findings}
            self.assertIn("model_xml_path_mismatch", mismatch_codes)
            self.assertIn("model_bin_path_mismatch", mismatch_codes)

            external_bin_hash = "0" * 64
            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value={"status": "unavailable", "runtime": "openvino", "available": False, "reason": "ImportError"},
            ):
                external_mismatch = audit_model_profile(
                    profile,
                    model_path=xml,
                    model_bin_path=binary,
                    expected_model_bin_sha256=external_bin_hash,
                )
            external_codes = {item.code for item in external_mismatch.findings}
            self.assertIn("model_bin_hash_declaration_mismatch", external_codes)
            self.assertIn("model_bin_hash_mismatch", external_codes)

            value = yaml.safe_load(profile.read_text(encoding="utf-8"))
            value["model"]["artifacts"]["bin"].pop("sha256")
            profile.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
            missing_digest = audit_model_profile(profile, model_path=xml, model_bin_path=binary, allow_test_only=True)
            self.assertIn("model_bin_hash_missing", {item.code for item in missing_digest.findings})

            value["model"]["artifacts"]["bin"]["sha256"] = "not-a-sha256"
            profile.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
            invalid_digest = audit_model_profile(profile, model_path=xml, model_bin_path=binary, allow_test_only=True)
            self.assertIn("model_bin_hash_invalid", {item.code for item in invalid_digest.findings})

    def test_schema_v1_production_is_rejected(self):
        import yaml

        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory) / "legacy-production.yaml"
            value = yaml.safe_load(self.model_profile.read_text(encoding="utf-8"))
            value["profile"] = "production"
            profile.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
            result = audit_model_profile(profile)
            self.assertIn("production_schema_v1", {item.code for item in result.findings})

    def test_schema_v1_cannot_silently_accept_bin_inputs(self):
        result = audit_model_profile(
            self.model_profile,
            model_bin_path=Path(tempfile.gettempdir()) / "unexpected.bin",
            expected_model_bin_sha256="",
            allow_test_only=True,
        )
        codes = {item.code for item in result.findings}
        self.assertIn("model_bin_undeclared", codes)
        self.assertIn("model_bin_hash_external_invalid", codes)

    def test_runtime_shape_type_and_dynamic_contract_mismatches_fail_closed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"test-only-artifact")
            profile = self._artifact_profile(root, artifact)
            cases = (
                (
                    {
                        "status": "checked", "runtime": "openvino", "available": True,
                        "input_count": 1, "output_count": 1,
                        "input_shape": [1, 3, 320, 320], "input_shape_error": None,
                        "output_shape": [1, 25200, 22], "output_shape_error": None,
                        "input_element_type": "f32", "output_element_type": "f32",
                        "input_layout": "NCHW", "output_layout": "NRC",
                    },
                    "runtime_input_shape_mismatch",
                ),
                (
                    {
                        "status": "checked", "runtime": "openvino", "available": True,
                        "input_count": 1, "output_count": 1,
                        "input_shape": None, "input_shape_error": "dynamic",
                        "output_shape": [1, 25200, 22], "output_shape_error": None,
                        "input_element_type": "f32", "output_element_type": "f16",
                        "input_layout": "NCHW", "output_layout": "NRC",
                    },
                    "runtime_dynamic_shape",
                ),
                (
                    {
                        "status": "checked", "runtime": "openvino", "available": True,
                        "input_count": 2, "output_count": 1,
                    },
                    "runtime_input_count",
                ),
                (
                    {
                        "status": "checked", "runtime": "openvino", "available": True,
                        "input_count": 1, "output_count": 1,
                        "input_shape": [1, 3, 640, 640], "input_shape_error": None,
                        "output_shape": [1, 25200, 22], "output_shape_error": None,
                        "input_element_type": "f32", "output_element_type": "f32",
                        "input_layout": "NHWC", "output_layout": "NRC",
                    },
                    "runtime_input_layout_mismatch",
                ),
                (
                    {
                        "status": "checked", "runtime": "openvino", "available": True,
                        "input_count": 1, "output_count": 1,
                        "input_shape": [1, 3, 640, 640], "input_shape_error": None,
                        "output_shape": [1, 25200, 22], "output_shape_error": None,
                        "input_element_type": "f32", "output_element_type": "f32",
                        "input_layout": "NCHW", "output_layout": "NCHW",
                    },
                    "runtime_output_layout_mismatch",
                ),
            )
            for observed, expected in cases:
                with self.subTest(expected=expected):
                    with mock.patch(
                        "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                        return_value=observed,
                    ):
                        result = audit_model_profile(profile, model_path=artifact, allow_test_only=True)
                    self.assertEqual(result.status, "FAIL")
                    self.assertIn(expected, {item.code for item in result.findings})

    def test_runtime_layout_unavailable_is_warn_for_evidence_and_fail_for_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"test-only-artifact")
            profile = self._artifact_profile(root, artifact)
            observed = {
                "status": "checked", "runtime": "openvino", "available": True,
                "input_count": 1, "output_count": 1,
                "input_shape": [1, 3, 640, 640], "input_shape_error": None,
                "output_shape": [1, 25200, 22], "output_shape_error": None,
                "input_element_type": "f32", "output_element_type": "f32",
                "input_layout": None, "output_layout": "NRC",
            }
            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value=observed,
            ):
                evidence = audit_model_profile(profile, model_path=artifact, allow_test_only=True)
            self.assertEqual(evidence.status, "WARN")
            self.assertIn("runtime_layout_unavailable", {item.code for item in evidence.findings})

            with mock.patch(
                "tools.auto_aim_qualification.model_profile_audit._inspect_openvino_runtime",
                return_value=observed,
            ):
                strict = audit_model_profile(
                    profile, model_path=artifact, mode="strict", allow_test_only=True
                )
            self.assertEqual(strict.status, "FAIL")
            self.assertIn("runtime_layout_unavailable", {item.code for item in strict.findings})

    def test_report_contains_runtime_contract_preprocessing_and_effective_test_only(self):
        report = qualify_offline(
            model_profile=self.model_profile,
            pnp_config=self.pnp_config,
            input_csv=self.csv,
            metadata_json=self.metadata,
            allow_test_only=True,
        )
        self.assertTrue(report["test_only"])
        self.assertTrue(report["effective_test_only"])
        self.assertFalse(report["model_file_exists"])
        self.assertEqual(report["runtime_status"], "model_artifact_unavailable")
        self.assertEqual(report["preprocessing_contract"]["source_color_order"], "BGR")
        self.assertEqual(report["preprocessing_contract"]["model_color_order"], "RGB")
        self.assertEqual(report["preprocessing_contract"]["normalization"], "divide_255")
        self.assertEqual(report["preprocessing_contract"]["layout"], "NCHW")
        self.assertEqual(report["keypoint_order"], [0, 3, 2, 1])
        self.assertTrue(report["software_preprocessing_evidence"]["software_evidence_only"])
        self.assertFalse(report["software_preprocessing_evidence"]["mcu_raw_hex_fixture"])
        markdown = render_markdown(report)
        self.assertIn("## Model contract evidence", markdown)
        self.assertIn("preprocessing golden: status=`PASS`, software-only=`true`", markdown)
        self.assertIn("may read OpenVINO model metadata", markdown)
        self.assertIn("never compiles a model, runs inference, uses hardware", markdown)

    def test_model_input_and_output_shape_reject_boolean_dimensions(self):
        import yaml

        cases = (
            ("input", [True, 3, 640, 640], "input_shape"),
            ("output", [True, 25200, 22], "output_shape"),
        )
        for section, shape, expected_code in cases:
            with self.subTest(section=section), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / f"boolean-{section}-shape.yaml"
                value = yaml.safe_load(self.model_profile.read_text(encoding="utf-8"))
                value[section]["shape"] = shape
                path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")

                result = audit_model_profile(path, allow_test_only=True)

                self.assertEqual(result.status, "FAIL")
                self.assertIn(expected_code, {item.code for item in result.findings})

    def test_model_class_mapping_rejects_boolean_and_normalized_duplicate_keys(self):
        import yaml

        cases = (
            (
                "boolean-key",
                lambda mapping: (mapping.pop(1), mapping.__setitem__(True, "small")),
                {"class_mapping_key"},
            ),
            (
                "numeric-string-alias",
                lambda mapping: mapping.__setitem__("1", "large"),
                {"class_mapping_key", "class_mapping_duplicate_key"},
            ),
        )
        for name, mutate, expected_codes in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / f"{name}.yaml"
                value = yaml.safe_load(self.model_profile.read_text(encoding="utf-8"))
                mutate(value["semantics"]["class_to_armor_type"])
                path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")

                result = audit_model_profile(path, allow_test_only=True)

                self.assertEqual(result.status, "FAIL")
                codes = {item.code for item in result.findings}
                self.assertTrue(expected_codes <= codes)
                self.assertNotIn("class_to_armor_type", result.values)

    def test_pnp_class_mapping_rejects_boolean_and_normalized_duplicate_keys(self):
        import yaml

        cases = (
            (
                "boolean-key",
                lambda mapping: (mapping.pop(1), mapping.__setitem__(True, "small")),
                {"class_mapping_key"},
            ),
            (
                "numeric-string-alias",
                lambda mapping: mapping.__setitem__("1", "large"),
                {"class_mapping_key", "class_mapping_duplicate_key"},
            ),
        )
        for name, mutate, expected_codes in cases:
            with self.subTest(name=name), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / f"{name}.yaml"
                value = yaml.safe_load(self.pnp_config.read_text(encoding="utf-8"))
                mutate(value["class_to_armor_type"])
                path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")

                result = audit_pnp_config(path, allow_test_only=True)

                self.assertEqual(result.status, "FAIL")
                codes = {item.code for item in result.findings}
                self.assertTrue(expected_codes <= codes)
                self.assertNotIn("class_to_armor_type", result.values)

    def test_external_model_hash_cannot_override_profile_hash(self):
        """Both profile and metadata/CLI hash assertions must be checked."""

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"MODEL_BYTES")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8")
                # Deliberately leave an all-zero digest unquoted.  PyYAML
                # parses it as an integer; the audit must still reject it
                # instead of dropping it and accepting the external hash.
                .replace("model:\n", "model:\n  sha256: " + "0" * 64 + "\n")
                .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"),
                encoding="utf-8",
            )

            result = audit_model_profile(
                profile,
                model_path=artifact,
                expected_model_sha256=digest,
                allow_test_only=True,
            )
            codes = {item.code for item in result.findings}
            self.assertEqual(result.status, "FAIL")
        self.assertIn("model_hash_declaration_mismatch", codes)
        self.assertIn("model_hash_invalid", codes)

    def test_validly_formatted_wrong_profile_hash_still_fails_with_correct_external_hash(self):
        """A correctly formatted but stale profile digest cannot be overridden."""

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"MODEL_BYTES")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8")
                .replace("model:\n", "model:\n  sha256: " + "a" * 64 + "\n")
                .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"),
                encoding="utf-8",
            )

            result = audit_model_profile(
                profile,
                model_path=artifact,
                expected_model_sha256=digest,
                allow_test_only=True,
            )
            codes = {item.code for item in result.findings}
            self.assertEqual(result.status, "FAIL")
            self.assertIn("model_hash_declaration_mismatch", codes)
            self.assertIn("model_hash_mismatch", codes)

    def test_missing_profile_hash_is_not_satisfied_by_external_hash(self):
        """An external hash cannot supply a missing reviewed declaration."""

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"MODEL_BYTES")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8").replace(
                    "path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"
                ),
                encoding="utf-8",
            )

            result = audit_model_profile(
                profile,
                model_path=artifact,
                expected_model_sha256=digest,
                allow_test_only=True,
            )
            self.assertEqual(result.status, "FAIL")
            self.assertIn("model_hash_missing", {item.code for item in result.findings})

    def test_malformed_metadata_model_hash_is_not_silently_ignored(self):
        """Structured/non-finite metadata hashes must fail closed."""

        malformed_values = ({"nested": "value"}, ["hash"], float("nan"), 123, True, None)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"MODEL_BYTES")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8")
                .replace("model:\n", "model:\n  sha256: " + digest + "\n")
                .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"),
                encoding="utf-8",
            )

            for malformed in malformed_values:
                with self.subTest(value=malformed):
                    result = qualify_offline(
                        model_profile=profile,
                        pnp_config=self.pnp_config,
                        model=artifact,
                        allow_test_only=True,
                        metadata={"model_sha256": malformed},
                    )
                    self.assertEqual(result["status"], "FAIL")
                    self.assertTrue(
                        any(item["code"] == "model_hash_invalid" for item in result["diagnostics"]["errors"])
                    )

    def test_malformed_metadata_bin_hash_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "model.xml"
            binary = root / "model.bin"
            xml.write_bytes(b"MODEL_XML")
            binary.write_bytes(b"MODEL_BIN")
            profile = self._schema_v2_profile(root, xml, binary)

            for malformed in ({"nested": "value"}, ["hash"], float("inf"), 123, True, None):
                with self.subTest(value=malformed):
                    result = qualify_offline(
                        model_profile=profile,
                        model=xml,
                        model_bin=binary,
                        pnp_config=self.pnp_config,
                        allow_test_only=True,
                        metadata={"model_bin_sha256": malformed},
                    )
                    self.assertEqual(result["status"], "FAIL")
                    self.assertTrue(
                        any(item["code"] == "model_bin_hash_invalid" for item in result["diagnostics"]["errors"])
                    )

    def test_cli_empty_model_hash_is_not_silently_ignored(self):
        """An explicitly empty --model-sha256 remains an external assertion."""

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            artifact = root / "model.xml"
            artifact.write_bytes(b"MODEL_BYTES")
            digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
            profile = root / "profile.yaml"
            profile.write_text(
                self.model_profile.read_text(encoding="utf-8")
                .replace("model:\n", "model:\n  sha256: " + digest + "\n")
                .replace("path: external://sp_vision_25/assets/yolov5.xml", f"path: {artifact}"),
                encoding="utf-8",
            )
            output_json = root / "qualification.json"
            output_markdown = root / "qualification.md"
            command = [
                sys.executable,
                str(ROOT / "tools/auto_aim_qualification/auto_aim_qualification.py"),
                "--allow-test-only",
                "--model-profile", str(profile),
                "--model", str(artifact),
                "--pnp-config", str(self.pnp_config),
                "--model-sha256", "",
                "--output-json", str(output_json),
                "--output-markdown", str(output_markdown),
            ]
            result = subprocess.run(command, check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            report = json.loads(output_json.read_text(encoding="utf-8"))
            self.assertTrue(any(item["code"] == "model_hash_external_invalid" for item in report["diagnostics"]["errors"]))

    def test_cli_empty_model_bin_hash_is_not_silently_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            xml = root / "model.xml"
            binary = root / "model.bin"
            xml.write_bytes(b"MODEL_XML")
            binary.write_bytes(b"MODEL_BIN")
            profile = self._schema_v2_profile(root, xml, binary)
            output_json = root / "qualification.json"
            output_markdown = root / "qualification.md"
            command = [
                sys.executable,
                str(ROOT / "tools/auto_aim_qualification/auto_aim_qualification.py"),
                "--allow-test-only",
                "--model-profile", str(profile),
                "--model", str(xml),
                "--model-bin", str(binary),
                "--pnp-config", str(self.pnp_config),
                "--model-bin-sha256", "",
                "--output-json", str(output_json),
                "--output-markdown", str(output_markdown),
            ]
            result = subprocess.run(command, check=False, capture_output=True, text=True)

            self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
            self.assertIn("status=FAIL", result.stdout)
            report = json.loads(output_json.read_text(encoding="utf-8"))
            self.assertTrue(
                any(item["code"] == "model_bin_hash_external_invalid" for item in report["diagnostics"]["errors"])
            )

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
