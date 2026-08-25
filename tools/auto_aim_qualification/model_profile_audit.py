#!/usr/bin/env python3
"""Fail-closed, read-only audits for detector and PnP profile contracts.

The C++ detector and PnP loaders remain the runtime source of truth.  This
module is a preflight gate around those contracts: it does not load OpenVINO,
run a camera, or infer semantics from file names/class numbers.  The checks
mirror the explicitly documented schema and add provenance/hash checks that
are deliberately outside the runtime loader.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

try:  # PyYAML is present in the project environment; keep import optional.
    import yaml  # type: ignore
except Exception:  # pragma: no cover - exercised only on minimal hosts.
    yaml = None


STATUS_CODES = {"PASS": 0, "WARN": 2, "FAIL": 1}
_SEVERITY = {"PASS": 0, "WARN": 1, "FAIL": 2}
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")


def status_code(status: str) -> int:
    """Return the stable CLI code used by the project evidence tools."""

    return STATUS_CODES.get(status, 1)


def _status_from(findings: Iterable["Finding"]) -> str:
    highest = "PASS"
    for finding in findings:
        if _SEVERITY.get(finding.status, 2) > _SEVERITY[highest]:
            highest = finding.status
    return highest


def _safe_name(value: Any) -> str:
    """Reduce a path/URI to a non-sensitive basename for reports."""

    if value is None:
        return ""
    text = str(value).replace("\\", "/")
    if "://" in text:
        # Keep a useful fixture identifier without exposing a host/path.
        return text.rsplit("/", 1)[-1] or text.split("://", 1)[0]
    return Path(text).name


def _redact_value(value: Any) -> Any:
    """Recursively redact absolute paths while retaining scalar evidence."""

    if isinstance(value, Mapping):
        return {str(key): _redact_value(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_redact_value(child) for child in value]
    if isinstance(value, str):
        normalized = value.replace("\\", "/")
        if normalized.startswith("/") or re.match(r"^[A-Za-z]:/", normalized) or normalized.startswith("//"):
            return "<path>/" + _safe_name(value)
        return value
    return value


def _is_uri(value: str) -> bool:
    return "://" in value


def _looks_unreviewed(text: Any) -> bool:
    if not isinstance(text, str):
        return False
    lowered = text.lower()
    return any(marker in lowered for marker in ("legacy", "unconfirmed", "synthetic", "fixture", "demo", "sp_vision_25"))


def _finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _mapping(value: Any) -> Optional[Mapping[str, Any]]:
    return value if isinstance(value, Mapping) else None


def _nonempty(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


@dataclass(frozen=True)
class Finding:
    """One review finding.  Values are kept deliberately small/redacted."""

    status: str
    code: str
    message: str
    field: Optional[str] = None
    value: Any = None

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": self.status,
            "code": self.code,
            "message": self.message,
        }
        if self.field is not None:
            result["field"] = self.field
        if self.value is not None:
            if isinstance(self.value, (str, int, float, bool)) or self.value is None:
                result["value"] = _redact_value(self.value)
            else:
                result["value"] = str(self.value)
        return result


@dataclass
class AuditResult:
    """JSON-compatible result from one profile audit."""

    kind: str
    path: str = ""
    profile_id: Optional[str] = None
    profile_version: Optional[str] = None
    profile_kind: Optional[str] = None
    model_path: Optional[str] = None
    model_sha256: Optional[str] = None
    values: dict[str, Any] = field(default_factory=dict)
    findings: list[Finding] = field(default_factory=list)

    @property
    def status(self) -> str:
        return _status_from(self.findings)

    def add(
        self,
        status: str,
        code: str,
        message: str,
        *,
        field: Optional[str] = None,
        value: Any = None,
    ) -> None:
        self.findings.append(Finding(status, code, message, field, value))

    def fail(self, code: str, message: str, *, field: Optional[str] = None, value: Any = None) -> None:
        self.add("FAIL", code, message, field=field, value=value)

    def warn(self, code: str, message: str, *, field: Optional[str] = None, value: Any = None) -> None:
        self.add("WARN", code, message, field=field, value=value)

    def as_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "status": self.status,
            "kind": self.kind,
            "profile_path": _safe_name(self.path),
            "profile_id": self.profile_id,
            "profile_version": self.profile_version,
            "profile_kind": self.profile_kind,
            "model_path": _safe_name(self.model_path) if self.model_path else None,
            "model_sha256": self.model_sha256,
            "values": _redact_value(self.values),
            "findings": [finding.as_dict() for finding in self.findings],
        }
        return result


def _load_yaml(path: Path, result: AuditResult) -> Optional[Mapping[str, Any]]:
    if yaml is None:
        result.fail("yaml_loader_unavailable", "PyYAML is unavailable; profile cannot be audited")
        return None
    try:
        with path.open("r", encoding="utf-8") as handle:
            value = yaml.safe_load(handle)
    except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:  # type: ignore[union-attr]
        result.fail("yaml_parse_error", f"unable to parse profile YAML: {type(exc).__name__}")
        return None
    if not isinstance(value, Mapping):
        result.fail("yaml_root_not_map", "profile YAML root must be a map")
        return None
    return value


def _require_map(root: Mapping[str, Any], key: str, result: AuditResult) -> Optional[Mapping[str, Any]]:
    value = root.get(key)
    if not isinstance(value, Mapping):
        result.fail("missing_or_invalid_section", f"{key} must be a map", field=key)
        return None
    return value


def _require_string(root: Mapping[str, Any], key: str, result: AuditResult, section: str) -> Optional[str]:
    value = root.get(key)
    if not _nonempty(value):
        result.fail("missing_required_field", f"{section}.{key} is required", field=f"{section}.{key}")
        return None
    return str(value).strip()


def _require_sequence(
    root: Mapping[str, Any],
    key: str,
    result: AuditResult,
    section: str,
    length: Optional[int] = None,
) -> Optional[list[Any]]:
    value = root.get(key)
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        result.fail("missing_or_invalid_sequence", f"{section}.{key} must be a sequence", field=f"{section}.{key}")
        return None
    values = list(value)
    if length is not None and len(values) != length:
        result.fail(
            "invalid_sequence_length",
            f"{section}.{key} must contain exactly {length} items",
            field=f"{section}.{key}",
            value=len(values),
        )
        return None
    return values


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _same_path(left: Path, right: Path) -> bool:
    try:
        return os.path.samefile(left, right)
    except (FileNotFoundError, OSError):
        try:
            return left.resolve(strict=False) == right.resolve(strict=False)
        except OSError:
            return os.path.normcase(os.path.abspath(left)) == os.path.normcase(os.path.abspath(right))


def _unsafe_file_alias(path: Path) -> bool:
    try:
        return path.is_symlink() or path.stat().st_nlink > 1
    except OSError:
        return True


def _expected_hash(model: Mapping[str, Any]) -> Optional[str]:
    for key in ("sha256", "sha256sum", "artifact_sha256", "hash"):
        value = model.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return None


def _check_common_profile_header(
    root: Mapping[str, Any],
    result: AuditResult,
    *,
    mode: str,
    allow_test_only: bool,
) -> tuple[Optional[Mapping[str, Any]], Optional[str]]:
    schema_version = root.get("schema_version")
    if schema_version != 1:
        result.fail("schema_version", "schema_version must be exactly 1", field="schema_version", value=schema_version)
    profile = root.get("profile")
    if profile not in {"test_only", "production"}:
        result.fail("profile_kind", "profile must be test_only or production", field="profile", value=profile)
        profile = None
    result.profile_kind = str(profile) if profile is not None else None
    if profile == "test_only":
        if not allow_test_only:
            result.fail("test_only_not_allowed", "test_only profile requires explicit opt-in", field="profile")
        else:
            result.warn("test_only_profile", "test_only profile is permitted only for evidence; never production-ready")
        if mode == "strict":
            result.fail("strict_test_only", "strict mode cannot qualify a test_only profile")
    return _require_map(root, "model", result), str(profile) if profile is not None else None


def _audit_model_contract(
    root: Mapping[str, Any],
    result: AuditResult,
    *,
    profile_kind: Optional[str],
    mode: str,
) -> Optional[Mapping[str, Any]]:
    model = _require_map(root, "model", result)
    if model is None:
        return None
    result.profile_id = _require_string(model, "id", result, "model")
    result.profile_version = _require_string(model, "version", result, "model")
    source = _require_string(model, "source", result, "model")
    model_path = _require_string(model, "path", result, "model")
    result.model_path = model_path
    result.values["model_source"] = source
    if profile_kind == "production" and model_path and (_is_uri(model_path) or not Path(model_path).is_absolute()):
        result.fail("production_model_path", "production model.path must be an absolute local path", field="model.path")
    if profile_kind == "test_only" and model_path and _is_uri(model_path):
        result.values["external_fixture_identifier"] = _safe_name(model_path)
    if source and profile_kind == "test_only":
        lowered = source.lower()
        if "synthetic" in lowered or "legacy" in lowered or "fixture" in lowered:
            result.values["fixture_marker"] = "test_only/synthetic_fixture/not_competition_evidence"
    if profile_kind == "production" and any(_looks_unreviewed(item) for item in (source, result.profile_id, result.profile_version, model_path)):
        result.fail(
            "unreviewed_production_provenance",
            "production profile provenance contains a legacy/demo/synthetic/unconfirmed marker",
            field="model.source",
        )

    input_node = _require_map(root, "input", result)
    if input_node is not None:
        shape = _require_sequence(input_node, "shape", result, "input", 4)
        if shape is not None:
            result.values["input_shape"] = shape
            if shape != [1, 3, shape[2], shape[3]] or not all(isinstance(item, int) and item > 0 for item in shape[2:]):
                result.fail("input_shape", "input.shape must be [1,3,height,width] with positive dimensions", field="input.shape")
        layout = _require_string(input_node, "layout", result, "input")
        if layout != "NCHW":
            result.fail("input_layout", "input.layout must be NCHW", field="input.layout", value=layout)
        element_type = _require_string(input_node, "element_type", result, "input")
        if element_type != "f32":
            result.fail("input_element_type", "input.element_type must be exactly f32 for the current decoder", field="input.element_type", value=element_type)
        source_color = _require_string(input_node, "source_color_order", result, "input")
        model_color = _require_string(input_node, "model_color_order", result, "input")
        if source_color != "BGR" or model_color != "RGB":
            result.fail("color_order", "input must explicitly declare BGR source and RGB model order", field="input.source_color_order")
        normalization = _require_string(input_node, "normalization", result, "input")
        if normalization != "divide_255":
            result.fail("normalization", "input.normalization must be divide_255", field="input.normalization", value=normalization)
        resize_mode = _require_string(input_node, "resize_mode", result, "input")
        if resize_mode not in {"top_left", "center"}:
            result.fail("resize_mode", "input.resize_mode must be top_left or center", field="input.resize_mode", value=resize_mode)
        # The current schema encodes letterbox/padding through resize_mode.
        # Newer profiles may make the rule explicit; production must not leave
        # that interpretation implicit.
        explicit_letterbox = input_node.get("letterbox_mode", input_node.get("letterbox"))
        explicit_padding = input_node.get("padding")
        if explicit_letterbox is None and explicit_padding is None:
            # Schema v1 deliberately uses resize_mode as the complete
            # top-left/center padding contract; do not invent a second rule.
            result.values["resize_padding_rule"] = resize_mode
        if explicit_letterbox is not None and explicit_letterbox not in {"top_left", "center", "letterbox"}:
            result.fail("letterbox_rule", "unsupported explicit letterbox rule", field="input.letterbox_mode")
        if explicit_padding is not None and not isinstance(explicit_padding, (str, list, tuple, Mapping)):
            result.fail("padding_rule", "input.padding must be a documented mapping/sequence/string", field="input.padding")

    output_node = _require_map(root, "output", result)
    if output_node is not None:
        shape = _require_sequence(output_node, "shape", result, "output", 3)
        if shape is not None:
            result.values["output_shape"] = shape
            if shape[0] != 1 or not all(isinstance(item, int) and item > 0 for item in shape):
                result.fail("output_shape", "output.shape must be [1,rows,columns] with positive dimensions", field="output.shape")
        layout = _require_string(output_node, "layout", result, "output")
        if layout != "NRC":
            result.fail("output_layout", "output.layout must be NRC", field="output.layout", value=layout)
        element_type = _require_string(output_node, "element_type", result, "output")
        if element_type != "f32":
            result.fail("output_element_type", "output.element_type must be exactly f32 for the current decoder", field="output.element_type", value=element_type)
        for key in ("keypoint_count", "objectness_index", "color_logits_offset", "color_class_count", "armor_logits_offset", "armor_class_count"):
            value = output_node.get(key)
            if not isinstance(value, int) or isinstance(value, bool) or value < 0:
                result.fail("output_field", f"output.{key} must be a non-negative integer", field=f"output.{key}", value=value)
        if output_node.get("keypoint_count") != 4:
            result.fail("keypoint_count", "output.keypoint_count must be exactly 4", field="output.keypoint_count")
        for key, expected in (("objectness_index", 8), ("color_logits_offset", 9), ("armor_logits_offset", 13)):
            if output_node.get(key) != expected:
                result.fail("tensor_offset", f"output.{key} must be {expected} for the current decoder", field=f"output.{key}", value=output_node.get(key))
        if isinstance(output_node.get("shape"), list) and len(output_node["shape"]) == 3:
            columns = output_node["shape"][2]
            for offset_key, count_key in (("color_logits_offset", "color_class_count"), ("armor_logits_offset", "armor_class_count")):
                offset, count = output_node.get(offset_key), output_node.get(count_key)
                if isinstance(columns, int) and isinstance(offset, int) and isinstance(count, int) and (offset + count > columns or count <= 0):
                    result.fail("tensor_range", f"{offset_key}+{count_key} must fit output columns", field=f"output.{offset_key}")

    postprocess = _require_map(root, "postprocess", result)
    if postprocess is not None:
        for key in ("objectness_threshold", "nms_threshold"):
            value = postprocess.get(key)
            if not _finite_number(value) or not 0.0 <= float(value) <= 1.0:
                result.fail("threshold", f"postprocess.{key} must be finite and in [0,1]", field=f"postprocess.{key}", value=value)
        order = _require_sequence(postprocess, "keypoint_order", result, "postprocess", 4)
        # The checked-in reference adapter explicitly uses [0,3,2,1] to
        # convert raw model pairs into TL,TR,BR,BL.  It is therefore the
        # canonical order for this schema, even though [0,1,2,3] is also a
        # mathematically valid permutation for a future reviewed adapter.
        if order is not None and order != [0, 3, 2, 1]:
            if sorted(order) != [0, 1, 2, 3]:
                result.fail("keypoint_order", "postprocess.keypoint_order must be a permutation of [0,1,2,3]", field="postprocess.keypoint_order", value=order)
            else:
                result.warn("noncanonical_keypoint_order", "keypoint order is explicit but differs from canonical TL,TR,BR,BL")

    semantics = _require_map(root, "semantics", result)
    if semantics is not None:
        color_count = output_node.get("color_class_count") if output_node else None
        armor_count = output_node.get("armor_class_count") if output_node else None
        colors = _require_sequence(semantics, "color_id_to_name", result, "semantics")
        armors = _require_sequence(semantics, "armor_class_names", result, "semantics")
        for label, values, count in (("color_id_to_name", colors, color_count), ("armor_class_names", armors, armor_count)):
            if values is not None:
                if count is not None and len(values) != count:
                    result.fail("semantic_count", f"semantics.{label} length must match output class count", field=f"semantics.{label}")
                if any(not _nonempty(item) for item in values) or len(set(str(item) for item in values)) != len(values):
                    result.fail("semantic_names", f"semantics.{label} must contain unique non-empty names", field=f"semantics.{label}")
                if profile_kind == "production" and any(_looks_unreviewed(item) for item in values):
                    result.fail("unreviewed_production_semantics", f"production semantics in {label} are marked legacy/unconfirmed", field=f"semantics.{label}")
        mapping = semantics.get("class_to_armor_type")
        if not isinstance(mapping, Mapping):
            result.fail("class_mapping", "semantics.class_to_armor_type must be an explicit map", field="semantics.class_to_armor_type")
        else:
            normalized: dict[int, str] = {}
            for key, value in mapping.items():
                try:
                    class_id = int(key)
                except (TypeError, ValueError):
                    result.fail("class_mapping_key", "class_to_armor_type keys must be integer ids", field="semantics.class_to_armor_type")
                    continue
                normalized[class_id] = str(value)
                if str(value) not in {"small", "large"}:
                    result.fail("class_mapping_value", "class_to_armor_type values must be small or large", field="semantics.class_to_armor_type", value=value)
            if isinstance(armor_count, int) and set(normalized) != set(range(armor_count)):
                result.fail("class_mapping_incomplete", "class_to_armor_type must map every armor class exactly once", field="semantics.class_to_armor_type")
            result.values["class_to_armor_type"] = {str(key): value for key, value in sorted(normalized.items())}
    return model


def audit_model_profile(
    profile_path: str | Path,
    *,
    model_path: str | Path | None = None,
    mode: str = "evidence_only",
    allow_test_only: bool = False,
    expected_model_sha256: Optional[str] = None,
) -> AuditResult:
    """Audit a detector ``model_profile.yaml`` and optional runtime artifact."""

    path = Path(profile_path)
    result = AuditResult(kind="model_profile", path=str(path))
    if not path.is_file():
        result.fail("profile_missing", "model profile file does not exist or is not readable")
        return result
    if _unsafe_file_alias(path):
        result.fail("profile_alias", "model profile must not be a symlink or hardlink alias")
    root = _load_yaml(path, result)
    if root is None:
        return result
    model, profile_kind = _check_common_profile_header(root, result, mode=mode, allow_test_only=allow_test_only)
    if model is None:
        return result
    model = _audit_model_contract(root, result, profile_kind=profile_kind, mode=mode) or model
    declared_path = model.get("path") if isinstance(model, Mapping) else None
    runtime = Path(model_path) if model_path is not None else None
    if runtime is None and isinstance(declared_path, str) and declared_path and not _is_uri(declared_path):
        runtime = Path(declared_path)
    if model_path is not None and declared_path and isinstance(declared_path, str) and not _is_uri(declared_path):
        if not _same_path(runtime, Path(declared_path)):
            result.fail("model_path_mismatch", "runtime model path does not match profile model.path", field="model.path")
    if profile_kind == "production" and model_path is None:
        result.fail("runtime_model_path_missing", "production audit requires the actual runtime model path", field="--model")
    if runtime is None:
        (result.fail if mode == "strict" or profile_kind == "production" else result.warn)(
            "model_artifact_unavailable", "model artifact path is external/unavailable; pipeline execution unavailable", field="model.path"
        )
    elif not runtime.is_file() or not os.access(runtime, os.R_OK):
        (result.fail if mode == "strict" or profile_kind == "production" else result.warn)(
            "model_artifact_missing", "model file does not exist or is not readable; pipeline execution unavailable", field="model.path"
        )
    else:
        try:
            if _unsafe_file_alias(runtime):
                (result.fail if mode == "strict" or profile_kind == "production" else result.warn)(
                    "model_artifact_alias", "model artifact must not be a symlink or hardlink alias", field="model.path"
                )
            digest = _sha256(runtime)
            result.model_sha256 = digest
            declared_hash = expected_model_sha256 or _expected_hash(model)
            if declared_hash is None:
                (result.fail if mode == "strict" or profile_kind == "production" else result.warn)(
                    "model_hash_missing", "model artifact SHA-256 is not declared in the reviewed profile", field="model.sha256"
                )
            elif not _SHA256_RE.fullmatch(str(declared_hash)):
                result.fail("model_hash_invalid", "declared model SHA-256 is not 64 hexadecimal characters", field="model.sha256")
            elif digest.lower() != str(declared_hash).lower():
                result.fail("model_hash_mismatch", "model artifact SHA-256 does not match the profile", field="model.sha256")
        except OSError:
            (result.fail if mode == "strict" or profile_kind == "production" else result.warn)(
                "model_hash_unreadable", "model artifact could not be hashed", field="model.path"
            )
    result.values["runtime_model_path"] = _safe_name(runtime) if runtime is not None else None
    result.values["schema_source"] = "docs/model_profile_schema.md + C++ loader contract"
    return result


def _check_matrix(values: Any, result: AuditResult, field_name: str) -> Optional[list[float]]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or len(values) != 9:
        result.fail("matrix_shape", f"{field_name} must contain 9 values", field=field_name)
        return None
    numbers: list[float] = []
    for item in values:
        if not _finite_number(item):
            result.fail("matrix_nonfinite", f"{field_name} contains a non-finite value", field=field_name)
            return None
        numbers.append(float(item))
    return numbers


def _check_rotation(values: Any, result: AuditResult) -> None:
    matrix = _check_matrix(values, result, "camera_to_gimbal.rotation_gimbal_from_camera")
    if matrix is None:
        return
    rows = [matrix[0:3], matrix[3:6], matrix[6:9]]
    dot = lambda a, b: sum(x * y for x, y in zip(a, b))
    norms = [dot(row, row) for row in rows]
    if any(abs(value - 1.0) > 1e-3 for value in norms) or any(abs(dot(rows[i], rows[j])) > 1e-3 for i in range(3) for j in range(i + 1, 3)):
        result.fail("extrinsic_rotation", "configured camera_to_gimbal rotation is not orthonormal", field="camera_to_gimbal.rotation_gimbal_from_camera")
    determinant = (
        matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7])
        - matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6])
        + matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6])
    )
    if abs(determinant - 1.0) > 1e-3:
        result.fail("extrinsic_determinant", "configured camera_to_gimbal rotation must have det=+1", field="camera_to_gimbal.rotation_gimbal_from_camera", value=determinant)


def audit_pnp_config(
    config_path: str | Path,
    *,
    mode: str = "evidence_only",
    allow_test_only: bool = False,
    expected_image_size: Optional[tuple[int, int]] = None,
    expected_class_mapping: Optional[Mapping[int, str]] = None,
) -> AuditResult:
    """Audit the versioned PnP/calibration YAML without running OpenCV."""

    path = Path(config_path)
    result = AuditResult(kind="pnp_config", path=str(path))
    if not path.is_file():
        result.fail("pnp_config_missing", "PnP configuration file does not exist or is not readable")
        return result
    if _unsafe_file_alias(path):
        result.fail("pnp_config_alias", "PnP configuration must not be a symlink or hardlink alias")
    root = _load_yaml(path, result)
    if root is None:
        return result
    schema_version = root.get("schema_version")
    if schema_version != 1:
        result.fail("schema_version", "PnP schema_version must be exactly 1", field="schema_version", value=schema_version)
    profile = root.get("profile")
    if profile not in {"test_only", "production"}:
        result.fail("profile_kind", "PnP profile must be test_only or production", field="profile", value=profile)
    result.profile_kind = str(profile) if profile in {"test_only", "production"} else None
    if profile == "test_only":
        if not allow_test_only:
            result.fail("test_only_not_allowed", "test_only PnP config requires explicit opt-in", field="profile")
        else:
            result.warn("test_only_pnp", "test_only PnP/calibration is synthetic evidence only")
        if mode == "strict":
            result.fail("strict_test_only", "strict mode cannot qualify a test_only PnP config")
    camera = _require_map(root, "camera", result)
    if camera is not None:
        width, height = camera.get("image_width"), camera.get("image_height")
        if not isinstance(width, int) or not isinstance(height, int) or width <= 0 or height <= 0:
            result.fail("camera_dimensions", "camera.image_width/image_height must be positive integers", field="camera.image_width")
        elif expected_image_size and (width, height) != expected_image_size:
            result.fail("pnp_resolution_mismatch", "PnP resolution does not match the expected frame resolution", field="camera.image_width", value=f"{width}x{height}")
        result.values["camera_resolution"] = [width, height]
        matrix = _check_matrix(camera.get("camera_matrix"), result, "camera.camera_matrix")
        if matrix is not None and (
            matrix[0] <= 0.0 or matrix[4] <= 0.0 or abs(matrix[8] - 1.0) > 1e-6 or
            any(abs(matrix[index]) > 1e-9 for index in (6, 7))
        ):
            result.fail("camera_intrinsics", "camera_matrix must have positive fx/fy and bottom row [0,0,1]", field="camera.camera_matrix")
        distortion = camera.get("distortion_coefficients")
        if not isinstance(distortion, Sequence) or isinstance(distortion, (str, bytes)) or len(distortion) not in {4, 5, 8, 12, 14} or any(not _finite_number(item) for item in distortion):
            result.fail("distortion", "distortion_coefficients must be a finite OpenCV coefficient sequence", field="camera.distortion_coefficients")
        for key in ("source", "version", "coordinate_frame"):
            _require_string(camera, key, result, "camera")
        if camera.get("coordinate_frame") != "opencv_camera_optical":
            result.fail("camera_frame", "camera.coordinate_frame must be opencv_camera_optical", field="camera.coordinate_frame")
        result.values["pnp_source"] = camera.get("source")
        if profile == "production" and _looks_unreviewed(camera.get("source")):
            result.fail("unreviewed_production_provenance", "production camera source is marked legacy/demo/synthetic/unconfirmed", field="camera.source")
    armor_geometry = _require_map(root, "armor_geometry", result)
    mapping = root.get("class_to_armor_type")
    normalized_mapping: dict[int, str] = {}
    if isinstance(mapping, Mapping):
        for key, value in mapping.items():
            try:
                class_id = int(key)
            except (TypeError, ValueError):
                result.fail("class_mapping_key", "class_to_armor_type keys must be integer ids", field="class_to_armor_type")
                continue
            normalized_mapping[class_id] = str(value)
            if str(value) not in {"small", "large"}:
                result.fail("class_mapping_value", "class_to_armor_type values must be small or large", field="class_to_armor_type")
    else:
        result.fail("class_mapping_missing", "PnP class_to_armor_type must be explicit", field="class_to_armor_type")
    if expected_class_mapping is not None and normalized_mapping != {int(k): str(v) for k, v in expected_class_mapping.items()}:
        result.fail("geometry_semantic_conflict", "model and PnP class_to_armor_type mappings disagree", field="class_to_armor_type")
    if armor_geometry is not None:
        corner_order = armor_geometry.get("corner_order")
        if corner_order != ["top_left", "top_right", "bottom_right", "bottom_left"]:
            result.fail("corner_order", "armor_geometry.corner_order must be TL,TR,BR,BL", field="armor_geometry.corner_order", value=corner_order)
        for size in ("small", "large"):
            geometry = armor_geometry.get(size)
            if not isinstance(geometry, Mapping):
                result.fail("armor_geometry_missing", f"armor_geometry.{size} must be a map", field=f"armor_geometry.{size}")
                continue
            for key in ("width_m", "height_m"):
                if not _finite_number(geometry.get(key)) or float(geometry.get(key)) <= 0.0:
                    result.fail("armor_geometry_size", f"armor_geometry.{size}.{key} must be positive and finite", field=f"armor_geometry.{size}.{key}")
            points = geometry.get("object_points_m")
            if not isinstance(points, Sequence) or isinstance(points, (str, bytes)) or len(points) != 4 or any(not isinstance(point, Sequence) or len(point) != 3 or any(not _finite_number(item) for item in point) for point in points):
                result.fail("armor_geometry_points", f"armor_geometry.{size}.object_points_m must contain four finite 3D points", field=f"armor_geometry.{size}.object_points_m")
            elif points:
                # Check the geometric contract without guessing dimensions:
                # all corners must be coplanar, cyclic, and consistent with
                # the declared width/height (within measurement tolerance).
                xyz = [[float(item) for item in point] for point in points]
                def dist(first: Sequence[float], second: Sequence[float]) -> float:
                    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first, second)))
                def vec(first: Sequence[float], second: Sequence[float]) -> list[float]:
                    return [second[index] - first[index] for index in range(3)]
                def dot(first: Sequence[float], second: Sequence[float]) -> float:
                    return sum(a * b for a, b in zip(first, second))
                def cross(first: Sequence[float], second: Sequence[float]) -> list[float]:
                    return [first[1] * second[2] - first[2] * second[1], first[2] * second[0] - first[0] * second[2], first[0] * second[1] - first[1] * second[0]]
                horizontal_top = vec(xyz[0], xyz[1])
                horizontal_bottom = vec(xyz[3], xyz[2])
                vertical_left = vec(xyz[0], xyz[3])
                vertical_right = vec(xyz[1], xyz[2])
                edge_lengths = [dist(xyz[index], xyz[(index + 1) % 4]) for index in range(4)]
                normal = cross(horizontal_top, vertical_left)
                diagonal = vec(xyz[0], xyz[2])
                normal_norm = math.sqrt(dot(normal, normal))
                if normal_norm <= 1e-12 or abs(dot(normal, diagonal)) > max(1e-9, normal_norm * 1e-6):
                    result.fail("armor_geometry_planarity", f"armor_geometry.{size} points must be planar", field=f"armor_geometry.{size}.object_points_m")
                width_value = geometry.get("width_m", 0.0)
                height_value = geometry.get("height_m", 0.0)
                width = float(width_value) if _finite_number(width_value) else 0.0
                height = float(height_value) if _finite_number(height_value) else 0.0
                if width > 0.0 and height > 0.0:
                    observed_width = (edge_lengths[0] + edge_lengths[2]) * 0.5
                    observed_height = (edge_lengths[1] + edge_lengths[3]) * 0.5
                    if abs(observed_width - width) > max(1e-9, width * 0.01) or abs(observed_height - height) > max(1e-9, height * 0.01):
                        result.fail("armor_geometry_dimensions", f"armor_geometry.{size} points do not match declared width/height", field=f"armor_geometry.{size}.object_points_m")
                horizontal_norm = math.sqrt(dot(horizontal_top, horizontal_top))
                bottom_norm = math.sqrt(dot(horizontal_bottom, horizontal_bottom))
                vertical_norm = math.sqrt(dot(vertical_left, vertical_left))
                right_norm = math.sqrt(dot(vertical_right, vertical_right))
                if (
                    dot(horizontal_top, horizontal_bottom) <= 0.0 or dot(vertical_left, vertical_right) <= 0.0 or
                    math.sqrt(dot(cross(horizontal_top, horizontal_bottom), cross(horizontal_top, horizontal_bottom))) > horizontal_norm * bottom_norm * 1e-6 or
                    math.sqrt(dot(cross(vertical_left, vertical_right), cross(vertical_left, vertical_right))) > vertical_norm * right_norm * 1e-6 or
                    abs(dot(horizontal_top, vertical_left)) > horizontal_norm * vertical_norm * 1e-6
                ):
                    result.fail("armor_geometry_rectangle", f"armor_geometry.{size} points are not an ordered rectangle", field=f"armor_geometry.{size}.object_points_m")
            for key in ("source", "version", "object_frame"):
                _require_string(geometry, key, result, f"armor_geometry.{size}")
            if profile == "production" and _looks_unreviewed(geometry.get("source")):
                result.fail("unreviewed_production_provenance", f"production {size} armor geometry source is marked legacy/demo/synthetic/unconfirmed", field=f"armor_geometry.{size}.source")
    pnp = _require_map(root, "pnp", result)
    if pnp is not None:
        method = pnp.get("method")
        if method != "ITERATIVE":
            result.fail("pnp_method", "only explicitly supported ITERATIVE PnP is accepted", field="pnp.method", value=method)
        threshold = pnp.get("max_reprojection_error_px")
        if not _finite_number(threshold) or float(threshold) <= 0.0:
            result.fail("pnp_threshold", "pnp.max_reprojection_error_px must be positive and finite", field="pnp.max_reprojection_error_px")
    extrinsic = _require_map(root, "camera_to_gimbal", result)
    if extrinsic is not None:
        configured = extrinsic.get("configured")
        if not isinstance(configured, bool):
            result.fail("extrinsic_configured", "camera_to_gimbal.configured must be boolean", field="camera_to_gimbal.configured")
        if profile == "production" and configured is not True:
            result.fail("production_extrinsic_missing", "production PnP requires a configured camera_to_gimbal extrinsic", field="camera_to_gimbal.configured")
        if configured is True:
            _check_rotation(extrinsic.get("rotation_gimbal_from_camera"), result)
            translation = extrinsic.get("translation_gimbal_from_camera_m")
            if not isinstance(translation, Sequence) or len(translation) != 3 or any(not _finite_number(item) for item in translation):
                result.fail("extrinsic_translation", "configured extrinsic translation must be three finite values", field="camera_to_gimbal.translation_gimbal_from_camera_m")
            for key in ("source", "version", "source_frame", "target_frame"):
                _require_string(extrinsic, key, result, "camera_to_gimbal")
            if profile == "production" and _looks_unreviewed(extrinsic.get("source")):
                result.fail("unreviewed_production_provenance", "production extrinsic source is marked legacy/demo/synthetic/unconfirmed", field="camera_to_gimbal.source")
            if extrinsic.get("source_frame") != "opencv_camera_optical" or extrinsic.get("target_frame") != "gimbal_x_forward_y_left_z_up":
                result.fail("extrinsic_frames", "camera_to_gimbal source/target frames are not the documented frames", field="camera_to_gimbal.source_frame")
    # If a profile explicitly repeats the test-only marker in nested sections,
    # it must agree with the top-level kind.  Missing nested markers are
    # allowed because the canonical C++ loader derives them from `profile`.
    expected_test_only = profile == "test_only"
    for section_name, section in (("camera", camera), ("armor_geometry", armor_geometry), ("camera_to_gimbal", extrinsic)):
        if isinstance(section, Mapping) and "test_only" in section and section.get("test_only") is not expected_test_only:
            result.fail("test_only_marker_mismatch", f"{section_name}.test_only disagrees with top-level profile", field=f"{section_name}.test_only")
    if isinstance(armor_geometry, Mapping):
        for size in ("small", "large"):
            nested = armor_geometry.get(size)
            if isinstance(nested, Mapping) and "test_only" in nested and nested.get("test_only") is not expected_test_only:
                result.fail("test_only_marker_mismatch", f"armor_geometry.{size}.test_only disagrees with top-level profile", field=f"armor_geometry.{size}.test_only")
    result.profile_id = str(root.get("id") or root.get("pnp_id") or (camera or {}).get("version") or "")
    result.profile_version = str((camera or {}).get("version") or (extrinsic or {}).get("version") or "")
    result.values["pnp_source"] = (camera or {}).get("source")
    if not result.profile_version:
        result.fail("pnp_provenance", "PnP camera/calibration version is required", field="camera.version")
    result.values["class_to_armor_type"] = {str(key): value for key, value in sorted(normalized_mapping.items())}
    result.values["schema_source"] = "docs/pnp_config_schema.md + PnpStage loader contract"
    if profile == "test_only":
        result.values["fixture_marker"] = "test_only/synthetic_fixture/not_competition_evidence"
    return result


__all__ = ["AuditResult", "Finding", "STATUS_CODES", "audit_model_profile", "audit_pnp_config", "status_code"]
