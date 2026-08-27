#!/usr/bin/env python3
"""Fail-closed, read-only audits for detector and PnP profile contracts.

The C++ detector and PnP loaders remain the runtime source of truth.  This
module is a preflight gate around those contracts: it may read OpenVINO model
metadata, but never compiles a model, runs inference, uses hardware, runs a
camera, or infers semantics from file names/class numbers.  The checks mirror
the explicitly documented schema and add provenance/hash checks that are
deliberately outside the runtime loader.
"""

from __future__ import annotations

import hashlib
import math
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

from .preprocess_contract import software_preprocessing_evidence

try:  # PyYAML is present in the project environment; keep import optional.
    import yaml  # type: ignore
except Exception:  # pragma: no cover - exercised only on minimal hosts.
    yaml = None


STATUS_CODES = {"PASS": 0, "WARN": 2, "FAIL": 1}
_SEVERITY = {"PASS": 0, "WARN": 1, "FAIL": 2}
_SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
_INTEGER_TEXT_RE = re.compile(r"^[+-]?[0-9]+$")


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
    if isinstance(value, float) and not math.isfinite(value):
        # Python's default JSON encoder writes NaN/Infinity tokens, which are
        # not valid JSON.  Keep the evidence serializable while making the
        # rejected non-finite value explicit.
        return "<nonfinite>"
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


def _strict_int(value: Any) -> bool:
    """Return true only for a YAML/JSON integer, excluding boolean aliases."""

    return isinstance(value, int) and not isinstance(value, bool)


def _numeric_mapping_key(value: Any) -> Optional[int]:
    """Return an integer only to detect aliases among class-map keys.

    String keys are never accepted as class identifiers, but their integer
    spelling must still be normalized here so YAML ``1`` and ``\"1\"`` cannot
    silently overwrite one another during a later conversion.
    """

    if _strict_int(value):
        return value
    if isinstance(value, str):
        text = value.strip()
        if _INTEGER_TEXT_RE.fullmatch(text):
            try:
                return int(text)
            except ValueError:
                # Python can reject deliberately enormous integer strings;
                # they remain invalid keys and must not crash the audit.
                return None
    return None


def _normalize_class_mapping(
    mapping: Mapping[Any, Any],
    result: "AuditResult",
    *,
    field: str,
) -> tuple[dict[int, str], bool]:
    """Validate class-map keys without coercion or collision overwrite.

    The returned map contains only unambiguous, strict integer keys. A caller
    must not use it as an authoritative contract when ``keys_valid`` is false.
    """

    normalized: dict[int, str] = {}
    seen_normalized: set[int] = set()
    keys_valid = True
    for key, value in mapping.items():
        normalized_key = _numeric_mapping_key(key)
        key_is_strict_integer = _strict_int(key)
        if not key_is_strict_integer:
            result.fail(
                "class_mapping_key",
                "class_to_armor_type keys must be integer ids, not boolean or numeric-string aliases",
                field=field,
            )
            keys_valid = False
        if normalized_key is None:
            continue
        if normalized_key in seen_normalized:
            result.fail(
                "class_mapping_duplicate_key",
                "class_to_armor_type keys must not normalize to the same integer id",
                field=field,
            )
            keys_valid = False
            continue
        seen_normalized.add(normalized_key)
        if key_is_strict_integer:
            normalized[normalized_key] = str(value)
    return normalized, keys_valid


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
            result["value"] = _redact_value(self.value)
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


@dataclass(frozen=True)
class _ArtifactDeclaration:
    """One explicitly named model artifact from a reviewed profile.

    Schema v1's single ``model.path`` is represented as the XML member for
    backwards-compatible test-only fixtures.  Schema v2 names XML and BIN
    separately so a graph digest can never be misreported as a binding of the
    OpenVINO weights file.
    """

    name: str
    path: Optional[str]
    sha256: Any
    path_field: str
    sha256_field: str
    required: bool
    legacy_v1: bool = False


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


_MODEL_HASH_KEYS = ("sha256", "sha256sum", "artifact_sha256", "hash")
_MISSING_HASH = object()


def _expected_hash(model: Mapping[str, Any]) -> Any:
    """Return the first profile hash declaration, preserving malformed values.

    Do not filter by type here.  YAML parses an unquoted all-numeric digest
    (for example, ``64 * "0"``) as an integer; dropping that value would make
    a tampered profile look as though it had no declaration and would let an
    external hash silently replace it.
    """

    for key in _MODEL_HASH_KEYS:
        if key in model:
            value = model.get(key)
            return value.strip() if isinstance(value, str) else value
    return None


def _artifact_hash_code(artifact: _ArtifactDeclaration, suffix: str) -> str:
    """Keep legacy XML diagnostics stable while naming v2 members clearly."""

    if artifact.legacy_v1 and artifact.name == "xml":
        return f"model_hash_{suffix}"
    return f"model_{artifact.name}_hash_{suffix}"


def _artifact_path_code(artifact: _ArtifactDeclaration, suffix: str) -> str:
    """Keep the old v1 XML path diagnostic stable for existing evidence."""

    if artifact.legacy_v1 and artifact.name == "xml":
        return f"model_{suffix}"
    return f"model_{artifact.name}_{suffix}"


def _artifact_value(value: Any) -> Any:
    """Preserve malformed declarations for fail-closed JSON evidence."""

    return value.strip() if isinstance(value, str) else value


def _hash_evidence(value: Any) -> Optional[str]:
    """Return a report-safe digest value without echoing malformed input."""

    normalized = _artifact_value(value)
    if normalized is None:
        return None
    if isinstance(normalized, str) and _SHA256_RE.fullmatch(normalized):
        return normalized.lower()
    return "<invalid>"


def _digest_matches(actual_digest: Optional[str], declared_digest: Any) -> Optional[bool]:
    """Compare a readable artifact digest only to a syntactically valid one."""

    normalized = _artifact_value(declared_digest)
    if actual_digest is None or not isinstance(normalized, str) or not _SHA256_RE.fullmatch(normalized):
        return None
    return actual_digest.lower() == normalized.lower()


def _runtime_contract_unavailable(status: str = "model_artifact_unavailable") -> dict[str, Any]:
    """Return the stable no-runtime record without inventing actual ports."""

    return {
        "runtime": "openvino",
        "status": status,
        "available": False,
        "actual_input_shape": None,
        "actual_output_shape": None,
        "actual_input_element_type": None,
        "actual_output_element_type": None,
        "actual_input_layout": None,
        "actual_output_layout": None,
    }


def _runtime_unavailable_finding(
    result: AuditResult,
    *,
    profile_kind: Optional[str],
    mode: str,
    code: str,
    message: str,
    field: str = "model.path",
) -> None:
    """Record an unavailable dependency without ever allowing a false PASS.

    Evidence-only test fixtures may remain ``WARN`` when no model/runtime is
    present, because the report makes no detector-execution claim.  Production
    and strict qualification require the real artifact/runtime and therefore
    fail closed.
    """

    add = result.fail if mode == "strict" or profile_kind == "production" else result.warn
    add(code, message, field=field)


def _runtime_element_type(port: Any) -> str:
    """Normalize the OpenVINO port element-type spelling used by C++."""

    value = port.get_element_type()
    get_name = getattr(value, "get_type_name", None)
    if callable(get_name):
        value = get_name()
    text = str(value).strip().lower()
    # OpenVINO Python releases have used both ``f32`` and ``float32`` for the
    # same type.  Normalize spelling only; never coerce a different type.
    return {"float32": "f32", "fp32": "f32", "float16": "f16", "fp16": "f16"}.get(text, text)


def _runtime_layout(port: Any) -> Optional[str]:
    """Read an explicitly assigned OpenVINO layout, if the API exposes it.

    OpenVINO Python releases differ on whether layout is exposed directly on
    ``Output`` or on its owning node. Probe both locations without deriving an
    axis order from the tensor shape.
    """

    candidates = [port]
    get_node = getattr(port, "get_node", None)
    if callable(get_node):
        try:
            node = get_node()
        except Exception:
            node = None
        if node is not None:
            candidates.append(node)
    for candidate in candidates:
        getter = getattr(candidate, "get_layout", None)
        if not callable(getter):
            continue
        try:
            value = getter()
        except Exception:
            continue
        text = str(value).strip()
        if not text or text in {"[]", "?", "{?}"}:
            continue
        # Layout's string representation can be ``[N,C,H,W]`` in some releases.
        text = text.strip("[]").replace(",", "").replace(" ", "")
        if text:
            return text.upper()
    return None


def _runtime_static_shape(port: Any) -> tuple[Optional[list[int]], Optional[str]]:
    """Extract one static OpenVINO port shape without compiling or inferring."""

    partial = port.get_partial_shape()
    static = getattr(partial, "is_static", None)
    try:
        is_static = bool(static() if callable(static) else static)
    except Exception:
        return None, "partial_shape_invalid"
    if not is_static:
        return None, "dynamic"
    try:
        raw_shape = partial.to_shape()
        shape = [int(item) for item in raw_shape]
    except Exception:
        return None, "shape_unreadable"
    if not shape or any(item <= 0 for item in shape):
        return None, "shape_invalid"
    return shape, None


def _inspect_openvino_runtime(model_path: Path, model_bin_path: Path | None = None) -> dict[str, Any]:
    """Read an OpenVINO IR contract only; never compile or run inference.

    It is deliberately a small, patchable seam for unit tests.  The CLI does
    not provide an override: an installed OpenVINO Python runtime is the only
    source for ``actual_*`` values in a real report.
    """

    try:
        try:
            from openvino import Core  # type: ignore
        except ImportError:
            from openvino.runtime import Core  # type: ignore
    except Exception as exc:
        return {
            "status": "unavailable",
            "runtime": "openvino",
            "available": False,
            "reason": type(exc).__name__,
        }
    try:
        core = Core()
        # Schema-v2 profiles bind the XML graph and BIN weights separately.
        # Pass both paths to OpenVINO rather than relying on sibling-file
        # discovery, so the runtime contract is tied to the audited manifest.
        model = (
            core.read_model(str(model_path), str(model_bin_path))
            if model_bin_path is not None
            else core.read_model(str(model_path))
        )
        inputs = list(model.inputs)
        outputs = list(model.outputs)
    except Exception as exc:
        return {
            "status": "read_failed",
            "runtime": "openvino",
            "available": True,
            "reason": type(exc).__name__,
        }
    result: dict[str, Any] = {
        "status": "checked",
        "runtime": "openvino",
        "available": True,
        "input_count": len(inputs),
        "output_count": len(outputs),
    }
    if len(inputs) == 1:
        shape, shape_error = _runtime_static_shape(inputs[0])
        result["input_shape"] = shape
        result["input_shape_error"] = shape_error
        try:
            result["input_element_type"] = _runtime_element_type(inputs[0])
        except Exception:
            result["input_element_type"] = None
            result["input_element_type_error"] = "element_type_unreadable"
        result["input_layout"] = _runtime_layout(inputs[0])
    if len(outputs) == 1:
        shape, shape_error = _runtime_static_shape(outputs[0])
        result["output_shape"] = shape
        result["output_shape_error"] = shape_error
        try:
            result["output_element_type"] = _runtime_element_type(outputs[0])
        except Exception:
            result["output_element_type"] = None
            result["output_element_type_error"] = "element_type_unreadable"
        result["output_layout"] = _runtime_layout(outputs[0])
    return result


def _audit_runtime_contract(
    result: AuditResult,
    runtime: Path,
    *,
    runtime_bin: Path | None = None,
    input_node: Optional[Mapping[str, Any]],
    output_node: Optional[Mapping[str, Any]],
    profile_kind: Optional[str],
    mode: str,
) -> None:
    """Compare the actual static OpenVINO ports with the reviewed profile."""

    observed = _inspect_openvino_runtime(runtime, runtime_bin)
    status = observed.get("status")
    runtime_record = {
        "runtime": observed.get("runtime", "openvino"),
        "status": status if isinstance(status, str) else "invalid",
        "available": observed.get("available") is True,
        "reason": observed.get("reason"),
        "actual_input_shape": observed.get("input_shape"),
        "actual_output_shape": observed.get("output_shape"),
        "actual_input_element_type": observed.get("input_element_type"),
        "actual_output_element_type": observed.get("output_element_type"),
        "actual_input_layout": observed.get("input_layout"),
        "actual_output_layout": observed.get("output_layout"),
        "input_count": observed.get("input_count"),
        "output_count": observed.get("output_count"),
    }
    result.values["runtime_contract"] = runtime_record
    if status == "unavailable":
        _runtime_unavailable_finding(
            result,
            profile_kind=profile_kind,
            mode=mode,
            code="runtime_unavailable",
            message="OpenVINO runtime is unavailable; actual model contract was not claimed",
        )
        return
    if status != "checked":
        result.fail(
            "runtime_model_read_failed",
            "OpenVINO could not read the model artifact; actual contract is unavailable",
            field="model.path",
        )
        return

    if observed.get("input_count") != 1:
        result.fail("runtime_input_count", "OpenVINO model must expose exactly one input", field="input")
    if observed.get("output_count") != 1:
        result.fail("runtime_output_count", "OpenVINO model must expose exactly one output", field="output")
    for label, expected_node in (("input", input_node), ("output", output_node)):
        if (observed.get(f"{label}_count") if label == "input" else observed.get("output_count")) != 1:
            continue
        actual_shape = observed.get(f"{label}_shape")
        shape_error = observed.get(f"{label}_shape_error")
        actual_type = observed.get(f"{label}_element_type")
        type_error = observed.get(f"{label}_element_type_error")
        actual_layout = observed.get(f"{label}_layout")
        if shape_error is not None:
            code = "runtime_dynamic_shape" if shape_error == "dynamic" else "runtime_shape_unavailable"
            result.fail(code, f"OpenVINO {label} shape must be static and readable", field=f"{label}.shape")
        elif not isinstance(actual_shape, list) or not all(_strict_int(item) and item > 0 for item in actual_shape):
            result.fail("runtime_shape_unavailable", f"OpenVINO {label} shape is unavailable", field=f"{label}.shape")
        elif expected_node is not None and actual_shape != expected_node.get("shape"):
            result.fail(
                f"runtime_{label}_shape_mismatch",
                f"OpenVINO {label} shape does not match the reviewed profile",
                field=f"{label}.shape",
            )
        if type_error is not None or not isinstance(actual_type, str) or not actual_type:
            result.fail("runtime_element_type_unavailable", f"OpenVINO {label} element type is unavailable", field=f"{label}.element_type")
        elif expected_node is not None and actual_type != expected_node.get("element_type"):
            result.fail(
                f"runtime_{label}_element_type_mismatch",
                f"OpenVINO {label} element type does not match the reviewed profile",
                field=f"{label}.element_type",
                value=actual_type,
            )
        expected_layout = expected_node.get("layout") if expected_node is not None else None
        if not isinstance(actual_layout, str) or not actual_layout:
            # OpenVINO IR ports often carry no layout metadata at all.  Do
            # not invent one from dimensions: report that gap and let only a
            # reviewed profile provide the explicit NCHW/NRC interpretation.
            add = result.fail if mode == "strict" or profile_kind == "production" else result.warn
            add("runtime_layout_unavailable", f"OpenVINO {label} layout is unavailable", field=f"{label}.layout")
        elif expected_layout is not None and actual_layout != str(expected_layout).upper():
            result.fail(
                f"runtime_{label}_layout_mismatch",
                f"OpenVINO {label} layout does not match the reviewed profile",
                field=f"{label}.layout",
                value=actual_layout,
            )


def _check_common_profile_header(
    root: Mapping[str, Any],
    result: AuditResult,
    *,
    mode: str,
    allow_test_only: bool,
) -> tuple[Optional[str], Optional[int]]:
    schema_version = root.get("schema_version")
    if not _strict_int(schema_version) or schema_version not in {1, 2}:
        result.fail(
            "schema_version",
            "schema_version must be 1 (legacy test_only) or 2 (OpenVINO IR manifest)",
            field="schema_version",
            value=schema_version,
        )
        parsed_schema_version: Optional[int] = None
    else:
        parsed_schema_version = int(schema_version)
        result.values["model_profile_schema_version"] = parsed_schema_version
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
    return str(profile) if profile is not None else None, parsed_schema_version


def _profile_artifact_declarations(
    model: Mapping[str, Any],
    result: AuditResult,
    *,
    profile_kind: Optional[str],
    schema_version: Optional[int],
) -> tuple[_ArtifactDeclaration, ...]:
    """Parse only explicit model artifact fields; never infer a sibling BIN.

    Version 1 remains a compatibility format for the checked-in test-only
    fixture.  Version 2 is the only production-admissible representation and
    names both OpenVINO IR members independently.
    """

    if schema_version == 1:
        path = _require_string(model, "path", result, "model")
        if profile_kind == "production":
            result.fail(
                "production_schema_v1",
                "production model profiles require schema_version 2 XML/BIN artifact manifests",
                field="schema_version",
            )
        declaration = _ArtifactDeclaration(
            "xml",
            path,
            _expected_hash(model),
            "model.path",
            "model.sha256",
            True,
            legacy_v1=True,
        )
        result.values["model_format"] = "legacy_single_path"
        result.values["model_artifact_manifest"] = {
            "schema_version": 1,
            "format": "legacy_single_path",
            "xml": {
                "declared_path": _safe_name(path) if path else None,
                "declared_sha256": _hash_evidence(declaration.sha256),
                "required": True,
            },
            "bin": {"declared_path": None, "declared_sha256": None, "required": False},
        }
        return (declaration,)

    if schema_version != 2:
        return ()

    model_format = _require_string(model, "format", result, "model")
    if model_format != "openvino_ir":
        result.fail("model_format", "schema_version 2 model.format must be exactly openvino_ir", field="model.format", value=model_format)
    artifacts_node = model.get("artifacts")
    if not isinstance(artifacts_node, Mapping):
        result.fail("model_artifact_manifest", "model.artifacts must be an XML/BIN map", field="model.artifacts")
        return ()

    declarations: list[_ArtifactDeclaration] = []
    for name in ("xml", "bin"):
        node = artifacts_node.get(name)
        path_field = f"model.artifacts.{name}.path"
        sha_field = f"model.artifacts.{name}.sha256"
        if not isinstance(node, Mapping):
            result.fail("model_artifact_manifest", f"model.artifacts.{name} must be a map", field=f"model.artifacts.{name}")
            declaration = _ArtifactDeclaration(name, None, None, path_field, sha_field, True)
        else:
            path = _require_string(node, "path", result, f"model.artifacts.{name}")
            declared_hash = _artifact_value(node.get("sha256")) if "sha256" in node else None
            declaration = _ArtifactDeclaration(name, path, declared_hash, path_field, sha_field, True)
        declarations.append(declaration)

    xml, bin_artifact = declarations
    for artifact in declarations:
        if artifact.path is None:
            result.fail(
                _artifact_path_code(artifact, "path_missing"),
                f"schema_version 2 requires a declared {artifact.name.upper()} artifact path",
                field=artifact.path_field,
            )
        if artifact.sha256 is None:
            # Schema v2 is a manifest, not a best-effort reference.  Require
            # both member digests even for a test-only profile, so a future
            # caller cannot mistake an unbound BIN for reviewed evidence.
            result.fail(
                _artifact_hash_code(artifact, "missing"),
                f"schema_version 2 requires a declared {artifact.name.upper()} artifact SHA-256",
                field=artifact.sha256_field,
            )
    if profile_kind == "production":
        for artifact in declarations:
            if artifact.path and (_is_uri(artifact.path) or not Path(artifact.path).is_absolute()):
                result.fail(
                    "production_model_artifact_path",
                    f"production {artifact.name.upper()} artifact path must be an absolute local path",
                    field=artifact.path_field,
                )
        if xml.path and bin_artifact.path and _same_path(Path(xml.path), Path(bin_artifact.path)):
            result.fail(
                "model_artifact_path_alias",
                "production XML and BIN artifact paths must be distinct",
                field="model.artifacts",
            )

    for artifact in declarations:
        if artifact.sha256 is not None and (
            not isinstance(artifact.sha256, str) or not _SHA256_RE.fullmatch(artifact.sha256)
        ):
            result.fail(
                _artifact_hash_code(artifact, "invalid"),
                f"{artifact.name.upper()} artifact SHA-256 is not 64 hexadecimal characters",
                field=artifact.sha256_field,
            )

    result.values["model_format"] = model_format
    result.values["model_artifact_manifest"] = {
        "schema_version": 2,
        "format": model_format,
        **{
            artifact.name: {
                "declared_path": _safe_name(artifact.path) if artifact.path else None,
                "declared_sha256": _hash_evidence(artifact.sha256),
                "required": artifact.required,
            }
            for artifact in declarations
        },
    }
    return tuple(declarations)


def _audit_model_contract(
    root: Mapping[str, Any],
    result: AuditResult,
    *,
    profile_kind: Optional[str],
    mode: str,
    schema_version: Optional[int],
) -> tuple[_ArtifactDeclaration, ...]:
    model = _require_map(root, "model", result)
    if model is None:
        return ()
    result.profile_id = _require_string(model, "id", result, "model")
    result.profile_version = _require_string(model, "version", result, "model")
    source = _require_string(model, "source", result, "model")
    artifacts = _profile_artifact_declarations(
        model, result, profile_kind=profile_kind, schema_version=schema_version
    )
    xml_artifact = next((artifact for artifact in artifacts if artifact.name == "xml"), None)
    result.model_path = xml_artifact.path if xml_artifact is not None else None
    result.values["model_source"] = source
    if profile_kind == "test_only" and xml_artifact is not None and xml_artifact.path and _is_uri(xml_artifact.path):
        result.values["external_fixture_identifier"] = _safe_name(xml_artifact.path)
    if source and profile_kind == "test_only":
        lowered = source.lower()
        if "synthetic" in lowered or "legacy" in lowered or "fixture" in lowered:
            result.values["fixture_marker"] = "test_only/synthetic_fixture/not_competition_evidence"
    provenance_values = [source, result.profile_id, result.profile_version]
    provenance_values.extend(artifact.path for artifact in artifacts)
    if profile_kind == "production" and any(_looks_unreviewed(item) for item in provenance_values):
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
            if shape[0] != 1 or shape[1] != 3 or not all(_strict_int(item) and item > 0 for item in shape):
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
            if shape[0] != 1 or not all(_strict_int(item) and item > 0 for item in shape):
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
                if _strict_int(columns) and _strict_int(offset) and _strict_int(count) and (offset + count > columns or count <= 0):
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
            normalized, mapping_keys_valid = _normalize_class_mapping(
                mapping, result, field="semantics.class_to_armor_type"
            )
            mapping_valid = mapping_keys_valid
            for value in normalized.values():
                if str(value) not in {"small", "large"}:
                    result.fail("class_mapping_value", "class_to_armor_type values must be small or large", field="semantics.class_to_armor_type", value=value)
                    mapping_valid = False
            if not _strict_int(armor_count) or set(normalized) != set(range(armor_count)):
                result.fail("class_mapping_incomplete", "class_to_armor_type must map every armor class exactly once", field="semantics.class_to_armor_type")
                mapping_valid = False
            if mapping_valid:
                result.values["class_to_armor_type"] = {str(key): value for key, value in sorted(normalized.items())}

    # Keep the human-readable contract distinct from the small software golden
    # fixture.  The latter proves only deterministic preprocessing mechanics;
    # it is never a model inference or an MCU/hardware frame claim.
    if input_node is not None:
        preprocessing_evidence = software_preprocessing_evidence(input_node)
        result.values["preprocessing_contract"] = preprocessing_evidence.get("contract") or {
            "input_shape": input_node.get("shape"),
            "layout": input_node.get("layout"),
            "element_type": input_node.get("element_type"),
            "source_color_order": input_node.get("source_color_order"),
            "model_color_order": input_node.get("model_color_order"),
            "normalization": input_node.get("normalization"),
            "resize_mode": input_node.get("resize_mode"),
        }
        result.values["software_preprocessing_evidence"] = preprocessing_evidence
        for finding in preprocessing_evidence.get("findings", []):
            if not isinstance(finding, Mapping):
                result.fail("preprocess_evidence_invalid", "preprocessing evidence returned an invalid finding")
                continue
            code = str(finding.get("code", "preprocess_evidence_failure"))
            message = str(finding.get("message", "preprocessing evidence failed"))
            field = finding.get("field")
            if code == "preprocess_runtime_unavailable":
                _runtime_unavailable_finding(
                    result,
                    profile_kind=profile_kind,
                    mode=mode,
                    code=code,
                    message=message,
                )
            else:
                result.fail(code, message, field=str(field) if field else None)
    if output_node is not None:
        result.values["output_contract"] = {
            "shape": output_node.get("shape"),
            "layout": output_node.get("layout"),
            "element_type": output_node.get("element_type"),
            "keypoint_count": output_node.get("keypoint_count"),
            "objectness_index": output_node.get("objectness_index"),
            "color_logits_offset": output_node.get("color_logits_offset"),
            "color_class_count": output_node.get("color_class_count"),
            "armor_logits_offset": output_node.get("armor_logits_offset"),
            "armor_class_count": output_node.get("armor_class_count"),
        }
    if postprocess is not None:
        result.values["postprocess_contract"] = {
            "objectness_threshold": postprocess.get("objectness_threshold"),
            "nms_threshold": postprocess.get("nms_threshold"),
            "keypoint_order": postprocess.get("keypoint_order"),
        }
    if semantics is not None:
        result.values["semantic_contract"] = {
            "color_id_to_name": semantics.get("color_id_to_name"),
            "armor_class_names": semantics.get("armor_class_names"),
            "class_to_armor_type": result.values.get("class_to_armor_type", {}),
        }
    return artifacts


def _runtime_artifact_path(
    artifact: _ArtifactDeclaration,
    supplied_path: str | Path | None,
) -> Optional[Path]:
    """Choose only an explicit supplied path or a reviewed local path."""

    if supplied_path is not None:
        return Path(supplied_path)
    if artifact.path and not _is_uri(artifact.path):
        return Path(artifact.path)
    return None


def _artifact_io_state(path: Optional[Path]) -> tuple[bool, bool, bool]:
    """Return exists/readable/unsafe_alias without dereferencing on error."""

    try:
        exists = path is not None and path.is_file()
        readable = bool(exists and os.access(path, os.R_OK))
        unsafe_alias = bool(readable and _unsafe_file_alias(path))
    except OSError:
        return False, False, True
    return bool(exists), readable, unsafe_alias


def _audit_artifact_hash(
    result: AuditResult,
    artifact: _ArtifactDeclaration,
    *,
    actual_digest: Optional[str],
    expected_hash: Any,
    profile_kind: Optional[str],
    mode: str,
) -> None:
    """Check profile/external digest declarations independently for one file."""

    external_supplied = expected_hash is not _MISSING_HASH
    external_hash = _artifact_value(expected_hash) if external_supplied else _MISSING_HASH
    profile_hash = artifact.sha256
    profile_declared = profile_hash is not None
    missing_required = mode == "strict" or profile_kind == "production" or external_supplied

    # A reviewed declaration is never replaced by metadata/CLI input.  Compare
    # the two assertions even when the artifact is unavailable.
    if profile_declared and external_supplied and (
        not isinstance(profile_hash, str)
        or not isinstance(external_hash, str)
        or profile_hash.strip().lower() != external_hash.strip().lower()
    ):
        result.fail(
            _artifact_hash_code(artifact, "declaration_mismatch"),
            f"external {artifact.name.upper()} SHA-256 does not match the reviewed profile declaration",
            field=artifact.sha256_field,
        )
    if not profile_declared and missing_required:
        result.fail(
            _artifact_hash_code(artifact, "missing"),
            f"{artifact.name.upper()} artifact SHA-256 is not declared in the reviewed profile",
            field=artifact.sha256_field,
        )
    if external_supplied and (not isinstance(external_hash, str) or not external_hash):
        result.fail(
            _artifact_hash_code(artifact, "external_invalid"),
            f"external expected {artifact.name.upper()} SHA-256 must be a non-empty string",
            field="--model-sha256" if artifact.name == "xml" else "--model-bin-sha256",
        )

    declarations: list[tuple[str, Any]] = []
    if profile_declared:
        declarations.append(("profile", profile_hash))
    if external_supplied:
        declarations.append(("external", external_hash))
    for source, declared_hash in declarations:
        if not isinstance(declared_hash, str) or not _SHA256_RE.fullmatch(declared_hash):
            # Schema-v2 declaration syntax is also checked while parsing.  It
            # remains checked here for v1 compatibility and untrusted callers.
            result.fail(
                _artifact_hash_code(artifact, "invalid"),
                f"{source} {artifact.name.upper()} SHA-256 is not 64 hexadecimal characters",
                field=artifact.sha256_field,
            )
        elif actual_digest is not None and actual_digest.lower() != declared_hash.lower():
            result.fail(
                _artifact_hash_code(artifact, "mismatch"),
                f"{artifact.name.upper()} artifact SHA-256 does not match the {source} declaration",
                field=artifact.sha256_field,
            )


def audit_model_profile(
    profile_path: str | Path,
    *,
    model_path: str | Path | None = None,
    model_bin_path: str | Path | None = None,
    mode: str = "evidence_only",
    allow_test_only: bool = False,
    expected_model_sha256: Any = _MISSING_HASH,
    expected_model_bin_sha256: Any = _MISSING_HASH,
) -> AuditResult:
    """Audit a detector profile and its explicitly named OpenVINO artifacts."""

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
    profile_kind, schema_version = _check_common_profile_header(
        root, result, mode=mode, allow_test_only=allow_test_only
    )
    artifacts = _audit_model_contract(
        root,
        result,
        profile_kind=profile_kind,
        mode=mode,
        schema_version=schema_version,
    )
    artifact_by_name = {artifact.name: artifact for artifact in artifacts}
    xml_artifact = artifact_by_name.get("xml")
    if profile_kind == "production" and model_path is None:
        result.fail(
            "runtime_model_path_missing",
            "production audit requires the actual runtime XML model path",
            field="--model",
        )

    supplied_paths: dict[str, str | Path | None] = {
        "xml": model_path,
        "bin": model_bin_path,
    }
    expected_hashes: dict[str, Any] = {
        "xml": expected_model_sha256,
        "bin": expected_model_bin_sha256,
    }
    # An explicit XML/BIN input or external digest cannot be silently ignored
    # merely because the profile omitted that artifact member.  This matters
    # for schema-v1, whose lone XML reference must never be mistaken for a
    # binding of a separately supplied BIN file.
    for name in ("xml", "bin"):
        if name in artifact_by_name:
            continue
        if supplied_paths[name] is not None:
            result.fail(
                f"model_{name}_undeclared",
                f"the reviewed profile does not declare a {name.upper()} artifact",
                field="--model" if name == "xml" else "--model-bin",
            )
        if expected_hashes[name] is not _MISSING_HASH:
            external_hash = _artifact_value(expected_hashes[name])
            if not isinstance(external_hash, str) or not external_hash:
                result.fail(
                    f"model_{name}_hash_external_invalid",
                    f"external expected {name.upper()} SHA-256 must be a non-empty string",
                    field="--model-sha256" if name == "xml" else "--model-bin-sha256",
                )
            else:
                result.fail(
                    f"model_{name}_hash_undeclared",
                    f"the reviewed profile does not declare a {name.upper()} artifact SHA-256",
                    field="model.artifacts",
                )
    runtime_paths: dict[str, Optional[Path]] = {}
    artifact_records: dict[str, dict[str, Any]] = {}
    states: dict[str, tuple[bool, bool, bool]] = {}
    for artifact in artifacts:
        runtime_path = _runtime_artifact_path(artifact, supplied_paths.get(artifact.name))
        runtime_paths[artifact.name] = runtime_path
        path_match: Optional[bool] = None
        if artifact.path and not _is_uri(artifact.path) and runtime_path is not None:
            path_match = _same_path(runtime_path, Path(artifact.path))
            if profile_kind == "production" and not path_match:
                result.fail(
                    _artifact_path_code(artifact, "path_mismatch"),
                    f"runtime {artifact.name.upper()} path does not match the reviewed profile declaration",
                    field=artifact.path_field,
                )
        exists, readable, unsafe_alias = _artifact_io_state(runtime_path)
        states[artifact.name] = (exists, readable, unsafe_alias)
        artifact_records[artifact.name] = {
            "required": artifact.required,
            "declared_path": _safe_name(artifact.path) if artifact.path else None,
            "declared_sha256": _hash_evidence(artifact.sha256),
            "runtime_path": _safe_name(runtime_path) if runtime_path is not None else None,
            "runtime_path_matches_profile": path_match,
            "exists": exists,
            "readable": readable,
            "actual_sha256": None,
            "sha256_matches_profile": None,
            "sha256_matches_external": None,
        }
    for name, status in (
        ("xml", "not_declared_or_invalid_manifest"),
        ("bin", "not_declared_legacy_schema_v1"),
    ):
        artifact_records.setdefault(
            name,
            {
                "required": False,
                "declared_path": None,
                "declared_sha256": None,
                "runtime_path": None,
                "runtime_path_matches_profile": None,
                "exists": False,
                "readable": False,
                "actual_sha256": None,
                "sha256_matches_profile": None,
                "sha256_matches_external": None,
                "status": status,
            },
        )
    result.values["model_artifacts"] = artifact_records
    result.values["test_only"] = profile_kind == "test_only"
    xml_record = artifact_records.get("xml", {})
    bin_record = artifact_records.get("bin", {})
    result.values["declared_model_path"] = xml_record.get("declared_path")
    result.values["runtime_model_path"] = xml_record.get("runtime_path")
    result.values["runtime_path_matches_profile"] = xml_record.get("runtime_path_matches_profile")
    result.values["model_artifact_exists"] = xml_record.get("exists") is True
    result.values["model_artifact_readable"] = xml_record.get("readable") is True
    result.values["declared_model_sha256"] = xml_record.get("declared_sha256")
    result.values["declared_model_bin_path"] = bin_record.get("declared_path")
    result.values["runtime_model_bin_path"] = bin_record.get("runtime_path")
    result.values["runtime_model_bin_path_matches_profile"] = bin_record.get("runtime_path_matches_profile")
    result.values["model_bin_artifact_exists"] = bin_record.get("exists") is True
    result.values["model_bin_artifact_readable"] = bin_record.get("readable") is True
    result.values["declared_model_bin_sha256"] = bin_record.get("declared_sha256")

    unavailable_artifacts: list[_ArtifactDeclaration] = []
    alias_artifacts: list[_ArtifactDeclaration] = []
    for artifact in artifacts:
        _, readable, unsafe_alias = states[artifact.name]
        if not readable:
            unavailable_artifacts.append(artifact)
            _runtime_unavailable_finding(
                result,
                profile_kind=profile_kind,
                mode=mode,
                code=_artifact_path_code(artifact, "artifact_unavailable"),
                message=(
                    f"{artifact.name.upper()} artifact path is external, missing, or unreadable; "
                    "pipeline execution was not claimed"
                ),
                field=artifact.path_field,
            )
        elif unsafe_alias:
            alias_artifacts.append(artifact)
            result.fail(
                _artifact_path_code(artifact, "artifact_alias"),
                f"{artifact.name.upper()} artifact must not be a symlink or hardlink alias",
                field=artifact.path_field,
            )
    if unavailable_artifacts and not (
        len(unavailable_artifacts) == 1 and unavailable_artifacts[0].legacy_v1
    ):
        _runtime_unavailable_finding(
            result,
            profile_kind=profile_kind,
            mode=mode,
            code="model_artifact_unavailable",
            message="one or more required XML/BIN model artifacts are unavailable; pipeline execution was not claimed",
            field="model.artifacts",
        )

    hash_unavailable_artifacts: list[_ArtifactDeclaration] = []
    for artifact in artifacts:
        _, readable, unsafe_alias = states[artifact.name]
        actual_digest: Optional[str] = None
        if readable and not unsafe_alias:
            runtime_path = runtime_paths[artifact.name]
            try:
                assert runtime_path is not None
                actual_digest = _sha256(runtime_path)
                artifact_records[artifact.name]["actual_sha256"] = actual_digest
                artifact_records[artifact.name]["sha256_matches_profile"] = _digest_matches(
                    actual_digest, artifact.sha256
                )
                if expected_hashes[artifact.name] is not _MISSING_HASH:
                    artifact_records[artifact.name]["sha256_matches_external"] = _digest_matches(
                        actual_digest, expected_hashes[artifact.name]
                    )
                if artifact.name == "xml":
                    result.model_sha256 = actual_digest
                    result.values["model_artifact_sha256"] = actual_digest
                else:
                    result.values["model_artifact_bin_sha256"] = actual_digest
            except OSError:
                hash_unavailable_artifacts.append(artifact)
                artifact_records[artifact.name]["readable"] = False
                _runtime_unavailable_finding(
                    result,
                    profile_kind=profile_kind,
                    mode=mode,
                    code=_artifact_hash_code(artifact, "unreadable"),
                    message=f"{artifact.name.upper()} artifact could not be hashed",
                    field=artifact.path_field,
                )
        if (
            actual_digest is not None
            or expected_hashes[artifact.name] is not _MISSING_HASH
            or mode == "strict"
            or profile_kind == "production"
        ):
            _audit_artifact_hash(
                result,
                artifact,
                actual_digest=actual_digest,
                expected_hash=expected_hashes[artifact.name],
                profile_kind=profile_kind,
                mode=mode,
            )

    # A schema-v2 IR is eligible for an OpenVINO inspection only after every
    # required member has been cryptographically bound.  In particular, do
    # not read XML with a substituted/missing BIN merely to collect shape
    # evidence: that would reintroduce the same implicit-weight gap that this
    # manifest closes.  The legacy v1 test-only XML fixture remains a separate
    # compatibility path and cannot become production-admissible.
    unverified_manifest_artifacts = [
        artifact
        for artifact in artifacts
        if not artifact.legacy_v1
        and artifact.required
        and (
            artifact_records[artifact.name]["sha256_matches_profile"] is not True
            or (
                expected_hashes[artifact.name] is not _MISSING_HASH
                and artifact_records[artifact.name]["sha256_matches_external"] is not True
            )
        )
    ]
    if unavailable_artifacts or hash_unavailable_artifacts:
        result.values["runtime_contract"] = _runtime_contract_unavailable()
    elif alias_artifacts:
        result.values["runtime_contract"] = _runtime_contract_unavailable("model_artifact_alias")
    elif unverified_manifest_artifacts:
        result.values["runtime_contract"] = _runtime_contract_unavailable("model_artifact_unverified")
        _runtime_unavailable_finding(
            result,
            profile_kind=profile_kind,
            mode=mode,
            code="model_artifact_unverified",
            message=(
                "required XML/BIN artifact digest verification did not complete; "
                "OpenVINO model reading was not attempted"
            ),
            field="model.artifacts",
        )
    elif xml_artifact is None:
        result.values["runtime_contract"] = _runtime_contract_unavailable()
        _runtime_unavailable_finding(
            result,
            profile_kind=profile_kind,
            mode=mode,
            code="model_artifact_unavailable",
            message="no readable XML artifact was declared; runtime contract was not claimed",
            field="model.artifacts.xml.path",
        )
    else:
        runtime_xml = runtime_paths.get("xml")
        runtime_bin = runtime_paths.get("bin") if "bin" in artifact_by_name else None
        if runtime_xml is None:
            result.values["runtime_contract"] = _runtime_contract_unavailable()
        else:
            _audit_runtime_contract(
                result,
                runtime_xml,
                runtime_bin=runtime_bin,
                input_node=_mapping(root.get("input")),
                output_node=_mapping(root.get("output")),
                profile_kind=profile_kind,
                mode=mode,
            )
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
    mapping_valid = False
    if isinstance(mapping, Mapping):
        normalized_mapping, mapping_valid = _normalize_class_mapping(
            mapping, result, field="class_to_armor_type"
        )
        for value in normalized_mapping.values():
            if value not in {"small", "large"}:
                result.fail("class_mapping_value", "class_to_armor_type values must be small or large", field="class_to_armor_type")
                mapping_valid = False
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
    if mapping_valid:
        result.values["class_to_armor_type"] = {str(key): value for key, value in sorted(normalized_mapping.items())}
    result.values["schema_source"] = "docs/pnp_config_schema.md + PnpStage loader contract"
    if profile == "test_only":
        result.values["fixture_marker"] = "test_only/synthetic_fixture/not_competition_evidence"
    return result


__all__ = ["AuditResult", "Finding", "STATUS_CODES", "audit_model_profile", "audit_pnp_config", "status_code"]
