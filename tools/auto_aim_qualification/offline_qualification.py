#!/usr/bin/env python3
"""Unified, read-only qualification gate for model/PnP/offline evidence."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence

from .model_profile_audit import AuditResult, Finding, audit_model_profile, audit_pnp_config, status_code

try:
    from tools.offline_evidence_report.auto_aim_evidence_report import (
        analyze_csv,
        build_report,
        markdown_report,
        write_reports,
    )
    from tools.offline_evidence_report.offline_evidence_bundle import build_bundle, validate_manifest
    from tools.offline_evidence_report.offline_evidence_bundle import (
        _redact_text as _bundle_redact_text,
        _safe_name as _bundle_safe_name,
    )
except ImportError:  # direct script invocation from the repository root
    from pathlib import Path as _Path
    _ROOT = _Path(__file__).resolve().parents[2]
    import sys as _sys
    if str(_ROOT) not in _sys.path:
        _sys.path.insert(0, str(_ROOT))
    from tools.offline_evidence_report.auto_aim_evidence_report import (
        analyze_csv,
        build_report,
        markdown_report,
        write_reports,
    )
    from tools.offline_evidence_report.offline_evidence_bundle import build_bundle, validate_manifest
    from tools.offline_evidence_report.offline_evidence_bundle import (
        _redact_text as _bundle_redact_text,
        _safe_name as _bundle_safe_name,
    )


STATUS_CODES = {"PASS": 0, "WARN": 2, "FAIL": 1}
_SEVERITY = {"PASS": 0, "WARN": 1, "FAIL": 2}
_SAFE_METADATA_KEYS = {
    "run_id", "commit", "dataset_id", "model_profile_id", "model_profile_version",
    "model_source", "model_sha256", "pnp_profile", "pnp_profile_version", "pnp_source",
    "source_label", "run_command",
}


def _safe_name(value: Any) -> str:
    return _bundle_safe_name(value)


def _redact_text(value: Any) -> str:
    return _bundle_redact_text(str(value))


def _safe_metadata(value: Optional[Mapping[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    result: dict[str, Any] = {}
    warnings: list[dict[str, Any]] = []
    for key, raw in (value or {}).items():
        key_text = str(key)
        if key_text not in _SAFE_METADATA_KEYS:
            warnings.append({"code": "metadata_field_ignored", "message": f"metadata field {key_text!r} ignored"})
            continue
        if isinstance(raw, (dict, list, tuple)):
            warnings.append({"code": "metadata_field_ignored", "message": f"metadata field {key_text!r} must be scalar"})
            continue
        if isinstance(raw, float) and not math.isfinite(raw):
            warnings.append({"code": "metadata_field_ignored", "message": f"metadata field {key_text!r} must be finite"})
            continue
        if isinstance(raw, str):
            result[key_text] = _redact_text(raw)
        else:
            result[key_text] = raw
    return result, warnings


def _load_metadata(path: str | Path | None) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    if path is None:
        return {}, []
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        return {}, [{"code": "metadata_error", "message": f"unable to read metadata: {type(exc).__name__}"}]
    if not isinstance(value, Mapping):
        return {}, [{"code": "metadata_error", "message": "metadata JSON must contain an object"}]
    return _safe_metadata(value)


def _finding_dict(code: str, message: str, *, status: str = "FAIL", field: str | None = None, value: Any = None) -> dict[str, Any]:
    item: dict[str, Any] = {"status": status, "code": code, "message": _redact_text(message)}
    if field:
        item["field"] = field
    if value is not None:
        item["value"] = _safe_name(value) if isinstance(value, (str, Path)) else value
    return item


def _max_status(*statuses: str) -> str:
    return max(statuses or ("PASS",), key=lambda value: _SEVERITY.get(value, 2))


def _model_hash_declared(profile_path: str | Path) -> bool:
    try:
        import yaml  # type: ignore
        root = yaml.safe_load(Path(profile_path).read_text(encoding="utf-8"))
        model = root.get("model", {}) if isinstance(root, Mapping) else {}
        return any(isinstance(model.get(key), str) and model.get(key).strip() for key in ("sha256", "sha256sum", "artifact_sha256", "hash"))
    except Exception:
        return False


def _metadata_mismatch(
    findings: list[dict[str, Any]],
    metadata: Mapping[str, Any],
    model_audit: AuditResult,
    pnp_audit: AuditResult,
    *,
    mode: str,
) -> None:
    expected = {
        "model_profile_id": model_audit.profile_id,
        "model_profile_version": model_audit.profile_version,
        "pnp_profile_version": pnp_audit.profile_version,
    }
    for key, value in expected.items():
        if value and metadata.get(key) not in (None, "", value):
            findings.append(_finding_dict("metadata_identity_mismatch", f"metadata {key} does not match audited profile", field=key, value=metadata.get(key)))
    if metadata.get("pnp_profile") not in (None, "", pnp_audit.profile_kind, pnp_audit.profile_id, pnp_audit.profile_version):
        findings.append(_finding_dict("metadata_pnp_mismatch", "metadata pnp_profile does not match audited PnP config", field="pnp_profile", value=metadata.get("pnp_profile")))
    if mode == "strict":
        for key in ("run_id", "run_command", "commit", "dataset_id", "model_profile_id", "model_profile_version", "pnp_profile", "pnp_profile_version"):
            if metadata.get(key) in (None, ""):
                findings.append(_finding_dict("metadata_required", f"strict mode requires metadata field {key}", field=key))


def qualify_offline(
    *,
    model_profile: str | Path,
    pnp_config: str | Path,
    mode: str = "evidence_only",
    allow_test_only: bool = False,
    model: str | Path | None = None,
    input_csv: str | Path | None = None,
    metadata_json: str | Path | None = None,
    evidence_bundle: str | Path | None = None,
    manifest: str | Path | None = None,
    camera_intrinsic_report: str | Path | None = None,
    annotated_dir: str | Path | None = None,
    producer_command_file: str | Path | None = None,
    expected_image_size: tuple[int, int] | None = None,
    metadata: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """Run the profile→CSV→report→manifest admission checks.

    No detector or pipeline is launched by this function.  A pre-existing CSV
    is treated as evidence and is parsed by the PR #16 module.  Callers that
    have no CSV receive an explicit ``pipeline_execution_unavailable`` finding.
    """

    mode = mode if mode in {"evidence_only", "strict"} else "evidence_only"
    file_metadata, metadata_warnings = _load_metadata(metadata_json)
    merged_metadata: dict[str, Any] = dict(file_metadata)
    if metadata:
        supplied, supplied_warnings = _safe_metadata(metadata)
        merged_metadata.update(supplied)
        metadata_warnings.extend(supplied_warnings)
    metadata = merged_metadata
    findings: list[dict[str, Any]] = []
    for item in metadata_warnings:
        status = "FAIL" if item.get("code") == "metadata_error" else "WARN"
        findings.append({"status": status, **item})
    if mode == "strict" and metadata_json is None:
        findings.append(_finding_dict("metadata_missing", "strict mode requires metadata JSON"))
    if mode == "strict" and not metadata:
        findings.append(_finding_dict("metadata_empty", "strict mode requires non-empty metadata JSON"))

    model_audit = audit_model_profile(
        model_profile,
        model_path=model,
        mode=mode,
        allow_test_only=allow_test_only,
        expected_model_sha256=str(metadata.get("model_sha256")) if metadata.get("model_sha256") else None,
    )
    model_mapping = model_audit.values.get("class_to_armor_type")
    expected_mapping = None
    if isinstance(model_mapping, Mapping):
        expected_mapping = {int(key): str(value) for key, value in model_mapping.items()}
    pnp_audit = audit_pnp_config(
        pnp_config,
        mode=mode,
        allow_test_only=allow_test_only,
        expected_image_size=expected_image_size,
        expected_class_mapping=expected_mapping,
    )
    model_report = model_audit.as_dict()
    pnp_report = pnp_audit.as_dict()
    findings.extend(model_report["findings"])
    findings.extend(pnp_report["findings"])
    if any(item.get("code") in {"model_artifact_unavailable", "model_artifact_missing", "model_hash_unreadable"} for item in model_report["findings"]):
        findings.append(_finding_dict(
            "pipeline_execution_unavailable",
            "pipeline execution unavailable: model/OpenVINO runtime asset is unavailable; no detector replay was claimed",
            status="FAIL" if mode == "strict" else "WARN",
        ))

    csv_analysis = None
    if input_csv is None:
        findings.append(_finding_dict("pipeline_execution_unavailable", "pipeline execution unavailable: no CSV supplied", status="FAIL" if mode == "strict" else "WARN"))
        csv_report: dict[str, Any] = {"status": "FAIL" if mode == "strict" else "WARN", "report_status": "FAIL" if mode == "strict" else "WARN", "errors": [], "warnings": [{"code": "pipeline_execution_unavailable", "message": "no detector replay was executed"}], "evidence_boundary": {"real_hit_rate_computed": False, "hardware_validation": False, "gimbal_closed_loop_validated": False, "firing_validated": False}}
    else:
        csv_analysis = analyze_csv(input_csv, {key: value for key, value in metadata.items() if key in {"commit", "model_profile_id", "model_profile_version", "pnp_profile", "dataset_id", "source_label"}})
        csv_report = build_report(csv_analysis)
        csv_status = str(csv_report.get("status", "FAIL"))
        if csv_status == "FAIL":
            findings.append(_finding_dict("csv_report_fail", "PR #16 CSV evidence report is FAIL"))
        elif csv_status == "WARN":
            findings.append(_finding_dict("csv_report_warn", "PR #16 CSV evidence report is WARN", status="FAIL" if mode == "strict" else "WARN"))

    bundle_report: Optional[dict[str, Any]] = None
    manifest_status_value: Optional[str] = None
    if evidence_bundle is not None:
        if input_csv is not None:
            try:
                bundle_report = build_bundle(
                    input_csv,
                    evidence_bundle,
                    metadata_json=metadata_json,
                    mode=mode,
                    camera_intrinsic_report=camera_intrinsic_report,
                    model_profile=model_profile,
                    pnp_config=pnp_config,
                    annotated_dir=annotated_dir,
                    producer_command_file=producer_command_file,
                )
                bundle_status = str(bundle_report.get("status", "FAIL"))
                if bundle_status == "FAIL":
                    findings.append(_finding_dict("manifest_fail", "PR #17 evidence bundle build is FAIL"))
                elif bundle_status == "WARN":
                    findings.append(_finding_dict("manifest_warn", "PR #17 evidence bundle build is WARN", status="FAIL" if mode == "strict" else "WARN"))
            except Exception as exc:
                bundle_report = {"status": "FAIL"}
                findings.append(_finding_dict("manifest_exception", f"PR #17 evidence bundle build failed: {type(exc).__name__}"))
        else:
            findings.append(_finding_dict("manifest_missing", "evidence bundle cannot be built without input CSV", status="FAIL" if mode == "strict" else "WARN"))
    if manifest is not None:
        bundle_root = Path(evidence_bundle) if evidence_bundle is not None else Path(manifest).parent
        bundle_verification = validate_manifest(bundle_root, manifest)
        manifest_status_value = str(bundle_verification.get("status", "FAIL"))
        bundle_report = bundle_report or {"status": bundle_verification.get("status", "FAIL")}
        if bundle_verification.get("status") == "FAIL":
            findings.append(_finding_dict("manifest_fail", "PR #17 evidence manifest verification is FAIL"))
        elif bundle_verification.get("status") == "WARN":
            findings.append(_finding_dict("manifest_warn", "PR #17 evidence manifest verification is WARN", status="FAIL" if mode == "strict" else "WARN"))
    elif evidence_bundle is None:
        findings.append(_finding_dict("manifest_missing", "PR #17 evidence bundle/manifest was not supplied", status="FAIL" if mode == "strict" else "WARN"))

    if mode == "strict":
        if model_audit.profile_kind != "production":
            findings.append(_finding_dict("strict_requires_production_model", "strict mode requires profile: production"))
        if pnp_audit.profile_kind != "production":
            findings.append(_finding_dict("strict_requires_production_pnp", "strict mode requires production PnP/calibration"))
        if model is None:
            findings.append(_finding_dict("model_missing", "strict mode requires an actual model artifact"))
        if not _model_hash_declared(model_profile) and not metadata.get("model_sha256"):
            findings.append(_finding_dict("model_hash_missing", "strict mode requires a declared model SHA-256 in the profile"))
        if camera_intrinsic_report is None:
            findings.append(_finding_dict("camera_report_missing", "strict mode requires a camera intrinsic evidence report"))
        elif not Path(camera_intrinsic_report).is_file():
            findings.append(_finding_dict("camera_report_missing", "strict camera intrinsic evidence report is missing or unreadable", field="camera_intrinsic_report"))
    if allow_test_only and (model_audit.profile_kind == "test_only" or pnp_audit.profile_kind == "test_only"):
        findings.append(_finding_dict("test_only_evidence", "test-only model/PnP is explicitly allowed for evidence_only only", status="FAIL" if mode == "strict" else "WARN"))
    _metadata_mismatch(findings, metadata, model_audit, pnp_audit, mode=mode)
    for key, audited_value in (("model_source", model_audit.values.get("model_source")), ("pnp_source", pnp_audit.values.get("pnp_source"))):
        if metadata.get(key) not in (None, "", audited_value):
            findings.append(_finding_dict("metadata_source_mismatch", f"metadata {key} does not match audited profile source", field=key, value=metadata.get(key)))

    # Fixed, fail-closed boundary; never derive production readiness from CSV.
    safety_boundary = {
        "production_ready": False,
        "hardware_validation": False,
        "gimbal_closed_loop_validated": False,
        "firing_validated": False,
        "real_hit_rate_computed": False,
        "serial_enabled": False,
        "dry_run": True,
        "allow_fire": False,
        "fire_command": 0,
        "yaw_vel": 0,
        "pitch_vel": 0,
        "yaw_acc": 0,
        "pitch_acc": 0,
    }
    status = "PASS"
    for item in findings:
        item_status = str(item.get("status", "FAIL"))
        status = _max_status(status, item_status)
    # A complete evidence-only run with approved test fixtures is explicitly a
    # warning, even when PR #16/#17 themselves happen to report PASS.
    if mode == "evidence_only" and (model_audit.profile_kind == "test_only" or pnp_audit.profile_kind == "test_only"):
        status = _max_status(status, "WARN")
    result = {
        "schema_version": 1,
        "status": status,
        "mode": mode,
        "dataset_id": metadata.get("dataset_id"),
        "commit": metadata.get("commit"),
        "model_profile_id": model_audit.profile_id,
        "model_profile_version": model_audit.profile_version,
        "model_file": model_report.get("model_path"),
        "model_sha256": model_report.get("model_sha256"),
        "pnp_profile": pnp_audit.profile_id or pnp_audit.profile_kind,
        "pnp_profile_version": pnp_audit.profile_version,
        "camera_resolution": pnp_report.get("values", {}).get("camera_resolution"),
        "production_ready": False,
        "hardware_validation": False,
        "gimbal_closed_loop_validated": False,
        "firing_validated": False,
        "real_hit_rate_computed": False,
        "profile_audit": model_report,
        "pnp_audit": pnp_report,
        "csv_report_status": csv_report.get("status"),
        "csv_report": csv_report,
        "coverage": csv_report.get("coverage"),
        "detection_count": csv_report.get("detection_pnp", {}).get("detection_count"),
        "valid_pnp_count": csv_report.get("detection_pnp", {}).get("valid_pnp_count"),
        "pnp_failure_distribution": csv_report.get("detection_pnp", {}).get("pnp_failure_distribution"),
        "reprojection_error_px": csv_report.get("detection_pnp", {}).get("reprojection_error_px"),
        "tracker_target": csv_report.get("tracker_target"),
        "safety": csv_report.get("safety"),
        "absolute_command_valid": {
            str(value): sum(1 for record in (csv_analysis.records if csv_analysis is not None else []) if record.values.get("absolute_command_valid") is value)
            for value in (True, False)
        } if csv_analysis is not None else {},
        "fire_command_distribution": csv_report.get("safety", {}).get("fire_command_distribution"),
        "target_lock": csv_report.get("tracker_target", {}).get("target_lock_frame_counts"),
        "tracking_state": csv_report.get("tracker_target", {}).get("tracking_state_distribution"),
        "pipeline": {
            "executed": False,
            "status": "unavailable" if any(item.get("code") == "pipeline_execution_unavailable" for item in findings) else "not_invoked",
            "execution_claimed": False,
            "chain": ["OpenVinoYoloDetector", "RawArmorDetection", "PnpStage", "OfflineTracker", "TargetSelector", "SafeOfflineAimer", "CSV"],
        },
        "validators": {
            "runtime_contract": "C++ load_model_profile/load_pnp_configuration remain normative",
            "python_preflight": "schema mirror plus provenance/hash/path admission",
            "csv_report": "reused PR #16 analyze_csv/build_report",
            "evidence_bundle": "reused PR #17 build_bundle/validate_manifest",
        },
        "bundle_status": (bundle_report or {}).get("status"),
        "manifest_status": manifest_status_value if manifest_status_value is not None else (bundle_report or {}).get("status"),
        "metadata": metadata,
        "safety_boundary": safety_boundary,
        "diagnostics": {
            "errors": [item for item in findings if item.get("status") == "FAIL" or item.get("status") is None],
            "warnings": [item for item in findings if item.get("status") == "WARN"],
        },
        "unconfirmed_items": [
            "production detector model semantics and provenance",
            "production camera intrinsic calibration and measured armor geometry",
            "camera_to_gimbal extrinsic calibration",
            "absolute yaw/pitch mechanical zero",
            "Orin, real camera, serial, gimbal closed loop, and firing validation",
        ],
        "run_command": metadata.get("run_command"),
        "run_identity": {
            key: metadata.get(key)
            for key in ("run_id", "commit", "dataset_id", "model_profile_id", "model_profile_version", "pnp_profile", "pnp_profile_version")
            if metadata.get(key) not in (None, "")
        },
        "qualification_summary": {
            "dataset_id": metadata.get("dataset_id"),
            "commit": metadata.get("commit"),
            "model_profile_id": model_audit.profile_id,
            "model_profile_version": model_audit.profile_version,
            "model_file": model_report.get("model_path"),
            "model_sha256": model_report.get("model_sha256"),
            "pnp_profile": pnp_audit.profile_id or pnp_audit.profile_kind,
            "pnp_profile_version": pnp_audit.profile_version,
            "camera_resolution": pnp_report.get("values", {}).get("camera_resolution"),
            "detection_count": csv_report.get("detection_pnp", {}).get("detection_count"),
            "valid_pnp_count": csv_report.get("detection_pnp", {}).get("valid_pnp_count"),
            "pnp_failure_distribution": csv_report.get("detection_pnp", {}).get("pnp_failure_distribution"),
            "reprojection_error_px": csv_report.get("detection_pnp", {}).get("reprojection_error_px"),
            "tracker_target": csv_report.get("tracker_target"),
            "safety": csv_report.get("safety"),
            "coverage": csv_report.get("coverage"),
            "absolute_command_valid": {
                str(value): sum(1 for record in (csv_analysis.records if csv_analysis is not None else []) if record.values.get("absolute_command_valid") is value)
                for value in (True, False)
            } if csv_analysis is not None else {},
            "fire_command_distribution": csv_report.get("safety", {}).get("fire_command_distribution"),
            "selected_frames": csv_report.get("tracker_target", {}).get("selected_frames"),
            "target_lock_49_frames": csv_report.get("tracker_target", {}).get("target_lock_49_frames"),
            "target_lock_50_frames": csv_report.get("tracker_target", {}).get("target_lock_50_frames"),
        },
    }
    return result


def render_markdown(report: Mapping[str, Any]) -> str:
    status = report.get("status", "FAIL")
    lines = [
        "# Offline model qualification report",
        "",
        f"**Status:** `{status}`  ",
        f"**Mode:** `{report.get('mode', 'evidence_only')}`",
        "",
        "This is a read-only admission report. It never connects to ROS, OpenVINO, a camera, serial, a gimbal, or a shooter.",
        "",
        "## Fixed safety boundary",
        "",
        "```yaml",
    ]
    for key, value in report.get("safety_boundary", {}).items():
        if isinstance(value, bool):
            value = str(value).lower()
        lines.append(f"{key}: {value}")
    lines.extend(["```", "", "## Profile and evidence status", ""])
    lines.append(f"- model profile: `{report.get('profile_audit', {}).get('status', 'FAIL')}`")
    lines.append(f"- PnP config: `{report.get('pnp_audit', {}).get('status', 'FAIL')}`")
    lines.append(f"- PR #16 CSV report: `{report.get('csv_report_status')}`")
    lines.append(f"- PR #17 evidence bundle/manifest: `{report.get('manifest_status')}`")
    lines.append("")
    errors = report.get("diagnostics", {}).get("errors", [])
    warnings = report.get("diagnostics", {}).get("warnings", [])
    for label, values in (("Errors", errors), ("Warnings", warnings)):
        lines.extend([f"## {label}", ""])
        if not values:
            lines.append("- None")
        else:
            for item in values:
                lines.append(f"- `{item.get('code', 'diagnostic')}`: {_redact_text(item.get('message', ''))}")
        lines.append("")
    lines.extend(["## Unconfirmed items", ""])
    lines.extend(f"- {item}" for item in report.get("unconfirmed_items", []))
    lines.append("")
    return "\n".join(lines)


__all__ = ["qualify_offline", "render_markdown", "status_code"]
