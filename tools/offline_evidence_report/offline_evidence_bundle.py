#!/usr/bin/env python3
"""Build and verify a reproducible, read-only offline auto-aim evidence bundle.

This module is intentionally standard-library-only.  It reuses
``auto_aim_evidence_report`` for CSV parsing and statistics, then packages the
source CSV, reports, and optional offline evidence files under one directory.
It never connects to ROS, OpenVINO, a camera, serial hardware, or a robot.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import stat
import sys
from pathlib import Path, PurePosixPath
from typing import Any, Iterable, Mapping, Optional, Sequence

try:  # Direct script execution has no package context.
    from .auto_aim_evidence_report import analyze_csv, build_report, markdown_report
except ImportError:  # pragma: no cover - exercised by the CLI invocation.
    from auto_aim_evidence_report import analyze_csv, build_report, markdown_report


SCHEMA_VERSION = 1
MODES = {"evidence_only", "strict"}
STATUS_CODES = {"PASS", "WARN", "FAIL"}
REQUIRED_ROLES = ("input_csv", "csv_report_json", "csv_report_markdown")
STRICT_ROLES = (
    "run_metadata",
    "camera_intrinsic_report",
    "model_profile",
    "pnp_config",
)
STRICT_METADATA_KEYS = (
    "run_id",
    "commit",
    "dataset_id",
    "model_profile_id",
    "pnp_profile",
    "run_command",
)
OPTIONAL_ARGUMENTS = {
    "camera_intrinsic_report": "camera_intrinsic_report",
    "model_profile": "model_profile",
    "pnp_config": "pnp_config",
    "annotated_dir": "annotated",
    "producer_command_file": "producer_command",
}
SAFE_METADATA_KEYS = {
    "run_id",
    "commit",
    "dataset_id",
    "model_profile_id",
    "model_profile_version",
    "pnp_profile",
    "source_label",
    "run_command",
    "command",
}
CSV_REPORT_METADATA_KEYS = {
    "commit",
    "model_profile_id",
    "model_profile_version",
    "pnp_profile",
    "dataset_id",
    "source_label",
}
_SENSITIVE_NAME = (
    r"(?:access[-_]?token|refresh[-_]?token|auth[-_]?token|"
    r"client[-_]?secret|api[-_]?key|private[-_]?key|"
    r"password|passwd|secret|credential|token)"
)
SENSITIVE_KEY_RE = re.compile(_SENSITIVE_NAME, re.I)
_SENSITIVE_QUOTED_RE = re.compile(
    rf'''(?ix)
    (?P<prefix>(?<![\w-])(?:--?|/)?{_SENSITIVE_NAME}(?![\w-])\s*(?:=|:|\s+)\s*)
    (?P<quote>["']) (?P<value>[^\r\n]*?) (?P=quote)
    '''
)
_SENSITIVE_UNQUOTED_RE = re.compile(
    rf'''(?ix)
    (?P<prefix>(?<![\w-])(?:--?|/)?{_SENSITIVE_NAME}(?![\w-])\s*(?:=|:|\s+)\s*)
    (?P<value>[^\s"';,\r\n]+)
    '''
)
_BEARER_QUOTED_RE = re.compile(r'''(?ix)(?P<prefix>\bbearer\s+)(?P<quote>["'])(?P<value>[^\r\n]*?)(?P=quote)''')
_BEARER_UNQUOTED_RE = re.compile(r'''(?ix)(?P<prefix>\bbearer\s+)(?P<value>[^\s"';,\r\n]+)''')
_ABSOLUTE_PATH_PREFIX = r"(?:/[A-Za-z0-9_~.-]|//|[A-Za-z]:[\\/]|\\\\)"
_QUOTED_ABSOLUTE_PATH_RE = re.compile(
    rf'''(?P<quote>["'])(?P<path>{_ABSOLUTE_PATH_PREFIX}[^\r\n]*?)(?P=quote)'''
)
_UNQUOTED_ABSOLUTE_PATH_RE = re.compile(
    rf'''(?<![\w])(?P<path>{_ABSOLUTE_PATH_PREFIX}[^\r\n"'`;|&,]+?)(?=\s+(?:--?[A-Za-z0-9_]|/[A-Za-z0-9_]|[A-Za-z]:[\\/]|\\\\)|[;|&,]|$)'''
)
HASH_RE = re.compile(r"^[0-9a-fA-F]{64}$")


def _safe_name(value: Any) -> str:
    """Return only a basename, including for Windows paths on Linux."""
    text = str(value or "").replace("\\", "/")
    return text.rstrip("/").rsplit("/", 1)[-1]


def _redact_text(value: str) -> str:
    """Remove absolute paths and sensitive command arguments from text."""
    text = str(value)
    # Handle quoted values before unquoted values so ``--client-secret
    # "multi word"`` cannot leak the tail of the quoted secret.  Keep the
    # option spelling and replace only the value for a useful command record.
    text = _SENSITIVE_QUOTED_RE.sub(lambda match: f"{match.group('prefix')}<redacted>", text)
    text = _BEARER_QUOTED_RE.sub(lambda match: f"{match.group('prefix')}<redacted>", text)
    text = _SENSITIVE_UNQUOTED_RE.sub(lambda match: f"{match.group('prefix')}<redacted>", text)
    text = _BEARER_UNQUOTED_RE.sub(lambda match: f"{match.group('prefix')}<redacted>", text)
    text = re.sub(r"(?i)(ghp_|github_pat_|sk-|xox[baprs]-)[A-Za-z0-9_\-]+", "<redacted>", text)

    def replace_path(match: re.Match[str]) -> str:
        candidate = match.group("path").rstrip()
        basename = _safe_name(candidate)
        # A path basename can itself contain a credential-like marker
        # (e.g. ``WINDOW_SECRET.csv``).  Preserve harmless basenames for
        # traceability, but never echo a sensitive-looking filename.
        if SENSITIVE_KEY_RE.search(basename):
            basename = "<redacted>"
        return "<path>/" + basename

    # Redact quoted paths first, then unquoted drive/UNC paths.  The unquoted
    # expression consumes spaces until a shell option or delimiter, preventing
    # ``C:\\Program Files\\secret.csv`` from leaking ``Program Files``.
    text = _QUOTED_ABSOLUTE_PATH_RE.sub(replace_path, text)
    return _UNQUOTED_ABSOLUTE_PATH_RE.sub(replace_path, text)


def _safe_metadata(metadata: Mapping[str, Any]) -> tuple[dict[str, Any], list[str]]:
    safe: dict[str, Any] = {}
    diagnostics: list[str] = []
    for key, value in metadata.items():
        key_text = str(key)
        if key_text not in SAFE_METADATA_KEYS or SENSITIVE_KEY_RE.search(key_text):
            diagnostics.append(f"ignored metadata field: {key_text}")
            continue
        if isinstance(value, (dict, list, tuple)):
            diagnostics.append(f"ignored non-scalar metadata field: {key_text}")
            continue
        if value is None:
            safe[key_text] = None
        elif isinstance(value, str):
            safe[key_text] = _redact_text(value)
        elif isinstance(value, (int, float, bool)):
            safe[key_text] = value
        else:
            safe[key_text] = _redact_text(str(value))
    if "command" in safe and "run_command" not in safe:
        safe["run_command"] = safe.pop("command")
    return dict(sorted(safe.items())), diagnostics


def _redact_data(value: Any) -> Any:
    """Recursively redact strings in reused CSV reports before packaging."""
    if isinstance(value, Mapping):
        return {key: _redact_data(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_redact_data(child) for child in value]
    if isinstance(value, tuple):
        return [_redact_data(child) for child in value]
    if isinstance(value, str):
        return _redact_text(value)
    return value


def _load_metadata(path: Optional[Path]) -> tuple[dict[str, Any], list[str]]:
    if path is None:
        return {}, ["metadata JSON was not supplied"]
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return {}, [f"unable to read metadata JSON: {exc}"]
    if not isinstance(value, dict):
        return {}, ["metadata JSON must contain an object"]
    return _safe_metadata(value)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _resolved_under(path: Path, root: Path) -> bool:
    try:
        path.resolve(strict=False).relative_to(root.resolve(strict=False))
        return True
    except (ValueError, OSError, RuntimeError):
        return False


def _normal_relpath(value: Any) -> Optional[str]:
    if not isinstance(value, str) or not value.strip():
        return None
    raw = value.replace("\\", "/")
    if raw.startswith("//") or re.match(r"^[A-Za-z]:", raw):
        return None
    pure = PurePosixPath(raw)
    if pure.is_absolute() or any(part in {"", ".", ".."} for part in pure.parts):
        return None
    return pure.as_posix()


def _artifact(role: str, path: Path, bundle_root: Path) -> dict[str, Any]:
    relative = path.resolve(strict=False).relative_to(bundle_root.resolve(strict=False)).as_posix()
    return {
        "role": role,
        "path": relative,
        "size_bytes": path.stat().st_size,
        "sha256": _sha256(path),
    }


def _same_path_or_file(left: Path, right: Path) -> bool:
    """Return true when two paths name the same path or filesystem object.

    ``Path.resolve`` catches lexical aliases and symlinks even when the target
    does not exist yet.  ``samefile`` additionally catches hardlinks, which is
    the important case for a destination that would otherwise truncate an
    input file (or a file outside the bundle) through a second directory
    entry.
    """
    try:
        if left.resolve(strict=False) == right.resolve(strict=False):
            return True
    except (OSError, RuntimeError):
        pass
    try:
        return left.exists() and right.exists() and os.path.samefile(left, right)
    except (OSError, ValueError):
        return False


def _lexical_relative(path: Path, root: Path) -> Optional[Path]:
    """Return a lexical relative path without following symlinks."""
    try:
        return path.absolute().relative_to(root.absolute())
    except (ValueError, OSError):
        return None


def _ensure_output_parent(destination: Path, bundle_root: Path) -> None:
    """Create/check destination parents without traversing symlink aliases."""
    root = bundle_root
    _ensure_bundle_root(root)

    relative = _lexical_relative(destination, root)
    if relative is None or not relative.parts:
        raise OSError("destination is outside bundle output directory")
    current = root
    for part in relative.parts[:-1]:
        current = current / part
        if current.is_symlink():
            raise OSError(f"destination parent is a symlink: {part}")
        if current.exists():
            if not current.is_dir():
                raise OSError(f"destination parent is not a directory: {part}")
        else:
            # Create one component at a time so a symlink cannot be silently
            # accepted by ``mkdir(parents=True)`` between checks.
            current.mkdir()
        if current.is_symlink():
            raise OSError(f"destination parent became a symlink: {part}")


def _ensure_bundle_root(root: Path) -> None:
    """Create the bundle root only when it is a real directory."""
    root = Path(root)
    absolute = Path(os.path.abspath(os.fspath(root)))
    # Do not create through a symlinked ancestor.  A check of ``root`` alone
    # is insufficient for ``link-to-external/new-bundle`` because mkdir would
    # otherwise create the bundle outside the caller's intended tree.
    for ancestor in (absolute, *absolute.parents):
        try:
            info = os.lstat(ancestor)
        except FileNotFoundError:
            continue
        except OSError as exc:
            raise OSError(f"unable to inspect output path component: {ancestor}: {exc}") from exc
        if stat.S_ISLNK(info.st_mode):
            raise OSError(f"output path contains a symlink component: {ancestor.name or ancestor}")
        if ancestor != absolute and not stat.S_ISDIR(info.st_mode):
            raise OSError(f"output path component is not a directory: {ancestor}")
    if root.is_symlink():
        raise OSError("bundle output directory must not be a symlink")
    if root.exists() and not root.is_dir():
        raise OSError("bundle output path is not a directory")
    root.mkdir(parents=True, exist_ok=True)
    if root.is_symlink() or not root.is_dir():
        raise OSError("bundle output directory is not a real directory")


def _ensure_empty_output_root(root: Path) -> None:
    """Allow only a new or genuinely empty output directory.

    Reusing a directory is ambiguous: old optional artifacts can survive a
    subsequent run and become undeclared evidence.  Refusing the run before
    creating any generated file is safer than trying to infer which files are
    ours or deleting user data.  The normal per-file gates remain in place for
    symlink/hardlink races and for callers of the lower-level helpers.
    """
    root = Path(root)
    existed = root.exists() or root.is_symlink()
    _ensure_bundle_root(root)
    if existed:
        try:
            with os.scandir(root) as entries:
                first = next(entries, None)
        except OSError as exc:
            raise OSError(f"unable to inspect existing output directory: {exc}") from exc
        if first is not None:
            raise OSError("output directory must be empty; refusing to reuse stale bundle files")


def _prepare_output_destination(
    destination: Path,
    bundle_root: Path,
    protected_sources: Iterable[Path] = (),
) -> None:
    """Reject aliases before any operation can write through them.

    Existing regular files with one link are intentionally replaceable: this
    is what makes rebuilding a bundle deterministic.  Symlinks, hardlinks,
    directories and non-regular files are never replaceable because doing so
    could mutate a path outside the bundle or an input alias.
    """
    _ensure_output_parent(destination, bundle_root)
    if destination.is_symlink():
        raise OSError(f"destination is a symlink: {destination.name}")
    if destination.exists():
        try:
            info = destination.stat()
        except OSError as exc:
            raise OSError(f"unable to stat destination {destination.name}: {exc}") from exc
        if not stat.S_ISREG(info.st_mode):
            raise OSError(f"destination is not a regular file: {destination.name}")
        if info.st_nlink > 1:
            raise OSError(f"destination is a hardlink alias: {destination.name}")
    for source in protected_sources:
        if _same_path_or_file(Path(source), destination):
            raise OSError(f"destination aliases input/source: {destination.name}")


def _open_output_file(destination: Path, bundle_root: Path, protected_sources: Iterable[Path] = ()):
    """Open a verified output file without following a destination symlink."""
    _prepare_output_destination(destination, bundle_root, protected_sources)
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    nofollow = getattr(os, "O_NOFOLLOW", 0)
    if nofollow:
        flags |= nofollow
    try:
        return os.fdopen(os.open(destination, flags, 0o600), "wb")
    except OSError as exc:
        raise OSError(f"unable to open safe destination {destination.name}: {exc}") from exc


def _copy_file(source: Path, destination: Path, *, bundle_root: Optional[Path] = None, protected_sources: Iterable[Path] = ()) -> None:
    """Copy a file only after validating the destination and all aliases."""
    source = Path(source)
    destination = Path(destination)
    if bundle_root is None:
        bundle_root = destination.parent
    # Reading the source is safe, but a same-file source/destination must never
    # be treated as a harmless no-op: it is an output/input collision that the
    # caller should report and keep unchanged.
    if _same_path_or_file(source, destination):
        raise OSError("source and destination alias the same file")
    _prepare_output_destination(destination, bundle_root, tuple(protected_sources) + (source,))
    try:
        with source.open("rb") as src, _open_output_file(destination, bundle_root, tuple(protected_sources) + (source,)) as dst:
            shutil.copyfileobj(src, dst)
    except OSError:
        raise


def _copy_text_redacted(source: Path, destination: Path, *, bundle_root: Optional[Path] = None, protected_sources: Iterable[Path] = ()) -> None:
    if bundle_root is None:
        bundle_root = destination.parent
    text = source.read_text(encoding="utf-8")
    if _same_path_or_file(Path(source), Path(destination)):
        raise OSError("source and destination alias the same file")
    with _open_output_file(Path(destination), bundle_root, tuple(protected_sources) + (Path(source),)) as handle:
        handle.write(_redact_text(text).encode("utf-8"))


def _ensure_output_directory(destination: Path, bundle_root: Path) -> None:
    """Create an output subdirectory while rejecting symlink aliases."""
    destination = Path(destination)
    _ensure_output_parent(destination / ".codex_output_probe", bundle_root)
    if destination.is_symlink():
        raise OSError(f"destination directory is a symlink: {destination.name}")
    if destination.exists() and not destination.is_dir():
        raise OSError(f"destination path is not a directory: {destination.name}")
    if not destination.exists():
        destination.mkdir()
    if destination.is_symlink() or not destination.is_dir():
        raise OSError(f"destination directory is not a real directory: {destination.name}")


def _copy_annotated(
    source: Path,
    destination: Path,
    *,
    bundle_root: Optional[Path] = None,
    protected_sources: Iterable[Path] = (),
) -> list[Path]:
    if not source.is_dir():
        raise OSError(f"annotated path is not a directory: {_safe_name(source)}")
    if bundle_root is None:
        bundle_root = destination.parent
    if _same_path_or_file(source, destination) or _resolved_under(destination, source) or _resolved_under(source, destination):
        raise OSError("annotated source aliases or contains its bundle destination")
    _ensure_output_directory(destination, bundle_root)
    copied: list[Path] = []
    for item in sorted(source.rglob("*"), key=lambda p: p.as_posix()):
        if item.is_symlink():
            raise OSError(f"symlink is not allowed in annotated evidence: {_safe_name(item)}")
        if not item.is_file():
            continue
        if item.suffix.lower() != ".png":
            raise OSError(f"annotated evidence must contain PNG files only: {_safe_name(item)}")
        relative = item.relative_to(source)
        destination_file = destination / relative
        if not _resolved_under(destination_file, destination):
            raise OSError(f"annotated path escapes bundle: {relative}")
        _copy_file(item, destination_file, bundle_root=bundle_root, protected_sources=protected_sources)
        copied.append(destination_file)
    return copied


def _diagnostic(code: str, message: str, **extra: Any) -> dict[str, Any]:
    value: dict[str, Any] = {"code": code, "message": message}
    value.update(extra)
    return value


def _bundle_status(errors: list[dict[str, Any]], warnings: list[dict[str, Any]]) -> str:
    return "FAIL" if errors else ("WARN" if warnings else "PASS")


def _boundary() -> dict[str, Any]:
    return {
        "production_ready": False,
        "hardware_validation": False,
        "gimbal_closed_loop_validated": False,
        "firing_validated": False,
        "real_hit_rate_computed": False,
        "serial_enabled": False,
        "dry_run": True,
        "allow_fire": False,
        "fire_command": 0,
    }


UNCONFIRMED_ITEMS = (
    "production detector model semantics and provenance",
    "production camera intrinsic calibration and measured armor geometry",
    "camera_to_gimbal extrinsic calibration",
    "absolute yaw/pitch mechanical zero",
    "Orin, real camera, serial, gimbal closed loop, and firing validation",
)


def _check_boundary(csv_report: Mapping[str, Any], errors: list[dict[str, Any]]) -> None:
    boundary = csv_report.get("evidence_boundary")
    if not isinstance(boundary, Mapping):
        errors.append(_diagnostic("missing_evidence_boundary", "CSV report has no evidence boundary"))
    else:
        for key in ("real_hit_rate_computed", "hardware_validation", "gimbal_closed_loop_validated", "firing_validated"):
            if boundary.get(key) is not False:
                errors.append(_diagnostic("unsafe_boundary", f"CSV report boundary {key} is not false"))
    safety = csv_report.get("safety", {})
    if isinstance(safety, Mapping):
        if safety.get("anomalies"):
            errors.append(_diagnostic("csv_safety_anomaly", "CSV report contains safety anomalies", anomalies=safety["anomalies"]))
        if safety.get("action_triggered"):
            errors.append(_diagnostic("action_triggered", "CSV report claims an action was triggered"))
        if safety.get("nonzero_fire_command_count", 0):
            errors.append(_diagnostic("nonzero_fire_command", "CSV report contains non-zero fire_command"))
        if safety.get("test_only_false_frames"):
            errors.append(_diagnostic("test_only_false", "CSV report contains test_only=false"))
        for key, expected in (("serial_enabled", False), ("dry_run", True), ("allow_fire", False)):
            if key in safety and safety.get(key) is not expected:
                errors.append(_diagnostic("unsafe_runtime_config", f"CSV report safety field {key} is not {expected!r}"))

    def unsafe_claims(value: Any, prefix: str = "") -> Iterable[str]:
        if isinstance(value, Mapping):
            for key, child in value.items():
                name = f"{prefix}.{key}" if prefix else str(key)
                lowered = str(key).lower()
                if any(token in lowered for token in ("hit_rate", "hardware_validation", "gimbal_closed_loop_validated", "firing_validated", "production_ready")) and child is True:
                    yield name
                yield from unsafe_claims(child, name)
        elif isinstance(value, list):
            for index, child in enumerate(value):
                yield from unsafe_claims(child, f"{prefix}[{index}]")

    claims = list(unsafe_claims(csv_report))
    if claims:
        errors.append(_diagnostic("unsafe_capability_claim", "report claims unverified production or hardware capability", fields=claims))


def _check_calibration_promotion(source: Path, errors: list[dict[str, Any]]) -> None:
    try:
        text = source.read_text(encoding="utf-8").lower()
    except (OSError, UnicodeError):
        return
    evidence_profile = re.search(r"profile\s*:\s*['\"]?(?:evidence_only|test_only)\b", text)
    production_claim = re.search(r"production_ready\s*:\s*(?:true|yes|1)\b", text)
    if evidence_profile and production_claim:
        errors.append(_diagnostic("calibration_promotion", "evidence_only calibration claims production_ready=true"))


def _iter_bundle_files(root: Path) -> Iterable[Path]:
    """Yield bundle entries without following symlinked directories."""
    if not root.is_dir() or root.is_symlink():
        return
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(current), key=lambda item: item.name)
        except OSError:
            continue
        for entry in entries:
            candidate = Path(entry.path)
            try:
                if entry.is_symlink():
                    yield candidate
                elif entry.is_dir(follow_symlinks=False):
                    pending.append(candidate)
                else:
                    yield candidate
            except OSError:
                yield candidate


def validate_manifest(bundle_dir: str | Path, manifest_path: str | Path | None = None) -> dict[str, Any]:
    """Validate manifest schema, relative paths, roles, sizes and hashes."""
    root = Path(bundle_dir)
    path = Path(manifest_path) if manifest_path is not None else root / "manifest.json"
    errors: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    if root.is_symlink():
        errors.append(_diagnostic("bundle_symlink", "bundle directory must not be a symlink"))
    elif root.exists() and not root.is_dir():
        errors.append(_diagnostic("bundle_not_directory", "bundle path is not a directory"))
    if not _resolved_under(path, root):
        errors.append(_diagnostic("manifest_outside_bundle", "manifest path is outside bundle directory"))
    if path.is_symlink():
        errors.append(_diagnostic("manifest_symlink", "manifest must not be a symlink"))
    elif path.exists():
        try:
            manifest_info = path.stat()
            if manifest_info.st_nlink > 1:
                errors.append(_diagnostic("manifest_hardlink", "manifest must not be a hardlink alias"))
            if not stat.S_ISREG(manifest_info.st_mode):
                errors.append(_diagnostic("manifest_not_regular", "manifest must be a regular file"))
        except OSError as exc:
            errors.append(_diagnostic("manifest_stat", f"unable to stat manifest: {exc}"))
    if errors and any(item["code"] in {"bundle_symlink", "bundle_not_directory", "manifest_outside_bundle", "manifest_symlink", "manifest_not_regular", "manifest_hardlink"} for item in errors):
        return {"status": "FAIL", "errors": errors, "warnings": warnings}
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(_diagnostic("manifest_invalid", f"unable to read manifest JSON: {exc}"))
        return {"status": "FAIL", "errors": errors, "warnings": warnings}
    if not isinstance(value, dict):
        errors.append(_diagnostic("manifest_invalid", "manifest root must be an object"))
        return {"status": "FAIL", "errors": errors, "warnings": warnings}
    if value.get("schema_version") != SCHEMA_VERSION:
        errors.append(_diagnostic("manifest_schema", "unsupported manifest schema_version"))
    if value.get("status") not in STATUS_CODES:
        errors.append(_diagnostic("manifest_status", "manifest status must be PASS, WARN, or FAIL"))
    if value.get("mode") not in MODES:
        errors.append(_diagnostic("manifest_mode", "manifest mode must be evidence_only or strict"))
    fixed_safety = _boundary()
    manifest_safety = value.get("safety_boundary")
    if not isinstance(manifest_safety, Mapping):
        errors.append(_diagnostic("manifest_safety_boundary", "manifest safety_boundary must be an object"))
    else:
        for key, expected in fixed_safety.items():
            if manifest_safety.get(key) != expected:
                errors.append(_diagnostic("manifest_safety_boundary", f"manifest safety field {key} is not {expected!r}", field=key))
    manifest_evidence = value.get("evidence_boundary")
    if not isinstance(manifest_evidence, Mapping):
        errors.append(_diagnostic("manifest_evidence_boundary", "manifest evidence_boundary must be an object"))
    else:
        for key in ("real_hit_rate_computed", "hardware_validation", "gimbal_closed_loop_validated", "firing_validated"):
            if manifest_evidence.get(key) is not False:
                errors.append(_diagnostic("manifest_evidence_boundary", f"manifest evidence field {key} is not false", field=key))
    diagnostics = value.get("diagnostics", {})
    if not isinstance(diagnostics, Mapping):
        errors.append(_diagnostic("manifest_diagnostics", "manifest diagnostics must be an object"))
        diagnostics = {}
    declared_errors = diagnostics.get("errors", [])
    declared_warnings = diagnostics.get("warnings", [])
    if not isinstance(declared_errors, list) or not isinstance(declared_warnings, list):
        errors.append(_diagnostic("manifest_diagnostics", "manifest diagnostics errors/warnings must be lists"))
        declared_errors, declared_warnings = [], []
    expected_status = "FAIL" if declared_errors else ("WARN" if declared_warnings else "PASS")
    if value.get("status") in STATUS_CODES and value.get("status") != expected_status:
        errors.append(_diagnostic("manifest_status", f"manifest status {value.get('status')} does not match diagnostics ({expected_status})"))
    artifacts = value.get("artifacts")
    if not isinstance(artifacts, list):
        errors.append(_diagnostic("manifest_artifacts", "manifest artifacts must be a list"))
        return {"status": "FAIL", "errors": errors, "warnings": warnings}
    unhashed_files = value.get("unhashed_files")
    unhashed_paths: set[str] = set()
    if not isinstance(unhashed_files, list) or any(not isinstance(item, str) for item in unhashed_files):
        errors.append(_diagnostic("manifest_unhashed_files", "unhashed_files must be a list of strings"))
    else:
        for item in unhashed_files:
            relative = _normal_relpath(item)
            if relative is None:
                errors.append(_diagnostic("manifest_unhashed_files", f"unhashed file path is invalid: {item!r}"))
                continue
            unhashed_paths.add(relative)
            if relative != "manifest.json":
                errors.append(_diagnostic("manifest_unhashed_files", f"only manifest.json may be unhashed: {relative}"))
        if len(unhashed_paths) != len(unhashed_files):
            errors.append(_diagnostic("manifest_unhashed_files", "unhashed_files must not contain duplicates"))
    declared_roles = value.get("required_roles")
    if "required_roles" not in value:
        errors.append(_diagnostic("manifest_required_roles", "manifest must declare required_roles"))
    if not isinstance(declared_roles, list) or any(not isinstance(item, str) for item in declared_roles):
        errors.append(_diagnostic("manifest_required_roles", "required_roles must be a list of strings"))
        declared_roles = list(REQUIRED_ROLES)
    elif len(set(declared_roles)) != len(declared_roles):
        errors.append(_diagnostic("manifest_required_roles", "required_roles must not contain duplicates"))
    if isinstance(declared_roles, list):
        unknown_required_roles = sorted(set(declared_roles) - set(REQUIRED_ROLES))
        for role in unknown_required_roles:
            errors.append(_diagnostic("manifest_required_roles", f"manifest required_roles contains unknown role: {role}", role=role))
    strict_roles = value.get("strict_roles", list(STRICT_ROLES))
    if "strict_roles" not in value:
        errors.append(_diagnostic("manifest_strict_roles", "manifest must declare strict_roles"))
    if not isinstance(strict_roles, list) or any(not isinstance(item, str) for item in strict_roles):
        errors.append(_diagnostic("manifest_strict_roles", "strict_roles must be a list of strings"))
        strict_roles = list(STRICT_ROLES)
    elif len(set(strict_roles)) != len(strict_roles):
        errors.append(_diagnostic("manifest_strict_roles", "strict_roles must not contain duplicates"))
    roles: set[str] = set()
    paths: set[str] = set()
    ordering: list[tuple[str, str]] = []
    for index, entry in enumerate(artifacts):
        if not isinstance(entry, dict):
            errors.append(_diagnostic("manifest_artifact", "artifact entry must be an object", index=index))
            continue
        role = entry.get("role")
        relative = _normal_relpath(entry.get("path"))
        if not isinstance(role, str) or not role.strip():
            errors.append(_diagnostic("artifact_role", "artifact role must be non-empty", index=index))
            continue
        if role in roles:
            errors.append(_diagnostic("duplicate_artifact_role", f"duplicate artifact role: {role}"))
        roles.add(role)
        if relative is None:
            errors.append(_diagnostic("artifact_path", f"artifact path is absolute or traverses bundle: {entry.get('path')!r}", role=role))
            continue
        if role.startswith("annotated_png:") and not relative.lower().endswith(".png"):
            errors.append(_diagnostic("annotated_artifact_type", f"annotated artifact must have a .png path: {relative}", role=role))
        if relative in paths:
            errors.append(_diagnostic("duplicate_artifact_path", f"duplicate artifact path: {relative}"))
        paths.add(relative)
        ordering.append((role, relative))
        candidate = root / relative
        if candidate.is_symlink():
            errors.append(_diagnostic("artifact_symlink", f"symlink artifacts are not allowed: {relative}", role=role))
            continue
        if not _resolved_under(candidate, root):
            errors.append(_diagnostic("artifact_external_path", f"artifact path escapes bundle: {relative}", role=role))
            continue
        if not candidate.is_file():
            errors.append(_diagnostic("artifact_missing", f"artifact file is missing: {relative}", role=role))
            continue
        try:
            if candidate.stat().st_nlink > 1:
                errors.append(_diagnostic("artifact_hardlink", f"hardlink artifacts are not allowed: {relative}", role=role))
                continue
        except OSError as exc:
            errors.append(_diagnostic("artifact_stat", f"unable to stat {relative}: {exc}", role=role))
            continue
        size = entry.get("size_bytes")
        if not isinstance(size, int) or size < 0:
            errors.append(_diagnostic("artifact_size", f"invalid size_bytes for {relative}", role=role))
        elif size != candidate.stat().st_size:
            errors.append(_diagnostic("artifact_hash_mismatch", f"size mismatch for {relative}", role=role))
        digest = entry.get("sha256")
        if not isinstance(digest, str) or not HASH_RE.fullmatch(digest):
            errors.append(_diagnostic("artifact_hash", f"invalid sha256 for {relative}", role=role))
        elif digest.lower() != _sha256(candidate):
            errors.append(_diagnostic("artifact_hash_mismatch", f"sha256 mismatch for {relative}", role=role))
    if ordering != sorted(ordering):
        errors.append(_diagnostic("manifest_order", "artifacts are not stably sorted by role and path"))
    for required_role in REQUIRED_ROLES:
        if required_role not in declared_roles:
            errors.append(_diagnostic("manifest_required_roles", f"manifest required_roles omits fixed role: {required_role}", role=required_role))
    for required_role in REQUIRED_ROLES:
        if required_role not in roles:
            errors.append(_diagnostic("required_artifact_missing", f"required artifact role is missing: {required_role}", role=required_role))
    for strict_role in STRICT_ROLES:
        if strict_role not in strict_roles:
            errors.append(_diagnostic("manifest_strict_roles", f"manifest strict_roles omits fixed role: {strict_role}", role=strict_role))
    if value.get("mode") == "strict":
        for strict_role in STRICT_ROLES:
            if strict_role not in roles:
                errors.append(_diagnostic("strict_artifact_missing", f"strict artifact role is missing: {strict_role}", role=strict_role))
    declared_files = paths | unhashed_paths
    for candidate in _iter_bundle_files(root):
        try:
            relative = candidate.relative_to(root).as_posix()
        except ValueError:
            errors.append(_diagnostic("unexpected_bundle_file", f"bundle entry is outside root: {candidate}"))
            continue
        if relative not in declared_files:
            errors.append(_diagnostic("unexpected_bundle_file", f"file is not declared by manifest: {relative}", path=relative))
    return {"status": _bundle_status(errors, warnings), "errors": errors, "warnings": warnings, "artifact_count": len(artifacts)}


def _write_json(
    path: Path,
    value: Mapping[str, Any],
    *,
    bundle_root: Optional[Path] = None,
    protected_sources: Iterable[Path] = (),
) -> None:
    """Write generated JSON without following symlink/hardlink aliases."""
    path = Path(path)
    if bundle_root is None:
        bundle_root = path.parent
    payload = json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    with _open_output_file(path, bundle_root, protected_sources) as handle:
        handle.write(payload.encode("utf-8"))


def _write_text(
    path: Path,
    value: str,
    *,
    bundle_root: Optional[Path] = None,
    protected_sources: Iterable[Path] = (),
) -> None:
    """Write generated UTF-8 text through the same safe destination gate."""
    path = Path(path)
    if bundle_root is None:
        bundle_root = path.parent
    with _open_output_file(path, bundle_root, protected_sources) as handle:
        handle.write(str(value).encode("utf-8"))


def _summary_markdown(manifest: Mapping[str, Any]) -> str:
    lines = [
        "# Offline auto-aim evidence bundle",
        "",
        f"**Status:** `{manifest.get('status', 'FAIL')}`  ",
        f"**Mode:** `{manifest.get('mode', 'evidence_only')}`",
        "",
        "This bundle is a reproducible file-integrity and software-structure record. It is not a production, hardware, gimbal, firing, hit-rate, or competition result.",
        "",
        "## Run identity",
        "",
    ]
    metadata = manifest.get("metadata", {})
    if isinstance(metadata, Mapping) and metadata:
        for key in sorted(metadata):
            lines.append(f"- `{key}`: {metadata[key]}")
    else:
        lines.append("- No run metadata supplied.")
    lines.extend(["", "## Artifacts", "", "| Role | Relative path | Bytes | SHA-256 |", "|---|---|---:|---|"])
    # The manifest is an index over source evidence.  ``summary.md`` itself is
    # intentionally omitted from this table so its hash can be recorded in the
    # manifest without a circular content dependency; manifest.json is the
    # other generated index and is explicitly listed in ``unhashed_files``.
    for entry in manifest.get("artifacts", []):
        if entry.get("role") == "summary":
            continue
        lines.append(f"| `{entry.get('role')}` | `{entry.get('path')}` | {entry.get('size_bytes')} | `{entry.get('sha256')}` |")
    lines.extend(["", "## Safety boundary", "", "```yaml"])
    boundary = manifest.get("safety_boundary", _boundary())
    for key in ("production_ready", "hardware_validation", "gimbal_closed_loop_validated", "firing_validated", "real_hit_rate_computed", "serial_enabled", "dry_run", "allow_fire", "fire_command"):
        value = boundary.get(key)
        if isinstance(value, bool):
            value = str(value).lower()
        lines.append(f"{key}: {value}")
    lines.extend(["```", "", "## Diagnostics", ""])
    for label in ("errors", "warnings"):
        lines.append(f"### {label.title()}")
        values = manifest.get("diagnostics", {}).get(label, [])
        if not values:
            lines.append("- None")
        else:
            for item in values:
                lines.append(f"- `{item.get('code', 'diagnostic')}`: {item.get('message', item)}")
    lines.extend(["", "## Artifact presence", ""])
    presence = manifest.get("artifact_presence", {})
    for key in sorted(presence):
        value = presence[key]
        if isinstance(value, bool):
            value = str(value).lower()
        lines.append(f"- `{key}`: {value}")
    lines.extend(["", "## Unconfirmed items", ""])
    for item in manifest.get("unconfirmed_items", UNCONFIRMED_ITEMS):
        lines.append(f"- {item}")
    lines.extend([
        "",
        "`fire_command != 0` and `test_only=false` are safety failures. No command is executed by this tool.",
        "",
        "Low reprojection error does not establish distance accuracy or hit rate. This package does not validate Orin, a real camera, serial, gimbal closed loop, or firing.",
        "",
    ])
    return "\n".join(lines)


def _rejected_bundle_manifest(root: Path, input_csv: str | Path, mode: str, errors: list[dict[str, Any]]) -> dict[str, Any]:
    """Return a complete in-memory FAIL manifest when output is unsafe."""
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "FAIL",
        "mode": mode if mode in MODES else "evidence_only",
        "bundle_id": _safe_name(root),
        "input_csv_name": _safe_name(Path(input_csv)),
        "metadata": {},
        "run_identity": {},
        "csv_report_status": "FAIL",
        "artifacts": [],
        "required_roles": list(REQUIRED_ROLES),
        "strict_roles": list(STRICT_ROLES),
        "unhashed_files": ["manifest.json"],
        "diagnostics": {"errors": _redact_data(errors), "warnings": []},
        "safety_boundary": _boundary(),
        "unconfirmed_items": list(UNCONFIRMED_ITEMS),
        "evidence_boundary": {
            "software_structure_statistics_only": True,
            "real_hit_rate_computed": False,
            "hardware_validation": False,
            "gimbal_closed_loop_validated": False,
            "firing_validated": False,
        },
        "capability_validation": {
            "production_ready": False,
            "hardware_validation": False,
            "gimbal_closed_loop_validated": False,
            "firing_validated": False,
            "real_hit_rate_computed": False,
            "competition_result": False,
        },
        "artifact_presence": {key: False for key in (
            "input_csv", "csv_report_json", "csv_report_markdown", "summary",
            "run_metadata", "camera_intrinsic_report", "model_profile", "pnp_config",
            "annotated_png", "producer_command",
        )},
    }


def build_bundle(
    input_csv: str | Path,
    output_dir: str | Path,
    *,
    metadata_json: str | Path | None = None,
    mode: str = "evidence_only",
    camera_intrinsic_report: str | Path | None = None,
    model_profile: str | Path | None = None,
    pnp_config: str | Path | None = None,
    annotated_dir: str | Path | None = None,
    producer_command_file: str | Path | None = None,
) -> dict[str, Any]:
    """Build a bundle and return the JSON-compatible manifest object."""
    root = Path(output_dir)
    errors: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    try:
        _ensure_empty_output_root(root)
    except OSError as exc:
        # Refuse before any generated write.  In particular, never append a
        # report to an existing directory containing stale producer/annotated
        # files, and never follow an output-directory symlink.
        return _rejected_bundle_manifest(
            root,
            input_csv,
            mode,
            [_diagnostic("unsafe_output_directory", str(exc))],
        )
    if mode not in MODES:
        errors.append(_diagnostic("invalid_mode", f"unsupported mode: {mode}"))
        mode = "evidence_only"
    metadata, metadata_diagnostics = _load_metadata(Path(metadata_json) if metadata_json else None)
    for message in metadata_diagnostics:
        # Dropping unknown/sensitive fields is a non-fatal redaction warning;
        # malformed/unreadable metadata is a real integrity error.
        target = warnings if message.startswith(("ignored ", "metadata JSON was not supplied")) else errors
        target.append(_diagnostic("metadata", message))
    if metadata_json is None and mode == "strict":
        errors.append(_diagnostic("missing_metadata", "strict mode requires metadata JSON"))
    elif metadata_json is None:
        warnings.append(_diagnostic("missing_metadata", "metadata JSON was not supplied"))
    if mode == "strict":
        for key in STRICT_METADATA_KEYS:
            if key not in metadata or metadata.get(key) in (None, ""):
                errors.append(_diagnostic("metadata_required", f"strict metadata field is missing: {key}", field=key))

    source = Path(input_csv)
    protected_sources = [
        Path(item)
        for item in (metadata_json, camera_intrinsic_report, model_profile, pnp_config, producer_command_file)
        if item is not None
    ]
    protected_sources.insert(0, source)
    generated_paths = {
        root / "csv_report.json",
        root / "csv_report.md",
        root / "_rejected_csv_report.json",
        root / "_rejected_csv_report.md",
        root / "manifest.json",
        root / "summary.md",
        root / "run_metadata.json",
        root / "camera_intrinsic_report.yaml",
        root / "model_profile.yaml",
        root / "pnp_config.yaml",
        root / "producer_command.txt",
        root / "input" / "auto_aim.csv",
    }
    source_output_collision = any(_same_path_or_file(source, protected) for protected in generated_paths)
    if source_output_collision:
        errors.append(_diagnostic("input_output_collision", "input CSV aliases a generated bundle output; source was not overwritten"))
    # PR #16 intentionally has a narrower metadata allow-list.  Keep the
    # bundle's run_id/command in its own manifest, while passing only the
    # established fields into the reused CSV report module so harmless bundle
    # metadata does not turn a clean CSV report into WARN.
    csv_metadata = {key: value for key, value in metadata.items() if key in CSV_REPORT_METADATA_KEYS}
    analysis = analyze_csv(source, csv_metadata)
    csv_report = _redact_data(build_report(analysis))
    report_json_path = root / ("_rejected_csv_report.json" if source_output_collision else "csv_report.json")
    report_markdown_path = root / ("_rejected_csv_report.md" if source_output_collision else "csv_report.md")
    try:
        _write_json(report_json_path, csv_report, bundle_root=root, protected_sources=protected_sources)
    except OSError as exc:
        errors.append(_diagnostic("csv_report_json_write", f"unable to write CSV JSON report: {exc}"))
    try:
        _write_text(report_markdown_path, markdown_report(csv_report), bundle_root=root, protected_sources=protected_sources)
    except OSError as exc:
        errors.append(_diagnostic("csv_report_markdown_write", f"unable to write CSV Markdown report: {exc}"))
    _check_boundary(csv_report, errors)
    if csv_report.get("status") == "FAIL":
        errors.append(_diagnostic("csv_report_fail", "CSV evidence report status is FAIL"))
    elif csv_report.get("status") == "WARN":
        (errors if mode == "strict" else warnings).append(_diagnostic("csv_report_warn", "CSV evidence report status is WARN"))

    artifacts: list[dict[str, Any]] = []
    role_paths: set[str] = set()

    def register(role: str, path: Path) -> None:
        if role in {item["role"] for item in artifacts}:
            errors.append(_diagnostic("duplicate_artifact_role", f"duplicate artifact role: {role}"))
            return
        if not _resolved_under(path, root) or not path.is_file():
            errors.append(_diagnostic("artifact_missing", f"artifact is not a readable bundle file: {role}"))
            return
        relative = path.resolve().relative_to(root.resolve()).as_posix()
        if relative in role_paths:
            errors.append(_diagnostic("duplicate_artifact_path", f"duplicate artifact path: {relative}"))
            return
        role_paths.add(relative)
        artifacts.append(_artifact(role, path, root))

    if source.is_file():
        destination = root / "input" / "auto_aim.csv"
        try:
            _copy_file(source, destination, bundle_root=root, protected_sources=protected_sources)
            register("input_csv", destination)
        except OSError as exc:
            errors.append(_diagnostic("input_csv_copy", f"unable to package input CSV: {exc}"))
    else:
        errors.append(_diagnostic("missing_input_csv", f"input CSV is missing or unreadable: {_safe_name(source)}"))

    if not source_output_collision:
        register("csv_report_json", root / "csv_report.json")
        register("csv_report_markdown", root / "csv_report.md")
    if metadata_json and isinstance(metadata, Mapping) and not any(item["code"] == "metadata" for item in errors):
        metadata_destination = root / "run_metadata.json"
        if _same_path_or_file(Path(metadata_json), metadata_destination):
            errors.append(_diagnostic("metadata_output_collision", "metadata JSON aliases generated run_metadata.json"))
        else:
            try:
                _write_json(metadata_destination, metadata, bundle_root=root, protected_sources=protected_sources)
                register("run_metadata", metadata_destination)
            except OSError as exc:
                errors.append(_diagnostic("metadata_copy", f"unable to package run metadata: {exc}"))

    provided = {
        "camera_intrinsic_report": camera_intrinsic_report,
        "model_profile": model_profile,
        "pnp_config": pnp_config,
    }
    destinations = {
        "camera_intrinsic_report": root / "camera_intrinsic_report.yaml",
        "model_profile": root / "model_profile.yaml",
        "pnp_config": root / "pnp_config.yaml",
    }
    for role, value in provided.items():
        if value is None:
            (errors if mode == "strict" else warnings).append(_diagnostic("missing_optional_artifact", f"{role} was not supplied", role=role))
            continue
        source_path = Path(value)
        if not source_path.is_file():
            errors.append(_diagnostic("missing_artifact", f"{role} is missing or unreadable: {_safe_name(source_path)}", role=role))
            continue
        if any(_same_path_or_file(source_path, generated) for generated in generated_paths):
            errors.append(_diagnostic("artifact_output_collision", f"{role} aliases a generated bundle output", role=role))
            continue
        try:
            _copy_file(source_path, destinations[role], bundle_root=root, protected_sources=protected_sources)
            register(role, destinations[role])
            if role == "camera_intrinsic_report":
                _check_calibration_promotion(source_path, errors)
        except (OSError, ValueError) as exc:
            errors.append(_diagnostic("artifact_copy", f"unable to package {role}: {exc}", role=role))

    if annotated_dir is not None:
        source_path = Path(annotated_dir)
        try:
            annotated_destination = root / "annotated"
            if (
                _same_path_or_file(source_path, annotated_destination)
                or _resolved_under(annotated_destination, source_path)
                or _resolved_under(source_path, annotated_destination)
            ):
                raise OSError("annotated source aliases or contains its bundle destination")
            copied = _copy_annotated(source_path, root / "annotated", bundle_root=root, protected_sources=protected_sources)
            if not copied:
                warnings.append(_diagnostic("annotated_empty", "annotated directory contained no PNG files"))
            for path in copied:
                relative = path.relative_to(root).as_posix()
                register(f"annotated_png:{relative}", path)
        except OSError as exc:
            errors.append(_diagnostic("annotated_copy", str(exc)))

    if producer_command_file is not None:
        source_path = Path(producer_command_file)
        if not source_path.is_file():
            errors.append(_diagnostic("missing_artifact", f"producer command file is missing: {_safe_name(source_path)}", role="producer_command"))
        else:
            try:
                producer_destination = root / "producer_command.txt"
                if _same_path_or_file(source_path, producer_destination):
                    raise OSError("producer command source aliases its bundle destination")
                _copy_text_redacted(source_path, producer_destination, bundle_root=root, protected_sources=protected_sources)
                register("producer_command", producer_destination)
            except (OSError, UnicodeError) as exc:
                errors.append(_diagnostic("producer_command_copy", str(exc)))

    artifacts.sort(key=lambda item: (item["role"], item["path"]))
    errors = _redact_data(errors)
    warnings = _redact_data(warnings)
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "status": _bundle_status(errors, warnings),
        "mode": mode,
        "bundle_id": metadata.get("run_id") or _safe_name(root),
        "input_csv_name": _safe_name(source),
        "metadata": metadata,
        "run_identity": {
            key: metadata.get(key)
            for key in ("run_id", "commit", "dataset_id", "model_profile_id", "model_profile_version", "pnp_profile", "run_command")
            if metadata.get(key) is not None
        },
        "csv_report_status": csv_report.get("status"),
        "artifacts": artifacts,
        "required_roles": list(REQUIRED_ROLES),
        "strict_roles": list(STRICT_ROLES),
        "unhashed_files": ["manifest.json"],
        "diagnostics": {"errors": errors, "warnings": warnings},
        "safety_boundary": _boundary(),
        "unconfirmed_items": list(UNCONFIRMED_ITEMS),
        "evidence_boundary": {
            "software_structure_statistics_only": True,
            "real_hit_rate_computed": False,
            "hardware_validation": False,
            "gimbal_closed_loop_validated": False,
            "firing_validated": False,
        },
        "capability_validation": {
            "production_ready": False,
            "hardware_validation": False,
            "gimbal_closed_loop_validated": False,
            "firing_validated": False,
            "real_hit_rate_computed": False,
            "competition_result": False,
        },
        "artifact_presence": {},
    }
    artifact_roles = {item["role"] for item in artifacts}
    manifest["artifact_presence"] = {
        "input_csv": "input_csv" in artifact_roles,
        "csv_report_json": "csv_report_json" in artifact_roles,
        "csv_report_markdown": "csv_report_markdown" in artifact_roles,
        "summary": "summary" in artifact_roles,
        "run_metadata": "run_metadata" in artifact_roles,
        "camera_intrinsic_report": "camera_intrinsic_report" in artifact_roles,
        "model_profile": "model_profile" in artifact_roles,
        "pnp_config": "pnp_config" in artifact_roles,
        "annotated_png": any(role.startswith("annotated_png:") for role in artifact_roles),
        "producer_command": "producer_command" in artifact_roles,
    }
    try:
        _write_json(root / "manifest.json", manifest, bundle_root=root, protected_sources=protected_sources)
    except OSError as exc:
        errors.append(_diagnostic("manifest_write", f"unable to write manifest: {exc}"))
    verification = validate_manifest(root)
    errors.extend(verification.get("errors", []))
    warnings.extend(verification.get("warnings", []))
    manifest["diagnostics"] = {"errors": errors, "warnings": warnings}
    manifest["status"] = _bundle_status(errors, warnings)
    summary_path = root / "summary.md"
    try:
        _write_text(summary_path, _summary_markdown(manifest), bundle_root=root, protected_sources=protected_sources)
        register("summary", summary_path)
    except OSError as exc:
        errors.append(_diagnostic("summary_write", f"unable to write summary: {exc}"))
    artifacts.sort(key=lambda item: (item["role"], item["path"]))
    artifact_roles = {item["role"] for item in artifacts}
    manifest["artifact_presence"]["summary"] = "summary" in artifact_roles
    manifest["artifacts"] = artifacts
    try:
        _write_json(root / "manifest.json", manifest, bundle_root=root, protected_sources=protected_sources)
    except OSError as exc:
        errors.append(_diagnostic("manifest_write", f"unable to write manifest: {exc}"))
    verification = validate_manifest(root)
    if verification.get("errors"):
        errors.extend(verification["errors"])
        manifest["diagnostics"] = {"errors": errors, "warnings": warnings}
        manifest["status"] = _bundle_status(errors, warnings)
        try:
            _write_text(summary_path, _summary_markdown(manifest), bundle_root=root, protected_sources=protected_sources)
            for entry in manifest["artifacts"]:
                if entry.get("role") == "summary":
                    entry.update(_artifact("summary", summary_path, root))
                    break
        except OSError as exc:
            errors.append(_diagnostic("summary_write", f"unable to refresh summary: {exc}"))
        manifest["artifacts"].sort(key=lambda item: (item["role"], item["path"]))
        try:
            _write_json(root / "manifest.json", manifest, bundle_root=root, protected_sources=protected_sources)
        except OSError as exc:
            errors.append(_diagnostic("manifest_write", f"unable to write manifest: {exc}"))
    return manifest


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build or verify a read-only offline auto-aim evidence bundle")
    parser.add_argument("--input-csv", help="auto_aim_offline CSV input")
    parser.add_argument("--output-dir", required=True, help="bundle output directory")
    parser.add_argument("--metadata-json", help="run metadata JSON")
    parser.add_argument("--mode", choices=sorted(MODES), default="evidence_only")
    parser.add_argument("--camera-intrinsic-report", help="camera evidence-only YAML")
    parser.add_argument("--model-profile", help="model profile YAML")
    parser.add_argument("--pnp-config", help="PnP config YAML")
    parser.add_argument("--annotated-dir", help="annotated PNG directory")
    parser.add_argument("--producer-command-file", help="text file containing the producer command")
    parser.add_argument("--verify-manifest", help="validate an existing manifest instead of building")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    root = Path(args.output_dir)
    if args.verify_manifest:
        result = validate_manifest(root, args.verify_manifest)
        print(f"status={result['status']}")
        return {"PASS": 0, "WARN": 2, "FAIL": 1}[result["status"]]
    if not args.input_csv:
        _parser().error("--input-csv is required unless --verify-manifest is used")
    try:
        manifest = build_bundle(
            args.input_csv,
            root,
            metadata_json=args.metadata_json,
            mode=args.mode,
            camera_intrinsic_report=args.camera_intrinsic_report,
            model_profile=args.model_profile,
            pnp_config=args.pnp_config,
            annotated_dir=args.annotated_dir,
            producer_command_file=args.producer_command_file,
        )
        print(f"status={manifest['status']}")
        return {"PASS": 0, "WARN": 2, "FAIL": 1}[manifest["status"]]
    except Exception as exc:  # keep an output diagnostic for unexpected input failures
        fallback = {
            "schema_version": SCHEMA_VERSION,
            "status": "FAIL",
            "mode": args.mode,
            "metadata": {},
            "artifacts": [],
            "diagnostics": {"errors": [_diagnostic("bundle_exception", _redact_text(str(exc)))], "warnings": []},
            "safety_boundary": _boundary(),
        }
        try:
            # An exception must not turn the fallback into a second write pass
            # over a partially written or reused output directory.  Only a
            # newly-created/empty real directory is eligible for fallback.
            _ensure_empty_output_root(root)
            _write_json(root / "manifest.json", fallback, bundle_root=root)
            _write_json(root / "csv_report.json", {"status": "FAIL", "report_status": "FAIL", "errors": fallback["diagnostics"]["errors"], "evidence_boundary": fallback["safety_boundary"]}, bundle_root=root)
            _write_text(root / "csv_report.md", "# Offline auto-aim evidence report\n\n**Report status: `FAIL`**\n", bundle_root=root)
            _write_text(root / "summary.md", _summary_markdown(fallback), bundle_root=root)
        except OSError:
            pass
        print("status=FAIL")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
