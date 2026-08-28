#!/usr/bin/env python3
"""Software-freeze candidate admission gate.

The gate is deliberately a small, standard-library-only, read-only tool.  It
does not change source files, Git history, remote PRs, hardware, or system
configuration.  Command execution is kept behind an injectable runner so the
unit tests can exercise all admission decisions without GitHub, ROS, models,
or hardware.
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import hashlib
import json
import os
import re
import shlex
import stat
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Sequence


SCHEMA_VERSION = 1
REPORT_SCHEMA = "software-freeze-candidate"
ALLOWED_STATUSES = (
    "PASS",
    "FAIL",
    "BLOCKED",
    "UNAVAILABLE",
    "NOT_RUN",
    "NOT_VERIFIED",
)
SAFE_DEFAULTS: dict[str, Any] = {
    "serial_enabled": False,
    "dry_run": True,
    "allow_fire": False,
    "fire_command": 0,
    "yaw_vel": 0,
    "pitch_vel": 0,
    "yaw_acc": 0,
    "pitch_acc": 0,
}

REQUIRED_CTEST_NAMES = (
    "auto_aim_core_test",
    "offline_pipeline_test",
    "offline_tracker_replay_test",
    "offline_predictor_test",
    "offline_ballistic_test",
    "offline_scenario_benchmark_test",
    "pnp_stage_test",
    "raw_armor_detector_test",
    "ros_image_adapter_test",
    "ros_backend_test",
    "vision_time_alignment_test",
    "ros_safety_integration_test",
    "camera_calibration_test",
    "auto_aim_offline_csv_test",
    "auto_aim_offline_cli_test",
    "auto_aim_scenario_benchmark_cli_test",
    "serial_protocol_loopback_test",
    "robot_ctrl_safety_test",
    "preflight_analyzer_test",
    "calibration_dataset_test",
    "calibration_dataset_ros_test",
    "fake_ros_publishers_test",
    "process_contract_test",
    # The ROS message-level E2E package is part of the frozen tree. Its
    # report_test is the CTest-level regression entry; the standalone runner
    # remains a separate, explicit matrix exercised by its wrapper.
    "report_test",
)
REQUIRED_PYTHON_GROUPS = (
    "qualification",
    "offline_evidence_report",
    "offline_evidence_bundle",
    "orin_unavailable",
)
REQUIRED_SCENARIO_FILES = ("benchmark.csv", "summary.json", "summary.md")
SCENARIO_HELP_NAME = "auto_aim_scenario_benchmark_help"
OFFLINE_HELP_NAME = "auto_aim_offline_help"
REQUIRED_CLI_HELP_NAMES = (
    SCENARIO_HELP_NAME,
    OFFLINE_HELP_NAME,
    "auto_aim_dry_run_help",
    "auto_aim_detector_smoke_help",
    "auto_aim_pnp_smoke_help",
    "auto_aim_camera_calibrate_help",
    "auto_aim_calibration_dataset_help",
    "auto_aim_calibration_dataset_recorder_help",
    "ros_input_preflight_help",
    "release_manifest_audit_help",
    "auto_aim_release_smoke_help",
    "release_smoke_help",
    "ros_message_e2e_help",
    "model_qualification_help",
    "offline_evidence_report_help",
    "offline_evidence_bundle_help",
    "orin_environment_preflight_help",
)
SCENARIO_ORIGIN_ASSUMPTION = "synthetic_muzzle_frame"
# Keep this list in one place so the command ledger and the documentation
# cannot silently drift from the freeze procedure.  ``orin_hardware_evidence``
# is intentionally omitted from colcon selection: it is a standalone CMake
# project and is built/tested explicitly below without opening devices.
FREEZE_COLCON_PACKAGES = (
    "auto_aim_interfaces",
    "serical_device_ros2",
    "auto_aim_ros2",
    "auto_aim_tools",
    "auto_aim_ros_e2e",
    "auto_aim_release_smoke",
    "release_manifest_audit",
    "hik_camera",
)


@dataclasses.dataclass(frozen=True)
class CommandResult:
    """A normalized command result used by the runner and fake tests."""

    returncode: int
    stdout: str = ""
    stderr: str = ""
    duration_seconds: float | None = None

    @property
    def ok(self) -> bool:
        return self.returncode == 0


Runner = Callable[..., CommandResult]


def _default_runner(
    argv: Sequence[str],
    *,
    cwd: Path | None = None,
    timeout: float = 600.0,
    env: Mapping[str, str] | None = None,
) -> CommandResult:
    """Run one argv without a shell and capture its output."""

    started = time.monotonic()
    try:
        completed = subprocess.run(
            [str(item) for item in argv],
            cwd=str(cwd) if cwd is not None else None,
            env=dict(env) if env is not None else None,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=timeout,
        )
        return CommandResult(
            completed.returncode,
            completed.stdout,
            completed.stderr,
            time.monotonic() - started,
        )
    except FileNotFoundError as exc:
        return CommandResult(127, "", str(exc), time.monotonic() - started)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="replace")
        return CommandResult(124, stdout, f"timeout: {stderr}", time.monotonic() - started)
    except OSError as exc:
        return CommandResult(126, "", str(exc), time.monotonic() - started)


def _call_runner(runner: Runner, argv: Sequence[str], *, cwd: Path | None = None, timeout: float = 600.0) -> CommandResult:
    """Accept both the documented keyword runner and convenient test fakes."""

    try:
        value = runner(argv, cwd=cwd, timeout=timeout)
    except TypeError:
        try:
            value = runner(argv, cwd)
        except TypeError:
            value = runner(argv)
    if isinstance(value, CommandResult):
        return value
    if isinstance(value, subprocess.CompletedProcess):
        return CommandResult(value.returncode, value.stdout or "", value.stderr or "")
    if isinstance(value, Mapping):
        return CommandResult(
            int(value.get("returncode", value.get("exit_code", 1))),
            str(value.get("stdout", "")),
            str(value.get("stderr", "")),
            value.get("duration_seconds"),
        )
    if isinstance(value, tuple) and len(value) >= 1:
        return CommandResult(int(value[0]), str(value[1] if len(value) > 1 else ""), str(value[2] if len(value) > 2 else ""))
    raise TypeError(f"runner returned unsupported value: {type(value).__name__}")


def _display_command(argv: Sequence[str]) -> str:
    """Render a command without leaking local absolute paths."""

    rendered: list[str] = []
    for item in argv:
        value = str(item)
        if value.startswith("/") or re.match(r"^[A-Za-z]:[\\/]", value) or value.startswith("\\\\"):
            value = "<path>/" + Path(value.replace("\\", "/")).name
        rendered.append(value)
    # Bash wrapper commands contain paths inside one string (for example the
    # isolated /tmp colcon directories), so redact after joining as well.
    return _redact_text(shlex.join(rendered))


def _redact_text(value: str, *, repo_root: Path | None = None) -> str:
    text = str(value or "")
    # Terminal progress/control sequences are presentation noise and can
    # differ between PTY/non-PTY invocations; never persist them in evidence.
    text = re.sub(r"\x1b\[[0-?]*[ -/]*[@-~]", "", text)
    if repo_root is not None:
        for raw in (str(repo_root), str(repo_root).replace("\\", "/")):
            if raw:
                text = text.replace(raw, "<worktree>")
    text = re.sub(r"(?i)(token|password|passwd|secret|api[_ -]?key|access[_ -]?token)\s*[=:]\s*[^\s,;]+", r"\1=<redacted>", text)
    text = re.sub(r"(?:[A-Za-z]:[\\/]|\\\\)[^\s\"']+", "<path>", text)
    text = re.sub(r"(?<![A-Za-z0-9])/(?:home|tmp|workspace|mnt|opt)/[^\s\"']+", "<path>", text)
    return text[-4000:]


def _stable_command_output(
    value: str,
    *,
    repo_root: Path | None = None,
    sort_lines: bool = False,
) -> str:
    """Redact command output and remove run-to-run presentation variance."""

    text = _redact_text(value, repo_root=repo_root)
    text = re.sub(r"\[\s*\d+(?:min\s+\d+)?(?:\.\d+)?s\s*\]", "[<duration>]", text, flags=re.IGNORECASE)
    text = re.sub(r"(Passed\s+)\d+(?:\.\d+)?\s+sec\b", r"\1<duration>", text, flags=re.IGNORECASE)
    text = re.sub(r"(Failed\s+)\d+(?:\.\d+)?\s+sec\b", r"\1<duration>", text, flags=re.IGNORECASE)
    text = re.sub(r"(in\s+)\d+(?:\.\d+)?s\b", r"\1<duration>", text, flags=re.IGNORECASE)
    text = re.sub(r"(=\s*)\d+(?:\.\d+)?\s+sec\b", r"\1<duration>", text, flags=re.IGNORECASE)
    # colcon/ctest progress commonly prints millisecond durations in
    # parentheses, which otherwise vary between identical runs.
    text = re.sub(r"\(\s*\d+(?:\.\d+)?\s*(?:ms|s)\s*\)", "(<duration>)", text, flags=re.IGNORECASE)
    text = re.sub(r"\b\d+(?:\.\d+)?\s*ms\b", "<duration>", text, flags=re.IGNORECASE)
    text = re.sub(r"(Create new tag:\s*)\d{8}-\d{4}\b", r"\1<timestamp>", text, flags=re.IGNORECASE)
    text = re.sub(r"(?m)^\s*Site:\s*.*$", "   Site: <redacted>", text)
    if sort_lines:
        trailing_newline = text.endswith("\n")
        lines = text.splitlines()
        text = "\n".join(sorted(lines))
        if trailing_newline:
            text += "\n"
    return text[-4000:]


def _status(value: str) -> str:
    return value if value in ALLOWED_STATUSES else "FAIL"


def _normalise_check_status(value: Any, default: str = "NOT_RUN") -> str:
    text = str(value) if value is not None else default
    return text if text in ALLOWED_STATUSES else "FAIL"


def _finding(status: str, message: str, *, code: str | None = None, details: Any = None) -> dict[str, Any]:
    result: dict[str, Any] = {"status": _status(status), "message": message}
    if code is not None:
        result["code"] = code
    if details is not None:
        result["details"] = details
    return result


def _sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _safe_path_component(path: Path) -> bool:
    """Return whether every existing component is a real directory/file."""

    try:
        current = Path(path.anchor) if path.anchor else Path()
        for part in path.parts[1:] if path.anchor else path.parts:
            current = current / part
            if current.exists() or current.is_symlink():
                info = current.lstat()
                if stat.S_ISLNK(info.st_mode):
                    return False
        return True
    except OSError:
        return False


def _validate_output_dir(output_dir: Path) -> tuple[bool, str]:
    """Require a new, non-link directory and safe existing parents."""

    output_dir = Path(output_dir)
    if not _safe_path_component(output_dir.parent):
        return False, "output parent contains a symlink or cannot be inspected"
    try:
        if output_dir.exists() or output_dir.is_symlink():
            if output_dir.is_symlink():
                return False, "output directory is a symlink"
            if not output_dir.is_dir():
                return False, "output path is not a directory"
            if any(output_dir.iterdir()):
                return False, "output directory already exists and is not empty"
            return False, "output directory already exists"
    except OSError as exc:
        return False, f"cannot inspect output directory: {exc}"
    return True, ""


def _safe_write(path: Path, payload: bytes) -> None:
    """Create a new regular file without following a final symlink."""

    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    fd = os.open(path, flags, 0o644)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
    except Exception:
        try:
            os.close(fd)
        except OSError:
            pass
        raise


def _json_bytes(value: Mapping[str, Any]) -> bytes:
    return (json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")


def _parse_json(text: str) -> Any:
    try:
        return json.loads(text)
    except (TypeError, json.JSONDecodeError):
        return None


def _parse_sha_from_ls_remote(text: str) -> str | None:
    for line in str(text).splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[-1] == "refs/heads/main" and re.fullmatch(r"[0-9a-fA-F]{40}", fields[0]):
            return fields[0].lower()
    return None


def _parse_status_lines(text: str) -> list[str]:
    return [line for line in str(text).splitlines() if line.strip()]


def _command_record(name: str, argv: Sequence[str], result: CommandResult, *, cwd: Path | None = None) -> dict[str, Any]:
    command = _display_command(argv)
    stable_output = " ".join((command, result.stdout, result.stderr)).lower()
    sort_output = "colcon" in stable_output or "ctest" in stable_output
    return {
        "name": name,
        "command": command,
        "exit_code": result.returncode,
        "status": "PASS" if result.ok else "FAIL",
        "stdout": _stable_command_output(result.stdout, repo_root=cwd, sort_lines=sort_output),
        "stderr": _stable_command_output(result.stderr, repo_root=cwd, sort_lines=sort_output),
    }


def _extract_test_counts(text: str) -> dict[str, int | None]:
    """Extract common ctest/colcon totals while retaining unknowns."""

    combined = str(text)
    patterns: dict[str, tuple[str, ...]] = {
        "total": (r"(\d+)\s+tests?\s+passed", r"(\d+)\s+tests?", r"Test\s+#\d+"),
        "failed": (r"fail(?:ed|ures?)\s*[:=]\s*(\d+)", r"(\d+)\s+fail(?:ed|ures?)"),
        "errors": (r"errors?\s*[:=]\s*(\d+)", r"(\d+)\s+errors?"),
        "skipped": (r"skipped\s*[:=]\s*(\d+)", r"(\d+)\s+skipped"),
    }
    result: dict[str, int | None] = {key: None for key in patterns}
    for key, alternatives in patterns.items():
        for pattern in alternatives:
            match = re.search(pattern, combined, re.IGNORECASE)
            if match and match.lastindex:
                try:
                    result[key] = int(match.group(1))
                except ValueError:
                    pass
                break
    # ctest's summary is authoritative when present.
    summary = re.search(r"(\d+)% tests passed,?\s+(\d+) tests? failed out of (\d+)", combined, re.IGNORECASE)
    if summary:
        result["failed"] = int(summary.group(2))
        result["total"] = int(summary.group(3))
    return result


def _parse_ctest_xml_results(build_base: Path) -> list[dict[str, Any]]:
    """Read CTest XML result files without depending on a third-party parser."""

    import xml.etree.ElementTree as ET

    results: list[dict[str, Any]] = []
    for path in sorted(build_base.glob("*/test_results/*/*.xml")):
        try:
            root = ET.parse(path).getroot()
        except (OSError, ET.ParseError):
            continue
        name = root.attrib.get("name") or path.stem
        try:
            tests = int(root.attrib.get("tests", "0") or 0)
            failures = int(root.attrib.get("failures", "0") or 0)
            errors = int(root.attrib.get("errors", "0") or 0)
            skipped = int(root.attrib.get("disabled", "0") or 0)
        except (TypeError, ValueError):
            # A truncated or hand-edited XML result must never make the gate
            # crash or look like a passing test.  The missing result is later
            # represented as NOT_RUN by the required-test admission check.
            continue
        if tests < 1:
            # A result file declaring zero testcases cannot cover a required
            # executable, even when its failure/error counters are also zero.
            continue
        status = "PASS" if failures == 0 and errors == 0 and skipped == 0 else "FAIL"
        results.append({
            "name": path.stem.removesuffix(".gtest"),
            "status": status,
            "total": tests,
            "failed": failures,
            "errors": errors,
            "skipped": skipped,
            "xml": _redact_text(str(path)),
        })
    return results


def _discover_ctest_names(build_base: Path) -> set[str]:
    """Discover actual CTest names from generated CTestTestfile.cmake files."""

    names: set[str] = set()
    for path in sorted(build_base.glob("*/CTestTestfile.cmake")):
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        names.update(re.findall(r"(?m)^add_test\(([^\s\)]+)", text))
    return names


def _test_result_from_text(name: str, text: str, exit_code: int) -> dict[str, Any]:
    counts = _extract_test_counts(text)
    lower = text.lower()
    failed = counts["failed"]
    errors = counts["errors"]
    skipped = counts["skipped"]
    if exit_code != 0 or (failed or 0) > 0 or (errors or 0) > 0:
        status = "FAIL"
    elif skipped is not None and skipped > 0:
        # Skips are recorded, but they do not pass a complete software gate.
        status = "FAIL"
    elif (counts.get("total") == 0) or re.search(r"\bRan\s+0\s+tests?\b", text, re.IGNORECASE):
        # A successful command that executed no tests is evidence of a
        # missing/empty test suite, not a passing check.
        status = "NOT_RUN"
    elif "not run" in lower or "no tests" in lower:
        status = "NOT_RUN"
    else:
        status = "PASS"
    return {"name": name, "status": status, "exit_code": exit_code, **counts}


def _default_observation() -> dict[str, Any]:
    return {
        "git": {},
        "github": {"status": "NOT_RUN", "open_prs": [], "prs": {}},
        "commands": [],
        # These release-boundary checks are always present in a live report,
        # even when an earlier build failure prevents their runners from
        # starting.  Keeping an explicit NOT_RUN record avoids silently
        # dropping a required input from the final freeze evidence.
        "rosdep": {"status": "NOT_RUN", "reason": "rosdep check not run"},
        "build": {"status": "NOT_RUN"},
        "cpp_tests": {"status": "NOT_RUN", "tests": [], "total": None, "failed": None, "errors": None, "skipped": None},
        "python_tests": {name: {"status": "NOT_RUN"} for name in REQUIRED_PYTHON_GROUPS},
        "scenario": {"status": "NOT_RUN", "runs": []},
        "evidence": {"status": "NOT_RUN", "production_claim_rejected": None},
        "release_smoke": {"status": "NOT_RUN", "report": None},
        "ros_e2e": {"status": "NOT_RUN", "report": None},
        "ros_safety": {"status": "NOT_RUN", "rounds": []},
        "cli_help": {name: {"status": "NOT_RUN"} for name in REQUIRED_CLI_HELP_NAMES},
        "runtime": {
            "openvino_python": {"status": "NOT_RUN", "reason": "runtime probe not run"},
        },
        "camera_preflight": {"status": "NOT_VERIFIED", "reason": "real camera/SDK not exercised"},
        "safety_defaults": {"status": "PASS", "values": dict(SAFE_DEFAULTS), "violations": []},
        "hardware": {
            "model": {"status": "NOT_VERIFIED", "reason": "no production model artifact is checked in"},
            "calibration": {"status": "NOT_VERIFIED", "reason": "production K/D and extrinsic evidence are external"},
            "camera": {"status": "NOT_VERIFIED", "reason": "real camera/SDK not exercised"},
            "orin": {"status": "NOT_VERIFIED", "reason": "Orin hardware not exercised"},
            "cdc": {"status": "NOT_VERIFIED", "reason": "CDC link and golden frames not exercised"},
            "gimbal": {"status": "NOT_VERIFIED", "reason": "closed-loop gimbal not exercised"},
            "firing": {"status": "NOT_VERIFIED", "reason": "firing path not exercised"},
        },
    }


def _normalise_pr(value: Mapping[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key in ("number", "title", "state", "isDraft", "reviewDecision", "headRefName", "baseRefName", "mergedAt", "url"):
        if key in value:
            result[key] = value[key]
    if "number" in result:
        try:
            result["number"] = int(result["number"])
        except (ValueError, TypeError):
            pass
    return result


def _coerce_observations(observations: Mapping[str, Any] | None) -> dict[str, Any]:
    base = _default_observation()
    if observations:
        # A deep-ish merge preserves the stable report shape without requiring
        # tests to spell every unrelated field.
        for key, value in observations.items():
            if isinstance(value, Mapping) and isinstance(base.get(key), Mapping):
                merged = dict(base[key])  # type: ignore[index]
                merged.update(value)
                base[key] = merged
            else:
                base[key] = value
    return base


def _check_safety_values(values: Mapping[str, Any]) -> tuple[str, list[str], dict[str, Any]]:
    violations: list[str] = []
    normalized = dict(values)
    for key, expected in SAFE_DEFAULTS.items():
        actual = normalized.get(key)
        type_ok = True
        if isinstance(expected, bool):
            type_ok = type(actual) is bool
        elif isinstance(expected, int):
            type_ok = isinstance(actual, (int, float)) and not isinstance(actual, bool)
        if not type_ok or actual != expected:
            violations.append(f"{key} expected {expected!r}, got {actual!r}")
    return ("FAIL" if violations else "PASS"), violations, {key: normalized.get(key, expected) for key, expected in SAFE_DEFAULTS.items()}


def _scenario_consistent(scenario: Mapping[str, Any]) -> tuple[str, list[str]]:
    failures: list[str] = []
    runs = scenario.get("runs")
    if not isinstance(runs, list) or len(runs) != 2:
        return "NOT_RUN", ["scenario benchmark must have exactly two runs"]
    first, second = runs
    if not isinstance(first, Mapping) or not isinstance(second, Mapping):
        return "FAIL", ["scenario run record is not an object"]
    first_files = first.get("files", {})
    second_files = second.get("files", {})
    if not isinstance(first_files, Mapping) or not isinstance(second_files, Mapping):
        return "FAIL", ["scenario file records are missing"]
    for run_index, run in enumerate((first, second), 1):
        if run.get("status") != "PASS":
            failures.append(f"scenario run {run_index} did not PASS")
        csv_invariants = run.get("csv_invariants")
        if not isinstance(csv_invariants, Mapping):
            failures.append(f"scenario CSV safety invariants missing in run {run_index}")
        elif csv_invariants.get("status") != "PASS":
            failures.append(f"scenario CSV safety invariants did not PASS in run {run_index}")
    for filename in REQUIRED_SCENARIO_FILES:
        left = first_files.get(filename)
        right = second_files.get(filename)
        if not isinstance(left, Mapping) or not isinstance(right, Mapping):
            failures.append(f"missing scenario file metadata: {filename}")
            continue
        if left.get("status") != "PASS" or right.get("status") != "PASS":
            failures.append(f"scenario file unavailable: {filename}")
            continue
        for side, record in (("first", left), ("second", right)):
            digest = record.get("sha256")
            if not isinstance(digest, str) or re.fullmatch(r"[0-9a-fA-F]{64}", digest) is None:
                failures.append(f"scenario {side} file has invalid SHA-256: {filename}")
            size = record.get("size_bytes")
            if not isinstance(size, int) or isinstance(size, bool) or size < 0:
                failures.append(f"scenario {side} file has invalid byte size: {filename}")
            if record.get("bytes_equal") is not True:
                failures.append(f"scenario bytes were not proven identical: {filename}")
        if left.get("sha256") != right.get("sha256"):
            failures.append(f"scenario SHA-256 differs: {filename}")
        if left.get("size_bytes") != right.get("size_bytes"):
            failures.append(f"scenario byte size differs: {filename}")
        if left.get("byte_identical") is False or right.get("byte_identical") is False:
            failures.append(f"scenario bytes differ: {filename}")
    for run in (first, second):
        safety = run.get("safety", run.get("safety_boundary", {}))
        if isinstance(safety, Mapping):
            aliases = {
                "yaw_vel": "yaw_vel_rad_s",
                "pitch_vel": "pitch_vel_rad_s",
                "yaw_acc": "yaw_acc_rad_s2",
                "pitch_acc": "pitch_acc_rad_s2",
            }
            normalized_safety = dict(safety)
            for key, alias in aliases.items():
                if key not in normalized_safety and alias in normalized_safety:
                    normalized_safety[key] = normalized_safety[alias]
            missing_safety = [key for key in SAFE_DEFAULTS if key not in normalized_safety]
            if missing_safety:
                failures.append("scenario safety fields missing: " + ", ".join(missing_safety))
            status, violations, _ = _check_safety_values(normalized_safety)
            if status != "PASS":
                failures.extend([f"scenario safety: {item}" for item in violations])
            motion_flag = normalized_safety.get("motion_nonzero", False)
            if not isinstance(motion_flag, bool):
                failures.append("scenario safety: motion_nonzero is not a boolean")
            elif motion_flag:
                failures.append("scenario safety: motion_nonzero is true")
        else:
            failures.append("scenario safety fields missing in run")
        boundary = run.get("evidence_boundary", {})
        boundary_production = boundary.get("production_ready") is True if isinstance(boundary, Mapping) else False
        if run.get("production_ready") is True or boundary_production:
            failures.append("scenario contains production_ready=true")
        if run.get("synthetic") is not True or run.get("test_only") is not True:
            failures.append("scenario run is not explicitly synthetic/test_only")
        if run.get("production_ready") is not False:
            failures.append("scenario run is not explicitly production_ready=false")
        if run.get("software_only_synthetic_benchmark") is not True:
            failures.append("scenario run is not explicitly software-only synthetic")
        if run.get("origin_assumption") != SCENARIO_ORIGIN_ASSUMPTION:
            failures.append("scenario origin_assumption is not synthetic_muzzle_frame")
        claim_scan = scan_evidence_claims(run)
        if claim_scan.get("status") != "PASS":
            unsafe_claims = claim_scan.get("unsafe_claims", [])
            if isinstance(unsafe_claims, list) and unsafe_claims:
                failures.extend(
                    f"scenario contains unsafe production/control claim: {item.get('path', '<unknown>')}"
                    for item in unsafe_claims
                    if isinstance(item, Mapping)
                )
            else:
                failures.append("scenario contains unsafe production/control claim")
    return ("FAIL" if failures else "PASS"), failures


def _evidence_status(evidence: Mapping[str, Any]) -> tuple[str, list[str]]:
    if evidence.get("status") in (None, "NOT_RUN") and evidence.get("production_claim_rejected") is None and not any(
        key in evidence for key in (
            "report_status", "bundle_status", "report", "bundle", "files", "claims",
            "production_ready_true", "fire_command_nonzero", "motion_nonzero",
        )
    ):
        return "NOT_RUN", []
    failures: list[str] = []
    evidence_status = evidence.get("status")
    if evidence_status not in ALLOWED_STATUSES:
        failures.append("evidence status is missing or invalid")
    elif evidence_status != "PASS":
        failures.append(f"evidence status={evidence_status}")
    if evidence.get("production_claim_rejected") is not True:
        failures.append("production claim rejection was not demonstrated")
    report_status = evidence.get("report_status")
    if report_status is not None:
        if report_status not in ALLOWED_STATUSES or report_status != "PASS":
            failures.append(f"report_status={report_status}")
    bundle_status = evidence.get("bundle_status")
    def _has_valid_claim_rejection() -> bool:
        """Return whether the checked fixture proves a calibration rejection.

        A caller-provided boolean or free-form diagnostic is not sufficient to
        exempt an unsafe ``production_ready`` claim.  The bundle must have
        failed, its persisted report must be FAIL, and its structured
        diagnostics must contain the documented calibration-promotion code.
        """

        if evidence.get("production_claim_rejected") is not True:
            return False
        if evidence.get("bundle_status") != "FAIL" or evidence.get("bundle_report_status") != "FAIL":
            return False
        diagnostics = evidence.get("bundle_diagnostics")
        if not isinstance(diagnostics, Mapping):
            return False
        errors = diagnostics.get("errors")
        if not isinstance(errors, list):
            return False
        return any(
            isinstance(item, Mapping)
            and str(item.get("code", "")).lower() == "calibration_promotion"
            for item in errors
        )

    valid_claim_rejection = _has_valid_claim_rejection()
    if bundle_status is not None:
        # The evidence-only bundle is expected to return FAIL for the
        # deliberately injected production-claim fixture.  Every other
        # non-PASS state is an unverified/failed evidence run.
        if bundle_status == "FAIL" and evidence.get("production_claim_rejected") is True:
            if not valid_claim_rejection:
                failures.append("bundle failure was not tied to production-claim rejection")
        elif bundle_status not in ALLOWED_STATUSES or bundle_status != "PASS":
            failures.append(f"bundle_status={bundle_status}")
    def _flag_state(value: Any) -> bool | None:
        if isinstance(value, bool):
            return value
        if isinstance(value, (int, float)) and not isinstance(value, bool):
            return value != 0
        if isinstance(value, str):
            lowered = value.strip().lower()
            if lowered in {"true", "yes", "1"}:
                return True
            if lowered in {"false", "no", "0"}:
                return False
        return None

    for key, message in (
        ("production_ready_true", "evidence accepted production_ready=true"),
        ("fire_command_nonzero", "evidence accepted non-zero fire_command"),
        ("motion_nonzero", "evidence accepted non-zero velocity/acceleration"),
    ):
        if key not in evidence:
            continue
        state = _flag_state(evidence.get(key))
        if state is None:
            failures.append(f"{key} is missing or invalid")
        elif state:
            failures.append(message)
    claim_scan = scan_evidence_claims(evidence)
    unsafe_claims = claim_scan.get("unsafe_claims", [])
    # A production_ready=true fixture is intentionally used to prove the
    # rejection path.  It is acceptable only when the caller explicitly
    # records that it was rejected; unsafe control values remain fatal.
    expected_rejection = valid_claim_rejection
    residual = [
        item for item in unsafe_claims
        if not (
            expected_rejection
            and str(item.get("path", "")).lower().rsplit(".", 1)[-1] == "production_ready"
            and item.get("value") is True
        )
        and not (
            expected_rejection
            and str(item.get("path", "")).lower().startswith("$.bundle_diagnostics")
            and "production_ready=true" in str(item.get("value", "")).lower()
        )
    ]
    if residual:
        failures.append("evidence contains unsafe production/control claims")
    return ("FAIL" if failures else "PASS"), failures


def scan_evidence_claims(value: Any) -> dict[str, Any]:
    """Recursively find unsafe production/control claims in evidence data.

    This helper is intentionally independent of the existing evidence
    reporter so a freeze report can prove that its boundary check is not
    relying on a single parser implementation.
    """

    unsafe: list[dict[str, Any]] = []

    unsafe_true_keys = {
        "production_ready",
        "hardware_validation",
        "gimbal_closed_loop_validated",
        "firing_validated",
        "real_hit_rate_computed",
        "ballistic_control_applied",
        "prediction_control_applied",
        "serial_enabled",
        "allow_fire",
        "absolute_command_valid",
        "relative_angle_as_robotctrl",
        "relative_angle_used_as_robotctrl",
        "robotctrl_absolute_angle",
        "gimbal_origin_is_muzzle",
        "gimbal_origin_as_muzzle",
        "muzzle_origin_from_gimbal",
    }
    unsafe_false_keys = {"test_only", "dry_run"}
    motion_keys = {
        "fire_command",
        "yaw_vel",
        "pitch_vel",
        "yaw_acc",
        "pitch_acc",
        "yaw_vel_rad_s",
        "pitch_vel_rad_s",
        "yaw_acc_rad_s2",
        "pitch_acc_rad_s2",
        "velocity",
        "angular_velocity",
        "acceleration",
        "angular_acceleration",
        "fire",
    }

    def _boolish(node: Any) -> bool | None:
        if isinstance(node, bool):
            return node
        if isinstance(node, (int, float)) and not isinstance(node, bool):
            return node != 0
        if isinstance(node, str):
            lowered = node.strip().lower()
            if lowered in {"true", "yes", "1"}:
                return True
            if lowered in {"false", "no", "0"}:
                return False
        return None

    def _unsafe_boolean_key(lowered: str, child: Any) -> bool:
        truth = _boolish(child)
        recognized = lowered in unsafe_true_keys or lowered in unsafe_false_keys
        recognized = recognized or lowered.endswith((
            "_production_ready", "_hardware_validation", "_closed_loop_validated",
            "_firing_validated", "_test_only", "_control_applied",
        ))
        recognized = recognized or (
            "robotctrl" in lowered and ("angle" in lowered or "relative" in lowered)
        )
        recognized = recognized or ("relative" in lowered and "absolute" in lowered)
        recognized = recognized or (
            "gimbal" in lowered and "muzzle" in lowered
            and ("origin" in lowered or "assumption" in lowered)
        )
        if truth is None:
            # An explicitly present safety/production flag with an unknown
            # type is unverifiable and therefore unsafe.  Missing keys are
            # not visited and remain outside this rule.
            return recognized
        if lowered in unsafe_true_keys:
            return truth
        if lowered in unsafe_false_keys:
            return not truth
        # Existing evidence uses component-specific names such as
        # ballistic_production_ready and prediction_test_only.  Enforce the
        # same boundary for those suffix forms without rejecting the normal
        # false/true values respectively.
        if lowered.endswith(("_production_ready", "_hardware_validation", "_closed_loop_validated", "_firing_validated")):
            return truth
        if lowered.endswith("_test_only"):
            return not truth
        if lowered.endswith("_control_applied"):
            return truth
        if "robotctrl" in lowered and ("angle" in lowered or "relative" in lowered):
            return truth
        if "relative" in lowered and "absolute" in lowered:
            return truth
        if "gimbal" in lowered and "muzzle" in lowered and ("origin" in lowered or "assumption" in lowered):
            return truth
        return False

    def _motionish_key(lowered: str) -> bool:
        return any(token in lowered for token in (
            "fire", "velocity", "vel", "acceleration", "angular_acc", "yaw_acc", "pitch_acc",
        ))

    def visit(node: Any, path: str = "$") -> None:
        if isinstance(node, Mapping):
            for key, child in node.items():
                key_text = str(key)
                lowered = key_text.lower()
                child_path = f"{path}.{key_text}"
                if _unsafe_boolean_key(lowered, child):
                    unsafe.append({"path": child_path, "value": child})
                if lowered in motion_keys:
                    try:
                        if isinstance(child, bool):
                            raise ValueError("boolean is not a numeric motion value")
                        if float(child) != 0.0:
                            unsafe.append({"path": child_path, "value": child})
                    except (TypeError, ValueError):
                        unsafe.append({"path": child_path, "value": child})
                elif _motionish_key(lowered) and lowered not in unsafe_true_keys and lowered not in unsafe_false_keys:
                    try:
                        if isinstance(child, bool):
                            if child:
                                unsafe.append({"path": child_path, "value": child})
                        elif float(child) != 0.0:
                            unsafe.append({"path": child_path, "value": child})
                    except (TypeError, ValueError):
                        unsafe.append({"path": child_path, "value": child})
                if lowered == "motion_nonzero":
                    truth = _boolish(child)
                    if truth is None or truth:
                        unsafe.append({"path": child_path, "value": child})
                visit(child, child_path)
        elif isinstance(node, list):
            for index, child in enumerate(node):
                visit(child, f"{path}[{index}]")
        elif isinstance(node, str):
            claim_patterns = (
                r"(?:production_ready|hardware_validation|gimbal_closed_loop_validated|firing_validated|real_hit_rate_computed|ballistic_control_applied|prediction_control_applied|serial_enabled|allow_fire|absolute_command_valid|relative_angle_as_robotctrl|relative_angle_used_as_robotctrl|robotctrl_absolute_angle|gimbal_origin_is_muzzle|gimbal_origin_as_muzzle|muzzle_origin_from_gimbal)\s*[:=]\s*(?:true|yes|1)\b",
                r"(?:test_only|dry_run)\s*[:=]\s*(?:false|no|0)\b",
                r"(?:[A-Za-z0-9_]+_(?:production_ready|hardware_validation|closed_loop_validated|firing_validated|control_applied))\s*[:=]\s*(?:true|yes|1)\b",
                r"[A-Za-z0-9_]*_test_only\s*[:=]\s*(?:false|no|0)\b",
            )
            for pattern in claim_patterns:
                match = re.search(pattern, node, re.IGNORECASE)
                if match:
                    unsafe.append({"path": path, "value": match.group(0)})
                    break

    visit(value)
    return {"status": "FAIL" if unsafe else "PASS", "unsafe_claims": unsafe}


def _ros_safety_status(safety: Mapping[str, Any]) -> tuple[str, list[str]]:
    rounds = safety.get("rounds")
    if not isinstance(rounds, list) or len(rounds) < 3:
        return "NOT_RUN", ["at least three independent ROS safety rounds are required"]
    failures: list[str] = []
    outcomes: list[str] = []
    for index, value in enumerate(rounds, 1):
        if not isinstance(value, Mapping):
            failures.append(f"ROS safety round {index} has invalid record")
            continue
        normalized = dict(value)
        aliases = {
            "yaw_vel": "yaw_vel_rad_s",
            "pitch_vel": "pitch_vel_rad_s",
            "yaw_acc": "yaw_acc_rad_s2",
            "pitch_acc": "pitch_acc_rad_s2",
        }
        for canonical, alias in aliases.items():
            if canonical not in normalized and alias in normalized:
                normalized[canonical] = normalized[alias]
            if alias in normalized:
                try:
                    if float(normalized[alias]) != 0.0:
                        failures.append(f"ROS safety round {index}: {alias} is non-zero")
                except (TypeError, ValueError):
                    failures.append(f"ROS safety round {index}: {alias} is not numeric")
        missing = [key for key in SAFE_DEFAULTS if key not in normalized]
        if missing:
            failures.append(f"ROS safety round {index}: safety fields missing: {', '.join(missing)}")
        default_status, default_violations, normalized_defaults = _check_safety_values(normalized)
        if default_status != "PASS":
            failures.extend([f"ROS safety round {index}: {item}" for item in default_violations])
        normalized.update(normalized_defaults)
        status = _normalise_check_status(value.get("status"))
        normalized["status"] = status
        motion_value = value.get("motion_nonzero", False)
        motion_valid = True
        if isinstance(motion_value, bool):
            motion_nonzero = motion_value
        elif isinstance(motion_value, (int, float)):
            # NaN is not a safe zero; treating every non-zero/non-finite
            # numeric value as motion keeps malformed telemetry fail-closed.
            motion_nonzero = motion_value != 0
        elif isinstance(motion_value, str):
            lowered_motion = motion_value.strip().lower()
            if lowered_motion in {"true", "yes", "1"}:
                motion_nonzero = True
            elif lowered_motion in {"false", "no", "0"}:
                motion_nonzero = False
            else:
                motion_valid = False
                motion_nonzero = True
        else:
            motion_valid = False
            motion_nonzero = True
        if not motion_valid:
            failures.append(f"ROS safety round {index}: motion_nonzero is not a valid boolean")
        normalized["motion_nonzero"] = motion_value if not motion_valid else motion_nonzero
        parse_errors = value.get("_parse_errors", [])
        if isinstance(parse_errors, list):
            failures.extend(f"ROS safety round {index}: {item}" for item in parse_errors)
        observed_fields = value.get("_observed_fields")
        if isinstance(observed_fields, Mapping):
            missing_observed = [key for key in SAFE_DEFAULTS if observed_fields.get(key) is not True]
            if missing_observed:
                failures.append(
                    f"ROS safety round {index}: transcript omitted safety fields: {', '.join(missing_observed)}"
                )
        outcomes.append(status)
        if status != "PASS":
            failures.append(f"ROS safety round {index}: {status}")
        if motion_nonzero:
            failures.append(f"ROS safety round {index}: motion field is non-zero")
    if len(set(outcomes)) > 1:
        failures.append("ROS safety rounds are inconsistent/flaky")
    return ("FAIL" if failures else "PASS"), failures


def _parse_ros_safety_output(text: str) -> dict[str, Any]:
    """Extract explicit safety values from a ROS safety test transcript.

    The integration test itself asserts the complete safe command contract,
    so a quiet successful CTest transcript is evidence of the default values.
    If a runner nevertheless emits explicit values, parse them and let the
    normal safety assessor fail closed on any non-zero or unsafe value.
    """

    fields: dict[str, Any] = dict(SAFE_DEFAULTS)
    fields["motion_nonzero"] = False
    observed: dict[str, bool] = {key: False for key in SAFE_DEFAULTS}
    parse_errors: list[str] = []
    source = str(text or "")
    token = r"(?:[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?|nan|inf(?:inity)?)"

    def parse_number(value: str) -> int | float:
        lowered = value.strip().lower()
        if lowered in {"nan", "+nan", "-nan"}:
            return float("nan")
        if lowered in {"inf", "+inf", "-inf", "infinity", "+infinity", "-infinity"}:
            return float(lowered.replace("infinity", "inf"))
        number = float(value)
        return int(number) if number.is_integer() else number

    def parse_bool(value: str) -> bool | None:
        lowered = value.strip().lower()
        if lowered in {"true", "yes", "1"}:
            return True
        if lowered in {"false", "no", "0"}:
            return False
        return None

    numeric_names = {
        "fire_command": ("fire_command", "fire"),
        "yaw_vel": ("yaw_vel_rad_s", "yaw_vel"),
        "pitch_vel": ("pitch_vel_rad_s", "pitch_vel"),
        "yaw_acc": ("yaw_acc_rad_s2", "yaw_acc"),
        "pitch_acc": ("pitch_acc_rad_s2", "pitch_acc"),
    }
    for canonical, names in numeric_names.items():
        alternatives = "|".join(re.escape(name) for name in names)
        matches = re.findall(
            rf"(?<![A-Za-z0-9_])(?:{alternatives})\s*(?::=|=|:)\s*([^\s,;]+)",
            source,
            flags=re.IGNORECASE,
        )
        if matches:
            for raw_match in matches:
                observed[canonical] = True
                raw_value = raw_match.strip().strip("\"'()[]{}")
                try:
                    if re.fullmatch(token, raw_value, flags=re.IGNORECASE) is None:
                        raise ValueError("invalid numeric safety value")
                    parsed = parse_number(raw_value)
                    if parsed != SAFE_DEFAULTS[canonical]:
                        fields[canonical] = parsed
                except (TypeError, ValueError):
                    # Preserve an explicit malformed token so
                    # _ros_safety_status rejects it instead of silently
                    # retaining the safe default.
                    fields[canonical] = raw_value
                    parse_errors.append(f"{canonical} has invalid value {raw_value!r}")

    for key in ("serial_enabled", "dry_run", "allow_fire", "motion_nonzero"):
        match = re.findall(
            rf"(?<![A-Za-z0-9_]){re.escape(key)}\s*(?::=|=|:)\s*([^\s,;]+)",
            source,
            flags=re.IGNORECASE,
        )
        if match:
            for raw_match in match:
                observed[key] = True
                raw_value = raw_match.strip().strip("\"'()[]{}")
                parsed = parse_bool(raw_value)
                parsed_value = parsed if parsed is not None else raw_value
                if parsed_value != SAFE_DEFAULTS.get(key, False):
                    fields[key] = parsed_value
                if parsed is None:
                    parse_errors.append(f"{key} has invalid value {raw_value!r}")

    if re.search(r"\b(?:non[- ]zero|unsafe)\b[^\n]*(?:fire|motion|velocity|acceleration)", source, re.IGNORECASE):
        fields["motion_nonzero"] = True
    fields["_observed_fields"] = observed
    fields["_parse_errors"] = parse_errors
    return fields


def assess_observations(observations: Mapping[str, Any]) -> dict[str, Any]:
    """Apply the freeze rules to already-collected observations.

    This pure function is the main unit-test seam.  Every status in the
    returned report is one of :data:`ALLOWED_STATUSES`; readiness is kept as
    the explicit candidate value required by the project protocol.
    """

    # Keep the caller's top-level presence information separate from the
    # normalized report shape. Defaults make reports stable, but they must
    # never turn an omitted required observation into a passing result.
    raw_observations = observations if isinstance(observations, Mapping) else {}
    data = _coerce_observations(observations)
    # The full help matrix is required for live runs.  Injected observations
    # may intentionally use the historical two-help fixture; preserve that
    # test seam while still fail-closing an explicitly incomplete live map.
    blockers: list[str] = []
    non_blocking_hardware: list[str] = []
    not_run: list[str] = []
    not_verified: list[str] = []
    required_not_verified: list[str] = []

    git = data["git"] if isinstance(data.get("git"), Mapping) else {}
    git_status = "PASS"
    if git.get("worktree_clean") is not True:
        git_status = "FAIL"
        blockers.append("WORKTREE_DIRTY")
    if git.get("head_matches_origin_main") is not True and git.get("candidate_base_matches_origin_main") is not True:
        git_status = "FAIL"
        blockers.append("HEAD_NOT_AT_CHECKED_MAIN")
    head_sha = git.get("head_sha")
    origin_main_sha = git.get("origin_main_sha")
    if not head_sha:
        git_status = "UNAVAILABLE"
        blockers.append("GIT_HEAD_UNAVAILABLE")
    elif not isinstance(head_sha, str) or re.fullmatch(r"[0-9a-fA-F]{40}", head_sha) is None:
        git_status = "FAIL"
        blockers.append("GIT_HEAD_INVALID")
    if not origin_main_sha:
        git_status = "UNAVAILABLE"
        blockers.append("ORIGIN_MAIN_UNAVAILABLE")
    elif not isinstance(origin_main_sha, str) or re.fullmatch(r"[0-9a-fA-F]{40}", origin_main_sha) is None:
        git_status = "FAIL"
        blockers.append("ORIGIN_MAIN_INVALID")
    diff_check = git.get("diff_check")
    if not isinstance(diff_check, Mapping):
        not_run.append("git_diff_check")
    else:
        diff_check = dict(diff_check)
        diff_check["status"] = _normalise_check_status(diff_check.get("status"))
        git["diff_check"] = diff_check
    if isinstance(diff_check, Mapping) and str(diff_check.get("status")) != "PASS":
        git_status = "FAIL"
        blockers.append("GIT_DIFF_CHECK_FAILED")
    git["status"] = git_status
    data["git"] = dict(git)

    github = data["github"] if isinstance(data.get("github"), Mapping) else {}
    github_status = str(github.get("status", "UNAVAILABLE"))
    if github_status not in ALLOWED_STATUSES:
        github_status = "UNAVAILABLE"
    open_prs_present = (
        isinstance(raw_observations.get("github"), Mapping)
        and "open_prs" in raw_observations["github"]
    )
    open_prs = github.get("open_prs", [])
    # GitHub/PR state is informational only. Historical PR numbers must not
    # become admission blockers: the candidate SHA, worktree, and explicit
    # evidence inputs are the normative release contract.
    if github_status == "NOT_RUN":
        not_run.append("github_status")
    elif github_status not in {"PASS", "UNAVAILABLE", "NOT_VERIFIED"}:
        github_status = "UNAVAILABLE"
    if github_status == "PASS" and (not open_prs_present or not isinstance(open_prs, list)):
        github_status = "UNAVAILABLE"
    github["status"] = github_status
    data["github"] = dict(github)

    if "rosdep" in raw_observations:
        rosdep = data.get("rosdep") if isinstance(data.get("rosdep"), Mapping) else {}
        rosdep = dict(rosdep)
        rosdep_status = _normalise_check_status(rosdep.get("status"))
        rosdep["status"] = rosdep_status
        if rosdep_status == "NOT_RUN":
            not_run.append("rosdep_check")
        elif rosdep_status in {"UNAVAILABLE", "NOT_VERIFIED"}:
            not_verified.append("rosdep_check")
            required_not_verified.append("rosdep_check")
        elif rosdep_status != "PASS":
            blockers.append("ROSDEP_CHECK_FAILED")
        data["rosdep"] = rosdep

    # Wrapper reports are explicit release inputs. A successful shell return
    # is insufficient when the persisted report is absent, failed, or (for
    # ROS E2E) lacks the node-liveness contract.
    for key, blocker_name in (("release_smoke", "RELEASE_SMOKE_FAILED"), ("ros_e2e", "ROS_E2E_FAILED")):
        if key not in raw_observations:
            continue
        wrapper = data.get(key) if isinstance(data.get(key), Mapping) else {}
        wrapper = dict(wrapper)
        wrapper_status = _normalise_check_status(wrapper.get("status"))
        wrapper["status"] = wrapper_status
        if wrapper_status == "NOT_RUN":
            not_run.append(key)
        elif wrapper_status in {"UNAVAILABLE", "NOT_VERIFIED"}:
            not_verified.append(key)
            required_not_verified.append(key)
        elif wrapper_status != "PASS":
            blockers.append(blocker_name)
        nested = wrapper.get("report")
        if isinstance(nested, Mapping):
            nested_status = _normalise_check_status(nested.get("status"))
            if nested_status == "PASS":
                pass
            elif nested_status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"}:
                not_verified.append(f"{key}.report")
                required_not_verified.append(f"{key}.report")
            else:
                blockers.append(f"{blocker_name}_REPORT_NOT_PASS")
        elif wrapper_status == "PASS":
            blockers.append(f"{blocker_name}_REPORT_MISSING")
        data[key] = wrapper

    build = data["build"] if isinstance(data.get("build"), Mapping) else {}
    build = dict(build)
    build_status = _normalise_check_status(build.get("status"))
    build["status"] = build_status
    if build_status != "PASS":
        if build_status == "NOT_RUN":
            not_run.append("offline_build")
        else:
            blockers.append("OFFLINE_BUILD_FAILED")
    data["build"] = dict(build)

    cpp = data["cpp_tests"] if isinstance(data.get("cpp_tests"), Mapping) else {}
    cpp = dict(cpp)
    cpp_status = _normalise_check_status(cpp.get("status"))
    cpp["status"] = cpp_status
    if cpp_status != "PASS":
        if cpp_status == "NOT_RUN":
            not_run.append("cpp_ros_tests")
        else:
            blockers.append("CPP_ROS_TESTS_FAILED")
    total = cpp.get("total")
    if type(total) is not int or total <= 0:
        blockers.append("CPP_TEST_COUNT_INVALID")
    required_results = {}
    if isinstance(cpp.get("tests"), list):
        for item in cpp["tests"]:
            if isinstance(item, Mapping):
                normalized_item = dict(item)
                normalized_item["status"] = _normalise_check_status(item.get("status"))
                required_results[str(item.get("name"))] = normalized_item
        cpp["tests"] = list(required_results.values())
    for name in REQUIRED_CTEST_NAMES:
        result = required_results.get(name)
        if result is None:
            not_run.append(name)
        elif str(result.get("status")) != "PASS":
            blockers.append(f"TEST_{name}_NOT_PASS")
    data["cpp_tests"] = dict(cpp)

    python_tests = data["python_tests"] if isinstance(data.get("python_tests"), Mapping) else {}
    python_tests = {str(key): dict(value) if isinstance(value, Mapping) else {"status": "FAIL"} for key, value in python_tests.items()}
    for group in REQUIRED_PYTHON_GROUPS:
        item = python_tests.get(group, {})
        item_status = _normalise_check_status(item.get("status")) if isinstance(item, Mapping) else "NOT_RUN"
        item["status"] = item_status
        if item_status != "PASS":
            if group == "orin_unavailable" and item_status in {"UNAVAILABLE", "NOT_VERIFIED"}:
                not_verified.append(f"python_{group}")
            elif item_status == "NOT_RUN":
                not_run.append(f"python_{group}")
            else:
                blockers.append(f"PYTHON_{group.upper()}_FAILED")
    data["python_tests"] = dict(python_tests)

    cli_help = data.get("cli_help") if isinstance(data.get("cli_help"), Mapping) else {}
    cli_help = {str(key): dict(value) if isinstance(value, Mapping) else {"status": "FAIL"} for key, value in cli_help.items()}
    # Live runs populate the complete list above.  Injected observations may
    # intentionally provide a smaller legacy subset, so validate every
    # declared key while still requiring the two original offline CLIs.
    raw_cli_help = raw_observations.get("cli_help")
    declared_help_names = (
        [str(name) for name in raw_cli_help.keys()]
        if isinstance(raw_cli_help, Mapping)
        else []
    )
    help_names = list(dict.fromkeys(
        [SCENARIO_HELP_NAME, OFFLINE_HELP_NAME] + declared_help_names
    ))
    for name in help_names:
        item = cli_help.get(name, {}) if isinstance(cli_help, Mapping) else {}
        item_status = _normalise_check_status(item.get("status")) if isinstance(item, Mapping) else "NOT_RUN"
        item["status"] = item_status
        if item_status == "NOT_RUN":
            not_run.append(name)
        elif item_status != "PASS":
            blockers.append(f"CLI_HELP_{name}_FAILED")
    data["cli_help"] = dict(cli_help)

    runtime = data.get("runtime") if isinstance(data.get("runtime"), Mapping) else {}
    openvino = runtime.get("openvino_python", {}) if isinstance(runtime, Mapping) else {}
    runtime_status = _normalise_check_status(openvino.get("status"), "NOT_VERIFIED") if isinstance(openvino, Mapping) else "NOT_VERIFIED"
    if isinstance(openvino, Mapping):
        openvino = dict(openvino)
        openvino["status"] = runtime_status
        runtime = dict(runtime)
        runtime["openvino_python"] = openvino
    if runtime_status in {"UNAVAILABLE", "NOT_VERIFIED"}:
        not_verified.append("runtime.openvino_python")
    elif runtime_status == "NOT_RUN":
        not_run.append("runtime.openvino_python")
    elif runtime_status != "PASS":
        blockers.append("OPENVINO_PYTHON_RUNTIME_FAILED")
    data["runtime"] = dict(runtime)

    camera_preflight = data.get("camera_preflight") if isinstance(data.get("camera_preflight"), Mapping) else {}
    camera_preflight = dict(camera_preflight)
    camera_status = _normalise_check_status(camera_preflight.get("status"))
    camera_preflight["status"] = camera_status
    if camera_status == "NOT_RUN":
        not_run.append("camera_preflight")
    elif camera_status in {"UNAVAILABLE", "NOT_VERIFIED"}:
        not_verified.append("camera_preflight")
    elif camera_status not in {"UNAVAILABLE", "NOT_VERIFIED", "PASS"}:
        blockers.append("CAMERA_PREFLIGHT_FAILED")
    elif camera_status == "PASS":
        # A software-only run cannot establish real camera/SDK readiness; a
        # caller must explicitly mark this as unavailable/not verified.
        blockers.append("CAMERA_PREFLIGHT_CANNOT_CLAIM_HARDWARE_PASS")
    data["camera_preflight"] = dict(camera_preflight)

    scenario = data["scenario"] if isinstance(data.get("scenario"), Mapping) else {}
    scenario_status, scenario_failures = _scenario_consistent(scenario)
    scenario["status"] = scenario_status
    scenario["failures"] = scenario_failures
    if scenario_status == "NOT_RUN":
        not_run.append("scenario_benchmark")
    elif scenario_status != "PASS":
        blockers.append("SCENARIO_BENCHMARK_NOT_DETERMINISTIC")
    data["scenario"] = dict(scenario)

    evidence = data["evidence"] if isinstance(data.get("evidence"), Mapping) else {}
    evidence_status, evidence_failures = _evidence_status(evidence)
    evidence["status"] = evidence_status
    evidence["failures"] = evidence_failures
    if evidence_status == "FAIL":
        blockers.append("EVIDENCE_BOUNDARY_FAILED")
    elif evidence_status == "NOT_RUN":
        not_run.append("offline_evidence")
    data["evidence"] = dict(evidence)

    ros_safety = data["ros_safety"] if isinstance(data.get("ros_safety"), Mapping) else {}
    ros_status, ros_failures = _ros_safety_status(ros_safety)
    ros_safety["status"] = ros_status
    ros_safety["failures"] = ros_failures
    if ros_status == "NOT_RUN":
        not_run.append("ros_safety_three_rounds")
    elif ros_status != "PASS":
        blockers.append("ROS_SAFETY_ROUNDS_NOT_CONSISTENT")
    data["ros_safety"] = dict(ros_safety)

    defaults = data["safety_defaults"] if isinstance(data.get("safety_defaults"), Mapping) else {}
    if "safety_defaults" not in raw_observations:
        blockers.append("SAFETY_DEFAULTS_UNAVAILABLE")
    default_status, default_violations, normalized_defaults = _check_safety_values(defaults.get("values", defaults))
    defaults["status"] = default_status
    defaults["values"] = normalized_defaults
    defaults["violations"] = default_violations
    if default_status != "PASS":
        blockers.append("SAFETY_DEFAULTS_INVALID")
    data["safety_defaults"] = dict(defaults)

    hardware_value = data.get("hardware")
    hardware = dict(hardware_value) if isinstance(hardware_value, Mapping) else {}
    if "hardware" not in raw_observations or not isinstance(hardware_value, Mapping):
        blockers.append("HARDWARE_STATUS_UNAVAILABLE")
    for name in ("model", "calibration", "camera", "orin", "cdc", "gimbal", "firing"):
        item = hardware.get(name)
        if not isinstance(item, Mapping):
            blockers.append(f"HARDWARE_{name.upper()}_STATUS_UNAVAILABLE")
            continue
        item = dict(item)
        item_status = _normalise_check_status(item.get("status"), "NOT_VERIFIED")
        item["status"] = item_status
        hardware[name] = item
        if item_status == "NOT_VERIFIED":
            not_verified.append(name)
            non_blocking_hardware.append(name)
        elif item_status == "UNAVAILABLE":
            non_blocking_hardware.append(name)
        elif item_status == "NOT_RUN":
            not_run.append(f"hardware.{name}")
        elif item_status == "PASS":
            blockers.append(f"HARDWARE_{name.upper()}_CLAIMED_PASS")
        else:
            blockers.append(f"HARDWARE_{name.upper()}_{item_status}")
    data["hardware"] = dict(hardware)

    # Deduplicate while retaining deterministic order.
    blockers = list(dict.fromkeys(blockers))
    non_blocking_hardware = list(dict.fromkeys(non_blocking_hardware))
    not_run = list(dict.fromkeys(not_run))
    not_verified = list(dict.fromkeys(not_verified))

    if blockers:
        candidate_status = "BLOCKED"
    elif not_run:
        candidate_status = "BLOCKED"
    elif required_not_verified:
        candidate_status = "NOT_VERIFIED"
    else:
        candidate_status = "READY_CANDIDATE"

    # The report itself deliberately does not claim hardware readiness.
    data["software_candidate_status"] = candidate_status
    data["blockers"] = blockers
    data["non_blocking_hardware"] = non_blocking_hardware
    data["not_run"] = not_run
    data["not_verified"] = not_verified
    data["required_not_verified"] = list(dict.fromkeys(required_not_verified))
    data["blocker"] = blockers[0] if blockers else None
    return data


def build_report(observations: Mapping[str, Any], *, metadata: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Build the stable top-level report from observations."""

    assessed = assess_observations(observations)
    candidate_status = assessed.get("software_candidate_status")
    report: dict[str, Any] = {
        "schema": REPORT_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if candidate_status == "READY_CANDIDATE" else ("NOT_VERIFIED" if candidate_status == "NOT_VERIFIED" else "FAIL"),
        "software_candidate_status": assessed.pop("software_candidate_status"),
        "blocker": assessed.pop("blocker", None),
        "blockers": assessed.pop("blockers", []),
        "non_blocking_hardware": assessed.pop("non_blocking_hardware", []),
        "not_run": assessed.pop("not_run", []),
        "not_verified": assessed.pop("not_verified", []),
        "safety_boundary": dict(SAFE_DEFAULTS),
        "observations": assessed,
    }
    if metadata:
        # Only allow simple, non-sensitive metadata in the report.
        allowed = {"run_id", "commit", "source_label", "tool_version"}
        report["metadata"] = {key: str(value) for key, value in metadata.items() if key in allowed}
    return report


def _markdown_status(status: Any) -> str:
    return str(status)


def render_markdown(report: Mapping[str, Any]) -> str:
    # Input-manifest mode has a flatter report shape than the live observation
    # mode. Keep the renderer deterministic while retaining the same output
    # filenames and safety boundary.
    if isinstance(report.get("inputs"), list):
        return _render_input_markdown(report)
    observations = report.get("observations", {})
    git = observations.get("git", {}) if isinstance(observations, Mapping) else {}
    github = observations.get("github", {}) if isinstance(observations, Mapping) else {}
    lines = [
        "# Software-freeze candidate gate",
        "",
        f"- Schema: `{report.get('schema')}` v{report.get('schema_version')}",
        f"- Candidate status: `{report.get('software_candidate_status')}`",
        f"- Blocker: `{report.get('blocker') or 'none'}`",
        "",
        "## Git and PR admission",
        "",
        f"- HEAD: `{git.get('head_sha', 'unavailable')}`",
        f"- origin/main: `{git.get('origin_main_sha', 'unavailable')}`",
        f"- Branch: `{git.get('branch', 'unavailable')}`",
        f"- Worktree clean: `{git.get('worktree_clean')}` ({_markdown_status(git.get('status'))})",
        f"- GitHub status: `{github.get('status')}`",
        f"- Open PRs: `{len(github.get('open_prs', [])) if isinstance(github.get('open_prs'), list) else 'unavailable'}`",
        "",
        "## Software checks",
        "",
    ]
    for key, label in (
        ("build", "Offline build"),
        ("cpp_tests", "C++/ROS tests"),
        ("scenario", "Deterministic scenario benchmark"),
        ("evidence", "Evidence report/bundle"),
        ("ros_safety", "ROS safety (three rounds)"),
    ):
        item = observations.get(key, {}) if isinstance(observations, Mapping) else {}
        lines.append(f"- {label}: `{item.get('status', 'NOT_RUN')}`")
    python_tests = observations.get("python_tests", {}) if isinstance(observations, Mapping) else {}
    for key in REQUIRED_PYTHON_GROUPS:
        item = python_tests.get(key, {}) if isinstance(python_tests, Mapping) else {}
        lines.append(f"- Python {key}: `{item.get('status', 'NOT_RUN')}`")
    lines.extend([
        "",
        "## Safety boundary",
        "",
        "```text",
        "serial_enabled=false",
        "dry_run=true",
        "allow_fire=false",
        "fire_command=0",
        "yaw_vel=0",
        "pitch_vel=0",
        "yaw_acc=0",
        "pitch_acc=0",
        "```",
        "",
        "## Hardware and production boundary",
        "",
        "Hardware, model, calibration, camera, Orin, CDC, gimbal, and firing states are recorded as `NOT_VERIFIED` or `UNAVAILABLE`; they are never promoted to a production claim by this tool.",
        "",
        f"- Software blockers: {', '.join(f'`{x}`' for x in report.get('blockers', [])) or 'none'}",
        f"- Non-blocking hardware items: {', '.join(f'`{x}`' for x in report.get('non_blocking_hardware', [])) or 'none'}",
        f"- Not run: {', '.join(f'`{x}`' for x in report.get('not_run', [])) or 'none'}",
        f"- Not verified: {', '.join(f'`{x}`' for x in report.get('not_verified', [])) or 'none'}",
        "",
        "This is software-only admission evidence. It does not open a serial port, camera, Orin, robot, gimbal, or firing path.",
        "",
    ])
    return "\n".join(lines)


def create_manifest(report: Mapping[str, Any], files: Mapping[str, bytes]) -> dict[str, Any]:
    """Create a deterministic SHA-256 manifest for report artifacts."""

    artifacts = [
        {"path": name, "size_bytes": len(payload), "sha256": _sha256_bytes(payload)}
        for name, payload in sorted(files.items())
    ]
    return {
        "schema": REPORT_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "software_candidate_status": report.get("software_candidate_status"),
        "artifacts": artifacts,
        "safety_boundary": dict(SAFE_DEFAULTS),
    }


def write_report_bundle(report: Mapping[str, Any], output_dir: str | Path) -> dict[str, Any]:
    """Write JSON, Markdown and a SHA-256 manifest into a new directory.

    Existing, non-empty, symlinked, or otherwise unsafe output paths fail
    closed before any write.  The function returns the manifest object and
    raises ``ValueError`` for an unsafe destination.
    """

    root = Path(output_dir)
    valid, reason = _validate_output_dir(root)
    if not valid:
        raise ValueError(reason)
    root.mkdir(parents=True, exist_ok=False)
    json_payload = _json_bytes(report)
    markdown_payload = render_markdown(report).encode("utf-8")
    files = {
        "software_freeze_candidate.json": json_payload,
        "software_freeze_candidate.md": markdown_payload,
    }
    manifest = create_manifest(report, files)
    manifest_payload = _json_bytes(manifest)
    files["manifest.json"] = manifest_payload
    for name, payload in files.items():
        _safe_write(root / name, payload)
    # A conventional line-oriented manifest is useful to shell users while
    # the JSON manifest remains the canonical structured artifact.
    lines = [f"{entry['sha256']}  {entry['path']}" for entry in manifest["artifacts"]]
    _safe_write(root / "SHA256SUMS", ("\n".join(lines) + "\n").encode("utf-8"))
    return manifest


def _git_observations(repo_root: Path, runner: Runner, records: list[dict[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    commands = [
        ("git_head", ["git", "rev-parse", "HEAD"]),
        ("git_branch", ["git", "branch", "--show-current"]),
        ("git_status", ["git", "status", "--porcelain", "--untracked-files=all"]),
        ("git_diff_check", ["git", "diff", "--check"]),
        ("git_origin_main", ["git", "ls-remote", "origin", "refs/heads/main"]),
    ]
    outputs: dict[str, CommandResult] = {}
    for name, argv in commands:
        item = _call_runner(runner, argv, cwd=repo_root)
        outputs[name] = item
        records.append(_command_record(name, argv, item, cwd=repo_root))
    head = outputs["git_head"].stdout.strip() if outputs["git_head"].ok else ""
    branch = outputs["git_branch"].stdout.strip() if outputs["git_branch"].ok else ""
    status_lines = _parse_status_lines(outputs["git_status"].stdout)
    origin = _parse_sha_from_ls_remote(outputs["git_origin_main"].stdout) if outputs["git_origin_main"].ok else None
    candidate_base_matches = False
    if origin:
        # Compare against the SHA returned by this run's ls-remote query.  A
        # local ``origin/main`` ref may be stale when another PR merges while
        # the gate is running; consulting it would incorrectly bless an old
        # candidate.  merge-base is read-only and does not update refs.
        ancestor_argv = ["git", "merge-base", "--is-ancestor", origin, "HEAD"]
        ancestor_result = _call_runner(runner, ancestor_argv, cwd=repo_root)
        records.append(_command_record("git_main_ancestor", ancestor_argv, ancestor_result, cwd=repo_root))
        candidate_base_matches = ancestor_result.ok
    result.update({
        "head_sha": head or None,
        "origin_main_sha": origin,
        "branch": branch or None,
        "worktree_clean": not status_lines if outputs["git_status"].ok else None,
        "status_lines": [_redact_text(line, repo_root=repo_root) for line in status_lines],
        "head_matches_origin_main": bool(head and origin and head.lower() == origin.lower()),
        "candidate_base_matches_origin_main": candidate_base_matches,
        "diff_check": {
            "status": "PASS" if outputs["git_diff_check"].ok else "FAIL",
            "exit_code": outputs["git_diff_check"].returncode,
        },
    })
    return result


def _github_observations(repo: str, runner: Runner, records: list[dict[str, Any]]) -> dict[str, Any]:
    list_argv = ["gh", "pr", "list", "--repo", repo, "--state", "open", "--json", "number,title,state,isDraft,reviewDecision,headRefName,baseRefName,url"]
    listed = _call_runner(runner, list_argv, timeout=60.0)
    records.append(_command_record("github_open_prs", list_argv, listed))
    if not listed.ok:
        return {"status": "UNAVAILABLE", "open_prs": [], "prs": {}, "error": _redact_text(listed.stderr)}
    open_prs = _parse_json(listed.stdout)
    if not isinstance(open_prs, list):
        return {"status": "UNAVAILABLE", "open_prs": [], "prs": {}, "error": "gh returned invalid JSON"}
    if any(not isinstance(value, Mapping) for value in open_prs):
        # Do not silently discard malformed entries: an invalid PR record
        # could hide an open feature PR and incorrectly produce an empty-list
        # admission result.
        return {
            "status": "UNAVAILABLE",
            "open_prs": [],
            "prs": {},
            "error": "gh returned a non-object open PR record",
        }
    open_prs = [_normalise_pr(value) for value in open_prs]
    # Historical PR numbers are deliberately not part of the freeze
    # contract. Open-PR data is informational only; release admission is
    # decided from the candidate SHA, worktree, and explicit software evidence.
    return {"status": "PASS", "open_prs": open_prs, "prs": {}}


def _run_single_test_command(
    name: str,
    argv: Sequence[str],
    repo_root: Path,
    runner: Runner,
    records: list[dict[str, Any]],
    *,
    timeout: float = 3600.0,
) -> dict[str, Any]:
    result = _call_runner(runner, argv, cwd=repo_root, timeout=timeout)
    records.append(_command_record(name, argv, result, cwd=repo_root))
    return _test_result_from_text(name, result.stdout + "\n" + result.stderr, result.returncode)


def _read_regular_file(path: Path) -> tuple[bytes | None, str | None]:
    try:
        info = path.lstat()
        if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
            return None, "not a regular non-link file"
        return path.read_bytes(), None
    except OSError as exc:
        return None, str(exc)


def _scenario_file_record(path: Path) -> dict[str, Any]:
    payload, error = _read_regular_file(path)
    if payload is None:
        return {"status": "FAIL", "error": error}
    return {"status": "PASS", "size_bytes": len(payload), "sha256": _sha256_bytes(payload)}


def _scenario_csv_invariants(path: Path) -> dict[str, Any]:
    """Inspect every benchmark row for the software-only safety contract."""

    payload, error = _read_regular_file(path)
    if payload is None:
        return {"status": "FAIL", "failures": [error or "benchmark.csv unavailable"]}
    failures: list[str] = []
    origins: set[str] = set()
    row_count = 0
    try:
        text = payload.decode("utf-8")
        reader = csv.DictReader(text.splitlines())
        required = {
            "synthetic", "test_only", "production_ready", "origin_assumption",
            "fire_command", "yaw_vel_rad_s", "pitch_vel_rad_s",
            "yaw_acc_rad_s2", "pitch_acc_rad_s2", "serial_enabled",
            "dry_run", "allow_fire",
        }
        missing = sorted(required.difference(reader.fieldnames or ()))
        if missing:
            failures.append("benchmark.csv missing fields: " + ", ".join(missing))
        for row in reader:
            row_count += 1
            origins.add(str(row.get("origin_assumption", "")))
            expected = {
                "synthetic": {"1", "true"},
                "test_only": {"1", "true"},
                "production_ready": {"0", "false"},
                "fire_command": {"0"},
                "serial_enabled": {"0", "false"},
                "dry_run": {"1", "true"},
                "allow_fire": {"0", "false"},
            }
            for key, wanted in expected.items():
                actual = str(row.get(key, "")).strip().lower()
                accepted = {str(item).lower() for item in wanted}
                if actual not in accepted:
                    failures.append(f"row {row_count}: {key} expected one of {sorted(accepted)}, got {row.get(key)!r}")
            origin = str(row.get("origin_assumption", "")).strip()
            ballistic_reason = str(row.get("ballistic_reason", "")).strip()
            if origin != SCENARIO_ORIGIN_ASSUMPTION and not (
                origin == "omitted_for_intentional_failure"
                and ballistic_reason == "missing_muzzle_transform"
            ):
                failures.append(
                    f"row {row_count}: origin_assumption must be {SCENARIO_ORIGIN_ASSUMPTION}"
                )
            for key in ("yaw_vel_rad_s", "pitch_vel_rad_s", "yaw_acc_rad_s2", "pitch_acc_rad_s2"):
                try:
                    if float(row.get(key, "nan")) != 0.0:
                        failures.append(f"row {row_count}: {key} is non-zero")
                except (TypeError, ValueError):
                    failures.append(f"row {row_count}: {key} is not numeric")
    except (UnicodeError, csv.Error) as exc:
        failures.append(f"invalid benchmark.csv: {exc}")
    if row_count == 0:
        failures.append("benchmark.csv has no data rows")
    if SCENARIO_ORIGIN_ASSUMPTION not in origins:
        failures.append("benchmark.csv has no synthetic_muzzle_frame origin rows")
    return {
        "status": "PASS" if not failures else "FAIL",
        "rows": row_count,
        "origin_assumptions": sorted(origins),
        "failures": failures,
    }


def _scenario_observations(
    repo_root: Path,
    runner: Runner,
    records: list[dict[str, Any]],
    *,
    scenario_root: Path | None = None,
    install_base: Path | None = None,
) -> dict[str, Any]:
    # Resolve the installed benchmark through ROS 2 rather than relying on a
    # developer's PATH.  The caller has already built/sourced the isolated
    # install for a live gate; the wrapper keeps that contract explicit.
    ros_prefix = "source /opt/ros/humble/setup.bash && "
    install_setup = str(install_base) if install_base is not None else os.environ.get(
        "GAME26_FREEZE_INSTALL", "/tmp/game26-freeze-install"
    )
    ros_prefix += "source " + shlex.quote(install_setup) + "/setup.bash && "
    help_argv = [
        "bash", "-lc",
        ros_prefix + "ros2 run auto_aim_ros2 auto_aim_scenario_benchmark --help",
    ]
    help_result = _call_runner(runner, help_argv, cwd=repo_root, timeout=60.0)
    records.append(_command_record("scenario_benchmark_help", help_argv, help_result, cwd=repo_root))
    if not help_result.ok:
        return {"status": "FAIL", "help": _test_result_from_text("scenario_benchmark_help", help_result.stdout + help_result.stderr, help_result.returncode), "runs": []}
    explicit_scenario_root = scenario_root is not None
    if explicit_scenario_root:
        root = Path(scenario_root)
        if not root.is_absolute():
            # The runner executes with ``cwd=repo_root``; resolve relative
            # scenario roots against the same directory before validating or
            # passing them to the installed benchmark.
            root = repo_root / root
    else:
        root = Path(tempfile.mkdtemp(prefix="game26-freeze-scenario-"))
    if explicit_scenario_root:
        # ``--scenario-root`` is a parent for the two benchmark output
        # directories.  The C++ benchmark requires that parent to already
        # exist, while each child output directory must be new.  Never create
        # or reuse the parent implicitly: that would either violate the
        # benchmark contract or make an unsafe path look fresh.
        try:
            root_info = root.lstat()
            if not stat.S_ISDIR(root_info.st_mode) or stat.S_ISLNK(root_info.st_mode):
                raise OSError("scenario root must be an existing non-symlink directory")
            if not _safe_path_component(root):
                raise OSError("scenario root contains a symlink")
        except OSError as exc:
            return {
                "status": "FAIL",
                "help": {"status": "PASS"},
                "runs": [],
                "failures": [f"unsafe scenario output root: {exc}"],
            }
    run_records: list[dict[str, Any]] = []
    for suffix in ("a", "b"):
        output_dir = root / suffix
        valid_output, output_reason = _validate_output_dir(output_dir)
        if not valid_output:
            return {
                "status": "FAIL",
                "help": {"status": "PASS"},
                "runs": run_records,
                "failures": [f"unsafe scenario output directory {suffix}: {output_reason}"],
            }
        argv = [
            "bash", "-lc",
            ros_prefix
            + "ros2 run auto_aim_ros2 auto_aim_scenario_benchmark "
            + "--scenario all --seed 260033 --output-dir "
            + shlex.quote(str(output_dir)),
        ]
        result = _call_runner(runner, argv, cwd=repo_root, timeout=900.0)
        records.append(_command_record(f"scenario_benchmark_{suffix}", argv, result, cwd=repo_root))
        run: dict[str, Any] = {
            "status": "PASS" if result.ok else "FAIL",
            "files": {},
            "synthetic": None,
            "test_only": None,
            "production_ready": None,
            "software_only_synthetic_benchmark": None,
            "origin_assumption": None,
            "safety": {},
            "csv_invariants": {"status": "NOT_RUN"},
        }
        if result.ok:
            for filename in REQUIRED_SCENARIO_FILES:
                run["files"][filename] = _scenario_file_record(output_dir / filename)
            run["csv_invariants"] = _scenario_csv_invariants(output_dir / "benchmark.csv")
            if run["csv_invariants"].get("status") != "PASS":
                run["status"] = "FAIL"
            assumptions = run["csv_invariants"].get("origin_assumptions", [])
            if isinstance(assumptions, list) and SCENARIO_ORIGIN_ASSUMPTION in assumptions:
                # Intentional ballistic-failure rows may omit a muzzle frame;
                # the normal rows still prove the documented origin contract.
                run["origin_assumption"] = SCENARIO_ORIGIN_ASSUMPTION
            summary_payload, _ = _read_regular_file(output_dir / "summary.json")
            parsed = _parse_json(summary_payload.decode("utf-8", errors="replace")) if summary_payload else None
            if isinstance(parsed, Mapping):
                run["synthetic"] = parsed.get("synthetic")
                run["test_only"] = parsed.get("test_only")
                run["production_ready"] = parsed.get("production_ready")
                run["software_only_synthetic_benchmark"] = parsed.get("software_only_synthetic_benchmark")
                summary_safety = parsed.get("safety", {})
                if not isinstance(summary_safety, Mapping):
                    summary_safety = {}
                merged_safety = dict(summary_safety)
                # The benchmark keeps the zero-motion safety fields both in
                # the summary map (with rad/s aliases) and in the top-level
                # JSON/CLI contract.  Verify all required fields together.
                for key in SAFE_DEFAULTS:
                    if key in parsed:
                        merged_safety[key] = parsed[key]
                run["safety"] = merged_safety
        run_records.append(run)
    if len(run_records) == 2:
        for filename in REQUIRED_SCENARIO_FILES:
            left, left_error = _read_regular_file(root / "a" / filename)
            right, right_error = _read_regular_file(root / "b" / filename)
            equal = left is not None and right is not None and left == right
            run_records[0]["files"].setdefault(filename, {})["bytes_equal"] = equal
            run_records[1]["files"].setdefault(filename, {})["bytes_equal"] = equal
            if left_error or right_error:
                run_records[0]["files"].setdefault(filename, {})["error"] = left_error or right_error
    status, failures = _scenario_consistent({"runs": run_records})
    return {"status": status, "help": {"status": "PASS"}, "runs": run_records, "failures": failures}


def _evidence_observations(
    repo_root: Path,
    runner: Runner,
    records: list[dict[str, Any]],
) -> dict[str, Any]:
    """Run the existing evidence report/bundle tools against checked fixtures.

    The normal fixture must pass.  The bundle is also run with the checked
    evidence-only ``production_claim.yaml`` fixture; its non-zero exit is the
    required proof that a production claim is rejected rather than silently
    promoted.
    """

    fixture_csv = repo_root / "tools" / "offline_evidence_report" / "fixtures" / "normal.csv"
    claim_yaml = repo_root / "tools" / "offline_evidence_report" / "fixtures" / "bundle" / "production_claim.yaml"
    try:
        scratch = Path(tempfile.mkdtemp(prefix="game26-freeze-evidence-"))
    except OSError as exc:
        return {"status": "UNAVAILABLE", "production_claim_rejected": False, "error": str(exc)}
    report_json = scratch / "report.json"
    report_md = scratch / "report.md"
    report_argv = [
        "python3", "tools/offline_evidence_report/auto_aim_evidence_report.py",
        "--input-csv", str(fixture_csv), "--json-report", str(report_json), "--markdown-report", str(report_md),
    ]
    report_result = _call_runner(runner, report_argv, cwd=repo_root, timeout=300.0)
    records.append(_command_record("offline_evidence_report", report_argv, report_result, cwd=repo_root))

    bundle_dir = scratch / "bundle"
    bundle_argv = [
        "python3", "tools/offline_evidence_report/offline_evidence_bundle.py",
        "--input-csv", str(fixture_csv), "--output-dir", str(bundle_dir),
        "--camera-intrinsic-report", str(claim_yaml), "--mode", "evidence_only",
    ]
    bundle_result = _call_runner(runner, bundle_argv, cwd=repo_root, timeout=300.0)
    records.append(_command_record("offline_evidence_bundle_production_claim", bundle_argv, bundle_result, cwd=repo_root))
    # Existing bundle CLI prints only ``status=FAIL`` and persists the precise
    # diagnostic in manifest.json.  Inspect both channels so a future CLI
    # change cannot accidentally turn a rejected production claim into a
    # passing evidence check merely because stderr wording changed.
    combined = (bundle_result.stdout + "\n" + bundle_result.stderr).lower()
    manifest_claim_rejection = False
    manifest_path = bundle_dir / "manifest.json"
    manifest_payload, _ = _read_regular_file(manifest_path)
    manifest_value = _parse_json(manifest_payload.decode("utf-8", errors="replace")) if manifest_payload else None
    if isinstance(manifest_value, Mapping):
        diagnostics = manifest_value.get("diagnostics", {})
        errors = diagnostics.get("errors", []) if isinstance(diagnostics, Mapping) else []
        if isinstance(errors, list):
            for item in errors:
                if not isinstance(item, Mapping):
                    continue
                code = str(item.get("code", "")).lower()
                message = str(item.get("message", "")).lower()
                if code == "calibration_promotion" or "production_ready=true" in message:
                    manifest_claim_rejection = True
                    break
    explicit_claim_signal = any(
        marker in combined
        for marker in ("calibration_promotion", "production_ready=true", "evidence_only calibration")
    )
    claim_rejected = bundle_result.returncode != 0 and (manifest_claim_rejection or explicit_claim_signal)
    result: dict[str, Any] = {
        "status": "PASS" if report_result.ok and claim_rejected else "FAIL",
        "report_status": "PASS" if report_result.ok else "FAIL",
        "report_exit_code": report_result.returncode,
        "bundle_status": "PASS" if bundle_result.ok else "FAIL",
        "bundle_exit_code": bundle_result.returncode,
        "production_claim_rejected": claim_rejected,
        "bundle_claim_rejection_signal": bool(manifest_claim_rejection or explicit_claim_signal),
        "claims": {"production_ready": False},
    }
    if isinstance(manifest_value, Mapping):
        result["bundle_report_status"] = manifest_value.get("status")
        result["bundle_diagnostics"] = manifest_value.get("diagnostics", {})
    if report_result.ok:
        for path in (report_json, report_md):
            payload, error = _read_regular_file(path)
            if payload is None:
                result["status"] = "FAIL"
                result.setdefault("errors", []).append(f"missing evidence output: {error}")
    return result


def _external_report_observation(
    path: Path,
    *,
    kind: str,
    candidate: Mapping[str, Any],
    base_dir: Path,
) -> dict[str, Any]:
    """Read one wrapper-produced report at its explicit path.

    Release-smoke and ROS-E2E wrappers have different report payloads, but
    both publish the same schema/status/safety/hash boundary.  Reuse the
    input-manifest adapter instead of copying either producer's parser.
    """

    declaration = {"id": kind, "kind": kind, "path": str(path), "skip_ctest": True}
    record, _ = _manifest_source_record(declaration, base_dir=base_dir, candidate=candidate)
    return record


def run_gate(
    repo_root: str | Path,
    *,
    runner: Runner | None = None,
    github_repo: str = "Miqi9880/game_26_team",
    scenario_root: str | Path | None = None,
    run_expensive: bool = True,
) -> dict[str, Any]:
    """Collect live observations and return an assessed report.

    ``runner`` can be replaced with a fake command/result provider.  With
    ``run_expensive=False`` no build/test/scenario commands are started; the
    report records those checks as ``NOT_RUN``.
    """

    root = Path(repo_root).resolve()
    command_records: list[dict[str, Any]] = []
    command_runner = runner or _default_runner
    observations = _default_observation()
    observations["git"] = _git_observations(root, command_runner, command_records)
    # GitHub state is retained as an optional informational observation.  The
    # freeze gate never hardcodes historical PR numbers or treats API access
    # as proof that a candidate is ready.
    observations["github"] = _github_observations(github_repo, command_runner, command_records)
    if run_expensive:
        # Every gate invocation gets an isolated build/install/log root.  A
        # fixed /tmp path could reuse stale CTest metadata or let concurrent
        # runs contaminate each other's evidence.  The generated paths are
        # external and are redacted from command records.
        scratch_root = Path(tempfile.mkdtemp(prefix="game26-freeze-run-"))
        build_base = scratch_root / "build"
        install_base = scratch_root / "install"
        log_base = scratch_root / "log"
        build_text = shlex.quote(str(build_base))
        install_text = shlex.quote(str(install_base))
        log_text = shlex.quote(str(log_base))
        package_text = " ".join(FREEZE_COLCON_PACKAGES)
        rosdep_argv = [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && rosdep check --from-paths src --ignore-src",
        ]
        rosdep_result = _call_runner(command_runner, rosdep_argv, cwd=root, timeout=900.0)
        command_records.append(_command_record("rosdep_check", rosdep_argv, rosdep_result, cwd=root))
        rosdep_text = (rosdep_result.stdout + "\n" + rosdep_result.stderr).lower()
        if rosdep_result.ok:
            rosdep_status = "PASS"
        elif rosdep_result.returncode in {126, 127} or "not been initialized" in rosdep_text or "command not found" in rosdep_text:
            rosdep_status = "UNAVAILABLE"
        else:
            # A dependency error is evidence that this environment cannot
            # verify the full build.  Keep it distinct from a source failure.
            rosdep_status = "NOT_VERIFIED"
        observations["rosdep"] = {
            "status": rosdep_status,
            "exit_code": rosdep_result.returncode,
            "reason": None if rosdep_status == "PASS" else "rosdep dependency check unavailable or incomplete",
            "command": _display_command(rosdep_argv),
        }
        source_cmd = (
            "source /opt/ros/humble/setup.bash && colcon --log-base " + log_text
            + " build --symlink-install --build-base " + build_text
            + " --install-base " + install_text
            + " --packages-select " + package_text
            + " --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON"
        )
        build_argv = ["bash", "-lc", source_cmd]
        build_result = _call_runner(command_runner, build_argv, cwd=root, timeout=3600.0)
        command_records.append(_command_record("offline_build", build_argv, build_result, cwd=root))
        # Store the same redacted command representation used by the command
        # ledger; the per-run scratch paths must not leak into the report.
        observations["build"] = {
            "status": "PASS" if build_result.ok else "FAIL",
            "exit_code": build_result.returncode,
            "command": _display_command(build_argv),
        }

        # Do not start tests or dependent probes after a failed build: their
        # outputs would be misleading and the final report should identify
        # the first causal failure.
        if not build_result.ok:
            observations["commands"] = command_records
            return build_report(observations, metadata={"tool_version": f"{REPORT_SCHEMA}-v{SCHEMA_VERSION}"})

        test_cmd = (
            "source /opt/ros/humble/setup.bash && source " + install_text + "/setup.bash"
            + " && colcon --log-base " + log_text + " test --build-base " + build_text
            + " --install-base " + install_text
            + " --packages-select " + package_text
            + " --event-handlers console_direct+"
        )
        test_argv = ["bash", "-lc", test_cmd]
        test_result = _call_runner(command_runner, test_argv, cwd=root, timeout=3600.0)
        command_records.append(_command_record("colcon_test", test_argv, test_result, cwd=root))
        result_text = test_result.stdout + "\n" + test_result.stderr
        counts = _extract_test_counts(result_text)
        tests = []
        discovered_ctest_names = _discover_ctest_names(build_base)
        xml_results = _parse_ctest_xml_results(build_base)
        xml_by_name = {str(item.get("name")): item for item in xml_results}
        for name in REQUIRED_CTEST_NAMES:
            # A test is considered covered only when CTest declares it in the
            # generated test files.  XML gives per-test counts for gtests;
            # Python/special tests retain the colcon exit result and summary.
            if name in xml_by_name:
                tests.append(dict(xml_by_name[name]))
            elif name in discovered_ctest_names:
                tests.append({"name": name, "status": "PASS" if test_result.ok else "FAIL"})
            else:
                tests.append({"name": name, "status": "NOT_RUN"})
        observations["cpp_tests"] = {"status": "PASS" if test_result.ok else "FAIL", **counts, "tests": tests}

        result_cmd = [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source " + install_text + "/setup.bash && "
            "colcon test-result --test-result-base " + build_text + " --verbose",
        ]
        result_result = _call_runner(command_runner, result_cmd, cwd=root, timeout=600.0)
        command_records.append(_command_record("colcon_test_result", result_cmd, result_result, cwd=root))
        result_counts = _extract_test_counts(result_result.stdout + "\n" + result_result.stderr)
        observations["cpp_tests"].update({key: value for key, value in result_counts.items() if value is not None})
        observations["cpp_tests"]["test_result_exit_code"] = result_result.returncode
        if xml_results:
            # XML totals cover gtests only; colcon test-result remains the
            # authoritative package-wide total for Python/special tests.
            observations["cpp_tests"]["xml_tests"] = xml_results
        if (
            not result_result.ok
            or (result_counts.get("failed") or 0) > 0
            or (result_counts.get("errors") or 0) > 0
            or (result_counts.get("skipped") or 0) > 0
        ):
            observations["cpp_tests"]["status"] = "FAIL"

        diff = observations["git"].get("diff_check", {})
        if isinstance(diff, Mapping) and diff.get("status") != "PASS":
            observations["git"]["status"] = "FAIL"

        py_commands = {
            "qualification": ["python3", "-m", "unittest", "discover", "-s", "tools/auto_aim_qualification/test", "-p", "test_*.py"],
            "offline_evidence_report": ["python3", "-m", "unittest", "discover", "-s", "tools/offline_evidence_report/test", "-p", "test_auto_aim_evidence_report.py"],
            "offline_evidence_bundle": ["python3", "-m", "unittest", "discover", "-s", "tools/offline_evidence_report/test", "-p", "test_offline_evidence_bundle.py"],
        }
        for group, argv in py_commands.items():
            observations["python_tests"][group] = _run_single_test_command(group, argv, root, command_runner, command_records, timeout=900.0)
        runtime_argv = ["python3", "-c", "import openvino"]
        runtime_result = _call_runner(command_runner, runtime_argv, cwd=root, timeout=60.0)
        command_records.append(_command_record("openvino_python_runtime_probe", runtime_argv, runtime_result, cwd=root))
        runtime_status = "PASS" if runtime_result.ok else ("UNAVAILABLE" if runtime_result.returncode in {1, 126, 127} else "FAIL")
        observations["runtime"] = {
            "openvino_python": {
                "status": runtime_status,
                "exit_code": runtime_result.returncode,
                "reason": "OpenVINO Python runtime unavailable; C++/offline evidence remains separate" if runtime_status == "UNAVAILABLE" else None,
            }
        }
        observations["hardware"]["model"] = {
            "status": "UNAVAILABLE",
            "reason": "No production XML/BIN model artifact is checked in; model qualification cannot claim runtime inference",
        }
        # The Orin preflight is a CMake executable, not a Python package.  A
        # missing executable is a truthful UNAVAILABLE/NOT_VERIFIED result;
        # it is never promoted to a hardware PASS.
        orin_build_dir = Path(tempfile.mkdtemp(prefix="game26-freeze-orin-"))
        orin_build_text = shlex.quote(str(orin_build_dir))
        orin_argv = [
            "bash", "-lc",
            "cmake -S tools/orin_hardware_evidence -B " + orin_build_text
            + " -DBUILD_TESTING=ON && cmake --build " + orin_build_text
            + " && ctest --test-dir " + orin_build_text + " --output-on-failure",
        ]
        orin_result = _call_runner(command_runner, orin_argv, cwd=root, timeout=1800.0)
        command_records.append(_command_record("orin_environment_preflight_test", orin_argv, orin_result, cwd=root))
        observations["python_tests"]["orin_unavailable"] = {
            "name": "orin_environment_preflight_test",
            "status": "PASS" if orin_result.ok else ("UNAVAILABLE" if orin_result.returncode in {126, 127} else "FAIL"),
            "exit_code": orin_result.returncode,
        }

        # Run the two reviewed release wrappers explicitly.  They each create
        # a fresh output root and retain their own JSON/Markdown evidence;
        # this gate records the wrapper result and hashes only the report at
        # the path it supplied.  No report is found by filename elsewhere.
        candidate_for_reports = {
            "head": observations["git"].get("head_sha"),
            "main_baseline": observations["git"].get("origin_main_sha"),
            "branch": observations["git"].get("branch"),
            "worktree_clean": observations["git"].get("worktree_clean"),
        }
        release_smoke_root = scratch_root / "release-smoke"
        release_smoke_script = root / "src" / "auto_aim_release_smoke" / "scripts" / "run_release_smoke.sh"
        release_smoke_argv = [
            "bash", str(release_smoke_script),
            "--workspace-root", str(root),
            "--output-root", str(release_smoke_root),
            "--baseline", str(observations["git"].get("origin_main_sha") or ""),
        ]
        release_smoke_result = _call_runner(command_runner, release_smoke_argv, cwd=root, timeout=3600.0)
        command_records.append(_command_record("release_smoke", release_smoke_argv, release_smoke_result, cwd=root))
        observations["release_smoke"] = {
            "status": "PASS" if release_smoke_result.ok else "FAIL",
            "exit_code": release_smoke_result.returncode,
            "command": _display_command(release_smoke_argv),
            "report": _external_report_observation(
                release_smoke_root / "report" / "smoke-report.json",
                kind="release_smoke", candidate=candidate_for_reports, base_dir=root,
            ),
        }

        ros_e2e_root = scratch_root / "ros-message-e2e"
        ros_e2e_script = root / "src" / "auto_aim_ros_e2e" / "scripts" / "run_ros_message_e2e.sh"
        ros_e2e_argv = [
            "bash", str(ros_e2e_script),
            "--workspace-root", str(root),
            "--output-root", str(ros_e2e_root),
            "--baseline", str(observations["git"].get("origin_main_sha") or ""),
            "--rounds", "5",
        ]
        ros_e2e_result = _call_runner(command_runner, ros_e2e_argv, cwd=root, timeout=3600.0)
        command_records.append(_command_record("ros_message_e2e", ros_e2e_argv, ros_e2e_result, cwd=root))
        observations["ros_e2e"] = {
            "status": "PASS" if ros_e2e_result.ok else "FAIL",
            "exit_code": ros_e2e_result.returncode,
            "command": _display_command(ros_e2e_argv),
            "report": _external_report_observation(
                ros_e2e_root / "report" / "ros-message-e2e-report.json",
                kind="ros_e2e", candidate=candidate_for_reports, base_dir=root,
            ),
        }
        # Exercise every installed/source CLI's ``--help`` path explicitly.
        # Help probes are read-only and do not require a model, video, camera,
        # serial device, or a ROS graph.
        ros_help_prefix = "source /opt/ros/humble/setup.bash && source " + install_text + "/setup.bash && "
        help_commands = [
            ("auto_aim_scenario_benchmark_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_scenario_benchmark --help"]),
            ("auto_aim_offline_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_offline --help"]),
            ("auto_aim_dry_run_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_dry_run --help"]),
            ("auto_aim_detector_smoke_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_detector_smoke --help"]),
            ("auto_aim_pnp_smoke_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_pnp_smoke --help"]),
            ("auto_aim_camera_calibrate_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_ros2 auto_aim_camera_calibrate --help"]),
            ("auto_aim_calibration_dataset_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_tools auto_aim_calibration_dataset --help"]),
            ("auto_aim_calibration_dataset_recorder_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_tools auto_aim_calibration_dataset_recorder --help"]),
            ("ros_input_preflight_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_tools ros_input_preflight --help"]),
            ("release_manifest_audit_help", ["bash", "-lc", ros_help_prefix + "ros2 run release_manifest_audit release_manifest_audit --help"]),
            ("auto_aim_release_smoke_help", ["bash", "-lc", ros_help_prefix + "ros2 run auto_aim_release_smoke auto_aim_release_smoke --help"]),
            ("release_smoke_help", ["bash", str(release_smoke_script), "--help"]),
            ("ros_message_e2e_help", ["bash", str(ros_e2e_script), "--help"]),
            ("model_qualification_help", ["python3", str(root / "tools" / "auto_aim_qualification" / "auto_aim_qualification.py"), "--help"]),
            ("offline_evidence_report_help", ["python3", str(root / "tools" / "offline_evidence_report" / "auto_aim_evidence_report.py"), "--help"]),
            ("offline_evidence_bundle_help", ["python3", str(root / "tools" / "offline_evidence_report" / "offline_evidence_bundle.py"), "--help"]),
            ("orin_environment_preflight_help", [str(orin_build_dir / "orin_environment_preflight"), "--help"]),
        ]
        for name, argv in help_commands:
            help_result = _call_runner(command_runner, argv, cwd=root, timeout=60.0)
            command_records.append(_command_record(name, argv, help_result, cwd=root))
            observations.setdefault("cli_help", {})[name] = {
                "status": "PASS" if help_result.ok else "FAIL",
                "exit_code": help_result.returncode,
            }

        # Run the ROS safety target independently at least three times.  A
        # first failure followed by success is retained as inconsistent and
        # therefore blocks the candidate; it is never hidden by the final
        # successful run.
        safety_rounds: list[dict[str, Any]] = []
        safety_argv = [
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash && source " + install_text
            + "/setup.bash && ctest --test-dir "
            + shlex.quote(str(build_base / "auto_aim_ros2"))
            + " -R ros_safety_integration_test --verbose",
        ]
        for index in range(3):
            safety_result = _call_runner(command_runner, safety_argv, cwd=root, timeout=900.0)
            command_records.append(_command_record(f"ros_safety_round_{index + 1}", safety_argv, safety_result, cwd=root))
            parsed_safety = _parse_ros_safety_output(safety_result.stdout + "\n" + safety_result.stderr)
            safety_rounds.append({
                "round": index + 1,
                "status": "PASS" if safety_result.ok else "FAIL",
                "exit_code": safety_result.returncode,
                **parsed_safety,
            })
        observations["ros_safety"] = {"rounds": safety_rounds}
        # Keep qualification/evidence diagnostics separate from hard blockers.
        # Their evidence-only WARN state is useful context but cannot claim
        # production readiness.
        observations["scenario"] = _scenario_observations(
            root,
            command_runner,
            command_records,
            scenario_root=Path(scenario_root) if scenario_root else None,
            install_base=install_base,
        )
        observations["evidence"] = _evidence_observations(root, command_runner, command_records)
    observations["commands"] = command_records
    return build_report(observations, metadata={"tool_version": f"{REPORT_SCHEMA}-v{SCHEMA_VERSION}"})


def _load_observations(path: Path) -> dict[str, Any]:
    payload, error = _read_regular_file(path)
    if payload is None:
        raise ValueError(error or "cannot read observations")
    value = _parse_json(payload.decode("utf-8"))
    if not isinstance(value, Mapping):
        raise ValueError("observations JSON must be an object")
    return dict(value)


# ---------------------------------------------------------------------------
# Explicit-input admission mode
# ---------------------------------------------------------------------------
#
# ``run_gate`` above is intentionally convenient for a local, source-tree
# smoke run.  A release freeze, however, must consume the exact evidence that
# a caller names.  The input-manifest mode is a small, read-only adapter for
# that workflow: it hashes each declared path, validates the common report
# contract, and delegates any domain-specific interpretation to the report
# producers.  In particular it does not search for files by basename or
# reimplement the CSV/evidence parsers.

INPUT_MANIFEST_SCHEMA = "software-freeze-inputs"
INPUT_MANIFEST_VERSION = 1
_SOFTWARE_INPUT_KINDS = frozenset({
    "build", "ctest", "release_smoke", "ros_e2e", "ros_message_e2e",
    "scenario_benchmark", "scenario", "model_qualification",
    "qualification", "calibration", "evidence", "evidence_report",
    "evidence_bundle", "bundle", "orin_preflight", "test",
})
_HARDWARE_INPUT_KINDS = frozenset({
    "model", "production_model", "formal_model", "formal_calibration",
    "camera", "orin", "cdc", "gimbal", "firing", "hardware",
})
_REQUIRED_ARTIFACT_ROLES = (
    "model_xml", "model_bin", "model_profile", "pnp_config",
    "calibration_manifest",
)


def _manifest_kind(value: Any) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value or "").strip().lower()).strip("_")


def _manifest_path(raw: Any, *, base_dir: Path) -> Path:
    if not isinstance(raw, str) or not raw.strip():
        raise ValueError("declared input path must be a non-empty string")
    # ``abspath`` normalises ``.`` and ``..`` without resolving symlinks.  A
    # symlink is rejected by the regular-file reader below, so a declaration
    # can never silently resolve to a substitute artifact.
    value = Path(raw).expanduser()
    if not value.is_absolute():
        value = base_dir / value
    return Path(os.path.abspath(str(value)))


def _manifest_sha(value: Any, *, context: str) -> str | None:
    if value is None:
        return None
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-fA-F]{64}", value) is None:
        raise ValueError(f"{context} sha256 must be 64 hexadecimal characters")
    return value.lower()


def _manifest_status(value: Any) -> str | None:
    """Map producer status values into the gate's conservative vocabulary."""

    if value is None:
        return None
    text = str(value).strip().upper()
    if text == "WARN":
        return "NOT_VERIFIED"
    if text == "READY_CANDIDATE":
        return "PASS"
    if text == "BLOCKED":
        return "FAIL"
    return text if text in ALLOWED_STATUSES else None


def _manifest_status_hint(root: Mapping[str, Any]) -> tuple[str | None, str | None]:
    for key in ("status", "overall_status", "result", "software_candidate_status"):
        if key in root:
            raw = root.get(key)
            status = _manifest_status(raw)
            return status, str(raw)
    return None, None


def _manifest_find_safety(value: Any) -> Mapping[str, Any] | None:
    """Find an explicit safety map without assuming a producer's nesting."""

    if isinstance(value, Mapping):
        keys = {str(key) for key in value}
        if any(key in keys for key in SAFE_DEFAULTS) or any(
            key in keys for key in ("yaw_vel_rad_s", "pitch_vel_rad_s", "yaw_acc_rad_s2", "pitch_acc_rad_s2")
        ):
            # ``values`` is used by the software gate's own report.  Prefer
            # it when present so metadata around it cannot hide omissions.
            nested = value.get("values")
            if isinstance(nested, Mapping):
                return nested
            return value
        for child in value.values():
            found = _manifest_find_safety(child)
            if found is not None:
                return found
    elif isinstance(value, list):
        for child in value:
            found = _manifest_find_safety(child)
            if found is not None:
                return found
    return None


def _manifest_normalise_safety(value: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(value)
    aliases = {
        "yaw_vel": "yaw_vel_rad_s",
        "pitch_vel": "pitch_vel_rad_s",
        "yaw_acc": "yaw_acc_rad_s2",
        "pitch_acc": "pitch_acc_rad_s2",
    }
    for canonical, alias in aliases.items():
        if canonical not in result and alias in result:
            result[canonical] = result[alias]
    return result


def _manifest_ctest_counts(path: Path) -> tuple[dict[str, int] | None, str | None]:
    """Parse one explicitly supplied CTest XML result file."""

    payload, error = _read_regular_file(path)
    if payload is None:
        return None, error or "CTest XML is unavailable"
    import xml.etree.ElementTree as ET
    try:
        root = ET.fromstring(payload)
    except ET.ParseError as exc:
        return None, f"CTest XML is malformed: {exc}"
    local_name = lambda tag: str(tag).rsplit("}", 1)[-1]
    if local_name(root.tag) != "Site":
        return None, "CTest XML root must be Site"
    testing_nodes = [node for node in root if isinstance(node.tag, str) and local_name(node.tag) == "Testing"]
    if len(testing_nodes) != 1:
        return None, "CTest XML must contain exactly one Testing element"
    records = [node for node in testing_nodes[0] if isinstance(node.tag, str) and local_name(node.tag) == "Test"]
    if not records:
        return None, "CTest XML contains no Test result records"
    counts = {"PASS": 0, "FAIL": 0, "UNAVAILABLE": 0, "NOT_RUN": 0, "NOT_VERIFIED": 0}
    for node in records:
        raw = node.attrib.get("Status")
        if raw is None:
            return None, "CTest result Test record has no Status attribute"
        status = raw.strip().lower().replace("-", "_")
        if status == "passed":
            counts["PASS"] += 1
        elif status == "failed":
            counts["FAIL"] += 1
        elif status in {"notrun", "not_run", "skipped"}:
            counts["NOT_RUN"] += 1
        else:
            return None, f"CTest Test record has unsupported Status: {raw}"
    return counts, None


def _manifest_number(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float) and value.is_integer():
        return int(value)
    return None


def _manifest_compare_ctest(root: Mapping[str, Any], xml_path: Path) -> str | None:
    counts_value = root.get("counts")
    cases_value = root.get("cases")
    if not isinstance(counts_value, Mapping) or not isinstance(cases_value, list):
        return "CTest-backed report must provide both counts and cases"
    actual, error = _manifest_ctest_counts(xml_path)
    if actual is None:
        return error
    for key, expected in actual.items():
        if _manifest_number(counts_value.get(key)) != expected:
            return f"report counts disagree with CTest XML for {key}"
    if len(cases_value) != sum(actual.values()):
        return "report case total disagrees with CTest XML"
    return None


def _manifest_role(value: Any) -> str:
    token = _manifest_kind(value)
    token = re.sub(r"(?:_)?sha(?:256)?$", "", token)
    aliases = {
        "xml": "model_xml", "model": "model_xml", "model_xml_file": "model_xml",
        "bin": "model_bin", "model_bin_file": "model_bin",
        "profile": "model_profile", "model_yaml": "model_profile",
        "pnp": "pnp_config", "pnp_yaml": "pnp_config",
        "calibration": "calibration_manifest", "calibration_manifest_json": "calibration_manifest",
    }
    if token in aliases:
        return aliases[token]
    for role in _REQUIRED_ARTIFACT_ROLES:
        if token == role or token.endswith("_" + role):
            return role
    return token


def _manifest_hash_references(value: Any, path: str = "$") -> list[tuple[str, str, str]]:
    references: list[tuple[str, str, str]] = []
    if isinstance(value, Mapping):
        role_hint = _manifest_role(value.get("role", value.get("id", "")))
        for key, child in value.items():
            child_path = f"{path}.{key}"
            key_text = str(key).lower()
            if isinstance(child, str) and re.fullmatch(r"[0-9a-fA-F]{64}", child):
                if key_text in {"sha256", "sha_256", "digest"} and role_hint:
                    references.append((role_hint, child.lower(), child_path))
                elif "sha256" in key_text:
                    references.append((_manifest_role(key_text), child.lower(), child_path))
            references.extend(_manifest_hash_references(child, child_path))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            references.extend(_manifest_hash_references(child, f"{path}[{index}]"))
    return references


def _manifest_source_record(
    declaration: Mapping[str, Any],
    *,
    base_dir: Path,
    candidate: Mapping[str, Any],
) -> tuple[dict[str, Any], Mapping[str, Any] | None]:
    source_id = declaration.get("id")
    kind = _manifest_kind(declaration.get("kind"))
    record: dict[str, Any] = {
        "id": str(source_id) if isinstance(source_id, str) else "",
        "kind": kind,
        "path": None,
        "schema": None,
        "sha256": None,
        "status": "FAIL",
        "failure_reasons": [],
    }
    reasons: list[str] = record["failure_reasons"]
    invalid_identity = not isinstance(source_id, str) or not source_id.strip()
    invalid_identity = invalid_identity or not kind
    if not isinstance(source_id, str) or not source_id.strip():
        reasons.append("source id must be a non-empty string")
    if not kind:
        reasons.append("source kind must be a non-empty string")
    try:
        path = _manifest_path(declaration.get("path"), base_dir=base_dir)
    except ValueError as exc:
        reasons.append(str(exc))
        return record, None
    record["path"] = str(path)
    absence_raw = declaration.get("absence_status")
    absence = _manifest_status(absence_raw)
    invalid_absence = absence_raw is not None and absence not in {"UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}
    if invalid_absence:
        reasons.append("absence_status must be UNAVAILABLE, NOT_RUN, or NOT_VERIFIED")
        absence = None
    payload, read_error = _read_regular_file(path)
    if payload is None:
        if absence is not None:
            record["status"] = absence
            reasons.append(f"source unavailable: explicit {absence}")
        else:
            reasons.append(read_error or "source is missing or not a regular file")
        record["reason"] = reasons[0] if reasons else None
        return record, None
    record["size_bytes"] = len(payload)
    digest = _sha256_bytes(payload)
    record["sha256"] = digest
    fatal = invalid_absence or invalid_identity
    unverified = False
    try:
        expected_digest = _manifest_sha(declaration.get("sha256"), context=f"source {source_id}")
    except ValueError as exc:
        reasons.append(str(exc))
        expected_digest = None
        fatal = True
    if expected_digest is not None and digest != expected_digest:
        reasons.append("declared SHA-256 mismatch")
        fatal = True
    parsed = _parse_json(payload.decode("utf-8", errors="replace"))
    if not isinstance(parsed, Mapping):
        reasons.append("source is not a JSON object")
        record["reason"] = reasons[0]
        return record, None
    root = dict(parsed)
    record["schema"] = root.get("schema", root.get("schema_version"))
    version = root.get("schema_version")
    if _manifest_number(version) != INPUT_MANIFEST_VERSION:
        reasons.append("unknown or missing schema_version")
        fatal = True
        fatal = True
    declared_schema = declaration.get("schema")
    if declared_schema is not None and root.get("schema") != declared_schema:
        reasons.append("declared schema does not match source schema")
        fatal = True
        fatal = True

    declared_status, raw_status = _manifest_status_hint(root)
    if declared_status is None:
        reasons.append("missing or invalid report status")
        fatal = True
    elif declared_status == "FAIL":
        reasons.append(f"source reports {raw_status}")
        fatal = True
    elif declared_status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"}:
        reasons.append(f"source reports {raw_status}")
        unverified = True
    if scan_evidence_claims(root).get("status") != "PASS":
        reasons.append("production or unsafe control claim detected")
        fatal = True

    required_safety = kind in _SOFTWARE_INPUT_KINDS or kind in _HARDWARE_INPUT_KINDS
    if required_safety:
        safety = _manifest_find_safety(root)
        if safety is None:
            reasons.append("safety fields are missing")
            fatal = True
        else:
            safety_status, safety_failures, _ = _check_safety_values(_manifest_normalise_safety(safety))
            if safety_status != "PASS":
                reasons.extend(f"safety: {item}" for item in safety_failures)
                fatal = True

    if kind in {"scenario_benchmark", "scenario", "calibration", "evidence", "evidence_report", "evidence_bundle", "bundle", "qualification", "model_qualification"}:
        synthetic_expected = (("synthetic", True), ("test_only", True), ("production_ready", False))
        for key, expected in synthetic_expected:
            if root.get(key) is not expected:
                reasons.append(f"missing or contradictory {key}")
                fatal = True

    ctest_declared = declaration.get("ctest_xml")
    if ctest_declared is not None:
        try:
            ctest_path = _manifest_path(ctest_declared, base_dir=base_dir)
            record["ctest_xml"] = str(ctest_path)
        except ValueError as exc:
            reasons.append(str(exc))
            ctest_path = None
        if ctest_path is not None:
            ctest_failure = _manifest_compare_ctest(root, ctest_path)
            if ctest_failure:
                reasons.append(ctest_failure)
                fatal = True
    elif ("counts" in root or "cases" in root) and not declaration.get("skip_ctest"):
        reasons.append("CTest-backed report requires explicit ctest_xml")
        fatal = True

    head = candidate.get("head", candidate.get("head_sha"))
    baseline = candidate.get("main_baseline", candidate.get("main_baseline_sha"))
    def _sha_equal(left: Any, right: Any) -> bool:
        return isinstance(left, str) and isinstance(right, str) and left.lower() == right.lower()

    for key in ("commit", "candidate_commit", "head_sha", "candidate_sha"):
        if key in root and not _sha_equal(root.get(key), head):
            reasons.append("candidate Git SHA mismatch")
            fatal = True
            break
    for key in ("baseline_main", "main_baseline", "baseline_sha"):
        if key in root and not _sha_equal(root.get(key), baseline):
            reasons.append("main baseline SHA mismatch")
            fatal = True
            break

    if kind in {"ros_e2e", "ros_message_e2e"}:
        liveness = root.get("node_liveness")
        valid_liveness = isinstance(liveness, Mapping)
        if valid_liveness:
            alive = liveness.get("alive_during_sampling")
            expected_exit = liveness.get("expected_exit_code")
            observed_exit = liveness.get("observed_exit_code")
            valid_liveness = (
                alive is True
                and _manifest_number(expected_exit) is not None
                and _manifest_number(observed_exit) is not None
                and _manifest_number(expected_exit) == _manifest_number(observed_exit)
            )
        if not valid_liveness:
            reasons.append("node_liveness evidence missing or contradictory")
            unverified = True

    if fatal:
        record["status"] = "FAIL"
    elif unverified or absence is not None or declared_status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"}:
        record["status"] = declared_status if declared_status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"} else "NOT_VERIFIED"
    else:
        record["status"] = "PASS"
    record["reason"] = reasons[0] if reasons else None
    return record, root


def _manifest_artifact_record(
    declaration: Mapping[str, Any], *, base_dir: Path
) -> dict[str, Any]:
    role = _manifest_role(declaration.get("role", declaration.get("id", "")))
    record: dict[str, Any] = {
        "role": role,
        "id": str(declaration.get("id", role)),
        "path": None,
        "sha256": None,
        "status": "FAIL",
        "failure_reasons": [],
    }
    reasons: list[str] = record["failure_reasons"]
    try:
        path = _manifest_path(declaration.get("path"), base_dir=base_dir)
    except ValueError as exc:
        reasons.append(str(exc))
        record["reason"] = reasons[0]
        return record
    record["path"] = str(path)
    absence_raw = declaration.get("absence_status")
    absence = _manifest_status(absence_raw)
    if absence_raw is not None and absence not in {"UNAVAILABLE", "NOT_RUN", "NOT_VERIFIED"}:
        reasons.append("absence_status must be UNAVAILABLE, NOT_RUN, or NOT_VERIFIED")
        absence = None
    payload, read_error = _read_regular_file(path)
    if payload is None:
        if absence is not None:
            record["status"] = absence
            reasons.append(f"artifact unavailable: explicit {absence}")
        else:
            reasons.append(read_error or "artifact is missing or not a regular file")
        record["reason"] = reasons[0]
        return record
    record["size_bytes"] = len(payload)
    digest = _sha256_bytes(payload)
    record["sha256"] = digest
    try:
        expected = _manifest_sha(declaration.get("sha256"), context=f"artifact {role}")
    except ValueError as exc:
        reasons.append(str(exc))
        expected = None
    if expected is not None and expected != digest:
        reasons.append("declared SHA-256 mismatch")
    record["status"] = "FAIL" if reasons else "PASS"
    record["reason"] = reasons[0] if reasons else None
    return record


def build_input_manifest_report(
    manifest: Mapping[str, Any], *, manifest_path: str | Path | None = None
) -> dict[str, Any]:
    """Assess a caller-provided, explicit software-freeze input manifest.

    The function is pure apart from reading the paths named in ``manifest``.
    It never searches a directory, invokes a producer, or substitutes a
    similarly named artifact.  Relative paths are resolved against the
    manifest file's directory (or the current directory for API callers).
    """

    if not isinstance(manifest, Mapping):
        raise ValueError("input manifest must be a JSON object")
    if _manifest_kind(manifest.get("schema")) not in {INPUT_MANIFEST_SCHEMA, "software_freeze_inputs"}:
        raise ValueError(f"input manifest schema must be {INPUT_MANIFEST_SCHEMA}")
    if _manifest_number(manifest.get("schema_version")) != INPUT_MANIFEST_VERSION:
        raise ValueError("input manifest requires schema_version 1")
    base_dir = Path(manifest_path).resolve().parent if manifest_path is not None else Path.cwd()
    candidate_value = manifest.get("candidate")
    if not isinstance(candidate_value, Mapping):
        raise ValueError("input manifest requires candidate object")
    candidate = dict(candidate_value)
    head = candidate.get("head", candidate.get("head_sha"))
    baseline = candidate.get("main_baseline", candidate.get("main_baseline_sha"))
    branch = candidate.get("branch")
    candidate_failures: list[str] = []
    if not isinstance(head, str) or re.fullmatch(r"[0-9a-fA-F]{40}", head) is None:
        candidate_failures.append("candidate head must be a 40-character SHA-1")
    if not isinstance(baseline, str) or re.fullmatch(r"[0-9a-fA-F]{40}", baseline) is None:
        candidate_failures.append("candidate main_baseline must be a 40-character SHA-1")
    if not isinstance(branch, str) or not branch.strip():
        candidate_failures.append("candidate branch must be a non-empty string")
    if candidate.get("worktree_clean") is not True:
        candidate_failures.append("candidate worktree_clean must be true")
    for observed_key, expected in (("observed_head", head), ("observed_head_sha", head), ("observed_main_baseline", baseline)):
        if observed_key in candidate and candidate.get(observed_key) != expected:
            candidate_failures.append(f"candidate {observed_key} does not match declared SHA")

    declarations = manifest.get("inputs", manifest.get("sources"))
    if not isinstance(declarations, list) or not declarations:
        raise ValueError("input manifest requires a non-empty inputs array")
    source_records: list[dict[str, Any]] = []
    source_roots: list[tuple[dict[str, Any], Mapping[str, Any]]] = []
    ids: set[str] = set()
    for declaration in declarations:
        if not isinstance(declaration, Mapping):
            raise ValueError("every input declaration must be an object")
        source_id = declaration.get("id")
        if not isinstance(source_id, str) or not source_id.strip():
            raise ValueError("every input declaration requires a non-empty id")
        if source_id in ids:
            raise ValueError(f"duplicate input id: {source_id}")
        ids.add(source_id)
        record, root = _manifest_source_record(declaration, base_dir=base_dir, candidate=candidate)
        source_records.append(record)
        if root is not None:
            source_roots.append((record, root))

    artifact_declarations = manifest.get("artifacts", [])
    if artifact_declarations is None:
        artifact_declarations = []
    if not isinstance(artifact_declarations, list):
        raise ValueError("artifacts must be an array")
    artifact_records = [_manifest_artifact_record(item, base_dir=base_dir) for item in artifact_declarations if isinstance(item, Mapping)]
    if len(artifact_records) != len(artifact_declarations):
        raise ValueError("every artifact declaration must be an object")

    blockers = list(candidate_failures)
    not_verified: list[str] = []
    non_blocking_hardware: list[str] = []
    source_by_id = {item["id"]: item for item in source_records}
    hardware_kinds = _HARDWARE_INPUT_KINDS
    for item in source_records:
        status = item.get("status")
        kind = _manifest_kind(item.get("kind"))
        if status == "FAIL":
            blockers.append(f"INPUT_{item['id']}_FAILED")
        elif status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"}:
            if kind in hardware_kinds:
                non_blocking_hardware.append(item["id"])
            else:
                not_verified.append(item["id"])
    for item in artifact_records:
        status = item.get("status")
        role = str(item.get("role", item.get("id", "artifact")))
        if status == "FAIL":
            blockers.append(f"ARTIFACT_{role.upper()}_FAILED")
        elif status in {"NOT_RUN", "UNAVAILABLE", "NOT_VERIFIED"}:
            non_blocking_hardware.append(role)

    required_kinds = manifest.get("required_kinds", [])
    if required_kinds is not None:
        if not isinstance(required_kinds, list) or any(not isinstance(value, str) for value in required_kinds):
            raise ValueError("required_kinds must be an array of strings")
        seen_kinds = {_manifest_kind(item.get("kind")) for item in source_records}
        for required in required_kinds:
            normalized = _manifest_kind(required)
            if normalized and normalized not in seen_kinds:
                not_verified.append(f"missing:{normalized}")

    # A duplicate role or a producer's declared role hash disagreement is a
    # safety contradiction, not an optional hardware gap.
    artifact_hashes: dict[str, str] = {}
    consistency_failures: list[str] = []
    for item in artifact_records:
        digest = item.get("sha256")
        role = _manifest_role(item.get("role", item.get("id", "")))
        if item.get("status") == "PASS" and isinstance(digest, str):
            previous = artifact_hashes.get(role)
            if previous is not None and previous != digest:
                consistency_failures.append(f"artifact hash differs for {role}")
            artifact_hashes[role] = digest
    for record, root in source_roots:
        for role, digest, path in _manifest_hash_references(root):
            expected = artifact_hashes.get(role)
            if expected is not None and expected != digest:
                consistency_failures.append(f"{record['id']} hash reference {path} disagrees for {role}")
    for reference_root, reference_label in ((candidate, "$.candidate"), (manifest.get("expected_hashes"), "$.expected_hashes")):
        if isinstance(reference_root, Mapping):
            for role, digest, path in _manifest_hash_references(reference_root, reference_label):
                expected = artifact_hashes.get(role)
                if expected is not None and expected != digest:
                    consistency_failures.append(f"hash reference {path} disagrees for {role}")
    blockers.extend(f"HASH_CONSISTENCY_{index + 1}" for index, _ in enumerate(consistency_failures))

    # Missing formal model/calibration/PnP artifacts remain explicit
    # NOT_VERIFIED evidence.  They never become a production PASS and do not
    # block a software-only candidate when every software input passed.
    for role in _REQUIRED_ARTIFACT_ROLES:
        if role not in artifact_hashes:
            non_blocking_hardware.append(role)

    blockers = list(dict.fromkeys(blockers))
    not_verified = list(dict.fromkeys(not_verified))
    non_blocking_hardware = list(dict.fromkeys(non_blocking_hardware))
    if blockers:
        status = "BLOCKED"
    elif not_verified:
        status = "NOT_VERIFIED"
    else:
        status = "READY_CANDIDATE"
    return {
        "schema": REPORT_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "software_candidate_status": status,
        "candidate": candidate,
        "inputs": sorted(source_records, key=lambda item: (str(item.get("id")), str(item.get("kind")))),
        "artifacts": sorted(artifact_records, key=lambda item: (str(item.get("role")), str(item.get("id")))),
        "artifact_hashes": {key: artifact_hashes[key] for key in sorted(artifact_hashes)},
        "consistency": {
            "candidate_sha_consistent": not candidate_failures,
            "artifact_hash_consistent": not consistency_failures,
            "failures": consistency_failures,
        },
        "blocker": blockers[0] if blockers else None,
        "blockers": blockers,
        "not_verified": not_verified,
        "non_blocking_hardware": non_blocking_hardware,
        "safety_boundary": dict(SAFE_DEFAULTS),
    }


def _render_input_markdown(report: Mapping[str, Any]) -> str:
    lines = [
        "# Software-freeze candidate input audit", "",
        f"- Schema: `{report.get('schema')}` v{report.get('schema_version')}",
        f"- Candidate status: `{report.get('software_candidate_status')}`",
        f"- Blocker: `{report.get('blocker') or 'none'}`", "",
        "## Candidate", "",
    ]
    candidate = report.get("candidate", {})
    if isinstance(candidate, Mapping):
        lines.extend([
            f"- HEAD: `{candidate.get('head', candidate.get('head_sha', 'unavailable'))}`",
            f"- main baseline: `{candidate.get('main_baseline', candidate.get('main_baseline_sha', 'unavailable'))}`",
            f"- branch: `{candidate.get('branch', 'unavailable')}`",
            f"- worktree clean: `{candidate.get('worktree_clean')}`",
        ])
    lines.extend(["", "## Explicit inputs", "", "| id | kind | status | SHA-256 | reason |", "|---|---|---|---|---|"])
    inputs = report.get("inputs", [])
    if isinstance(inputs, list):
        for item in inputs:
            if not isinstance(item, Mapping):
                continue
            lines.append(
                f"| `{item.get('id', '')}` | `{item.get('kind', '')}` | `{item.get('status', 'FAIL')}` | "
                f"`{item.get('sha256') or 'unavailable'}` | {item.get('reason') or ''} |"
            )
    lines.extend(["", "## Explicit artifacts", "", "| role | status | SHA-256 | reason |", "|---|---|---|---|"])
    artifacts = report.get("artifacts", [])
    if isinstance(artifacts, list):
        for item in artifacts:
            if not isinstance(item, Mapping):
                continue
            lines.append(
                f"| `{item.get('role', '')}` | `{item.get('status', 'FAIL')}` | `{item.get('sha256') or 'unavailable'}` | {item.get('reason') or ''} |"
            )
    lines.extend([
        "", "## Safety boundary", "", "```text",
        "serial_enabled=false", "dry_run=true", "allow_fire=false",
        "fire_command=0", "yaw_vel=0", "pitch_vel=0", "yaw_acc=0", "pitch_acc=0",
        "```", "",
        f"- Blockers: {', '.join(f'`{x}`' for x in report.get('blockers', [])) or 'none'}",
        f"- Not verified: {', '.join(f'`{x}`' for x in report.get('not_verified', [])) or 'none'}",
        f"- Hardware gaps: {', '.join(f'`{x}`' for x in report.get('non_blocking_hardware', [])) or 'none'}",
        "", "This audit reads only the paths explicitly declared by the caller; it does not locate substitutes or connect to hardware.", "",
    ])
    return "\n".join(lines)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Read-only software-freeze candidate admission gate")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd(), help="worktree to inspect")
    parser.add_argument("--output-dir", type=Path, required=True, help="new directory for JSON/Markdown/manifest")
    parser.add_argument("--github-repo", default="Miqi9880/game_26_team")
    parser.add_argument("--scenario-root", type=Path, help="parent for the two scenario output directories")
    parser.add_argument("--observations-json", type=Path, help="test/debug seam: assess injected observations instead of running commands")
    parser.add_argument(
        "--input-manifest", "--inputs-json", "--manifest-config",
        dest="input_manifest", type=Path,
        help="explicit JSON input manifest; no files are discovered by basename",
    )
    parser.add_argument("--no-expensive", action="store_true", help="record build/tests/scenario as NOT_RUN")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.input_manifest and args.observations_json:
            raise ValueError("--input-manifest and --observations-json are mutually exclusive")
        if args.input_manifest:
            report = build_input_manifest_report(
                _load_observations(args.input_manifest),
                manifest_path=args.input_manifest,
            )
        elif args.observations_json:
            report = build_report(_load_observations(args.observations_json))
        else:
            report = run_gate(
                args.repo_root,
                github_repo=args.github_repo,
                scenario_root=args.scenario_root,
                run_expensive=not args.no_expensive,
            )
        manifest = write_report_bundle(report, args.output_dir)
        print(json.dumps({
            "software_candidate_status": report["software_candidate_status"],
            "blocker": report.get("blocker"),
            "output_dir": "<output-dir>",
            "manifest_artifacts": len(manifest["artifacts"]),
        }, ensure_ascii=False, sort_keys=True))
        status = report["software_candidate_status"]
        if status == "READY_CANDIDATE":
            return 0
        if status == "NOT_VERIFIED":
            return 2
        return 1
    except (OSError, ValueError, TypeError) as exc:
        # Fail closed without creating a partial report in an unsafe path.
        print(f"software_freeze_gate: FAIL: {_redact_text(str(exc))}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
