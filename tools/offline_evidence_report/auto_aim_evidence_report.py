#!/usr/bin/env python3
"""Read-only quality and safety report for ``auto_aim_offline`` CSV files.

The module deliberately uses only the Python standard library.  It does not
connect to ROS, a camera, a serial device, or a robot and it never interprets
the CSV as a request to perform an action.  A malformed input still produces
the most complete report that can be made from the valid rows seen so far.

The public :func:`analyze_csv` and :func:`build_report` helpers are useful for
tests and callers that do not want to use the command line interface.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from collections import Counter, OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence, TextIO, Union


REQUIRED_COLUMNS = (
    "frame",
    "stamp_ns",
    "tracking_state",
    "selected",
    "target_lock",
    "fire_command",
    "test_only",
)

KNOWN_TRACKING_STATES = ("lost", "detecting", "tracking", "temp_lost")
VALID_TARGET_LOCKS = (49, 50)

INTEGER_COLUMNS = {
    "frame",
    "stamp_ns",
    "detection_count",
    "valid_pnp_count",
    "detection_index",
    "class_id",
    "pnp_valid",
    "track_id",
    "consecutive_valid",
    "target_lock",
    "fire_command",
}
FLOAT_COLUMNS = {
    "confidence",
    "reprojection_error_px",
    "camera_x_m",
    "camera_y_m",
    "camera_z_m",
    "gimbal_x_m",
    "gimbal_y_m",
    "gimbal_z_m",
    "relative_yaw_rad",
    "relative_pitch_rad",
    "command_yaw_rad",
    "command_pitch_rad",
    "command_yaw_degree",
    "command_pitch_degree",
}
BOOL_COLUMNS = {"selected", "test_only", "absolute_command_valid"}

@dataclass
class ParsedRecord:
    """One CSV row with values converted where a conversion is possible."""

    row_number: int
    values: dict[str, Any]
    raw: dict[str, str]


@dataclass
class Analysis:
    """Intermediate parse result retained for callers and report generation."""

    input_name: str = ""
    headers: list[str] = field(default_factory=list)
    records: list[ParsedRecord] = field(default_factory=list)
    # Number of physical data rows encountered, including rows that could not
    # be converted.  Reports distinguish this from parsed_rows.
    data_row_count: int = 0
    errors: list[dict[str, Any]] = field(default_factory=list)
    warnings: list[dict[str, Any]] = field(default_factory=list)
    metadata: dict[str, Any] = field(default_factory=dict)

    def error(self, code: str, message: str, row: Optional[int] = None, **extra: Any) -> None:
        item: dict[str, Any] = {"code": code, "message": message}
        if row is not None:
            item["row"] = row
        item.update(extra)
        self.errors.append(item)

    def warning(self, code: str, message: str, row: Optional[int] = None, **extra: Any) -> None:
        item: dict[str, Any] = {"code": code, "message": message}
        if row is not None:
            item["row"] = row
        item.update(extra)
        self.warnings.append(item)


def _as_name(value: Any) -> str:
    """Return a safe input name without leaking a complete local path."""

    if value is None:
        return ""
    if hasattr(value, "read"):
        value = getattr(value, "name", "")
        if not value:
            return ""
    try:
        text = str(value)
        # ``Path`` follows the host OS only; normalize both separators so a
        # Windows path passed to a Linux/WSL invocation cannot leak parents.
        return Path(text.replace("\\", "/")).name
    except (OSError, ValueError):
        return str(value).rsplit("/", 1)[-1].rsplit("\\", 1)[-1]


def _finite_float(value: str) -> Optional[float]:
    try:
        number = float(value.strip())
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def _parse_integer(value: str) -> Optional[int]:
    text = value.strip()
    if not text:
        return None
    # int("1.0") is intentionally rejected: CSV integer columns have an
    # unambiguous representation and accepting a float hides schema mistakes.
    try:
        return int(text, 10)
    except (TypeError, ValueError):
        return None


def _parse_bool(value: str) -> Optional[bool]:
    text = value.strip().lower()
    if text in {"1", "true", "yes"}:
        return True
    if text in {"0", "false", "no"}:
        return False
    return None


def _raw_value(row: Mapping[str, str], key: str) -> str:
    value = row.get(key, "")
    return "" if value is None else str(value)


def _convert_row(
    analysis: Analysis,
    row_number: int,
    row: Mapping[str, str],
    headers: Sequence[str],
) -> Optional[ParsedRecord]:
    values: dict[str, Any] = {}
    raw = {name: _raw_value(row, name) for name in headers}

    # Required fields are required on every usable row.  Optional fields are
    # allowed to be blank because auto_aim_offline intentionally leaves pose
    # and track columns blank for rejected/no-detection records.
    for name in REQUIRED_COLUMNS:
        text = raw.get(name, "").strip()
        if not text:
            analysis.error("empty_required_value", f"required column {name!r} is empty", row_number, column=name)
            return None
        if name in {"tracking_state"}:
            values[name] = text.lower()
        elif name in BOOL_COLUMNS:
            converted = _parse_bool(text)
            if converted is None:
                analysis.error("invalid_boolean", f"invalid boolean {text!r} in {name}", row_number, column=name, value=text)
                return None
            values[name] = converted
        else:
            converted = _parse_integer(text)
            if converted is None:
                analysis.error("invalid_integer", f"invalid integer {text!r} in {name}", row_number, column=name, value=text)
                return None
            values[name] = converted

    state = values["tracking_state"]
    if state not in KNOWN_TRACKING_STATES:
        analysis.error("unknown_tracking_state", f"unknown tracking_state {state!r}", row_number, value=state)

    lock = values["target_lock"]
    if lock not in VALID_TARGET_LOCKS:
        analysis.error("invalid_target_lock", f"target_lock must be 49 or 50, got {lock!r}", row_number, value=lock)

    # target_lock/fire_command are signed protocol values.  Negative values
    # are retained for the distribution so an operator can see the anomaly.
    for name in INTEGER_COLUMNS - set(REQUIRED_COLUMNS):
        text = raw.get(name, "").strip()
        if not text:
            values[name] = None
            continue
        converted = _parse_integer(text)
        if converted is None:
            analysis.error("invalid_integer", f"invalid integer {text!r} in {name}", row_number, column=name, value=text)
            values[name] = None
        else:
            values[name] = converted

    # ``absolute_command_valid`` is optional in older CSVs, but if supplied it
    # is a bool-like value rather than an arbitrary integer.
    if "absolute_command_valid" in raw:
        text = raw["absolute_command_valid"].strip()
        if text:
            converted_bool = _parse_bool(text)
            if converted_bool is None:
                analysis.error("invalid_boolean", f"invalid boolean {text!r} in absolute_command_valid", row_number, column="absolute_command_valid", value=text)
            values["absolute_command_valid"] = converted_bool

    for name in FLOAT_COLUMNS:
        if name not in raw:
            continue
        text = raw[name].strip()
        if not text:
            values[name] = None
            continue
        converted = _finite_float(text)
        if converted is None:
            analysis.error(
                "invalid_finite_float",
                f"invalid finite float {text!r} in {name}",
                row_number,
                column=name,
                value=text,
            )
            values[name] = None
        else:
            values[name] = converted

    # Preserve non-numeric optional fields as trimmed strings.  This also
    # lets the report count an empty or previously unseen PnP failure reason.
    for name in headers:
        if name not in values:
            text = raw[name].strip()
            values[name] = text if text else None

    return ParsedRecord(row_number=row_number, values=values, raw=raw)


def _open_csv(source: Union[str, Path, TextIO]) -> tuple[TextIO, bool, str]:
    if hasattr(source, "read"):
        stream = source  # type: ignore[assignment]
        return stream, False, _as_name(getattr(stream, "name", ""))
    path = Path(source)
    # newline="" is required by csv.reader to handle quoted newlines correctly.
    return path.open("r", encoding="utf-8-sig", newline=""), True, _as_name(path)


def analyze_csv(
    source: Union[str, Path, TextIO],
    metadata: Optional[Mapping[str, Any]] = None,
) -> Analysis:
    """Parse and validate a CSV source.

    The returned object intentionally contains both usable rows and all
    diagnostics.  It is safe to call with an absent/unreadable path; that
    condition is represented as a report error rather than raised to the CLI.
    """

    analysis = Analysis(input_name=_as_name(source), metadata=dict(metadata or {}))
    try:
        stream, should_close, detected_name = _open_csv(source)
    except (OSError, TypeError, ValueError) as exc:
        analysis.error("input_open_error", f"unable to open CSV: {exc}")
        return analysis
    if detected_name:
        analysis.input_name = detected_name

    try:
        reader = csv.reader(stream, strict=True)
        try:
            header_row = next(reader)
        except StopIteration:
            analysis.error("empty_csv", "CSV contains no header row")
            return analysis
        except csv.Error as exc:
            analysis.error("csv_parse_error", f"unable to read CSV header: {exc}")
            return analysis

        headers = [str(name).strip().lstrip("\ufeff") for name in header_row]
        analysis.headers = headers
        duplicate_names = sorted({name for name in headers if name and headers.count(name) > 1})
        if duplicate_names:
            analysis.error("duplicate_columns", "duplicate CSV column names", columns=duplicate_names)
        missing = [name for name in REQUIRED_COLUMNS if name not in headers]
        if missing:
            analysis.error("missing_required_columns", "required CSV columns are missing", columns=missing)
        if not headers or all(not name for name in headers):
            analysis.error("empty_header", "CSV header has no named columns")

        # DictReader cannot represent duplicate names safely, so map by index
        # ourselves while still preserving the first value under a duplicate.
        row_count = 0
        try:
            for row_number, row_values in enumerate(reader, start=2):
                row_count += 1
                analysis.data_row_count += 1
                if len(row_values) != len(headers):
                    analysis.error(
                        "row_length_mismatch",
                        f"row has {len(row_values)} fields but header has {len(headers)}",
                        row_number,
                        expected=len(headers),
                        actual=len(row_values),
                    )
                # Pad short rows and ignore surplus fields for best-effort
                # parsing; the mismatch remains visible in diagnostics.
                normalized = list(row_values[: len(headers)]) + [""] * max(0, len(headers) - len(row_values))
                row: dict[str, str] = {}
                for index, name in enumerate(headers):
                    if name and name not in row:
                        row[name] = normalized[index]
                if not missing and not duplicate_names and len(row_values) == len(headers):
                    record = _convert_row(analysis, row_number, row, headers)
                    if record is not None:
                        analysis.records.append(record)
                elif not missing and not duplicate_names:
                    # A malformed row can still be useful if all required
                    # values are present; parse it after padding.
                    record = _convert_row(analysis, row_number, row, headers)
                    if record is not None:
                        analysis.records.append(record)
        except csv.Error as exc:
            analysis.error("csv_parse_error", f"unable to parse CSV row: {exc}")
        if row_count == 0:
            analysis.error("empty_data", "CSV has a header but no data rows")
    finally:
        if should_close:
            stream.close()

    _validate_sequence_and_frames(analysis)
    return analysis


def _ordered_groups(records: Iterable[ParsedRecord]) -> OrderedDict[int, list[ParsedRecord]]:
    groups: OrderedDict[int, list[ParsedRecord]] = OrderedDict()
    for record in records:
        frame = record.values.get("frame")
        if isinstance(frame, int):
            groups.setdefault(frame, []).append(record)
    return groups


def _validate_sequence_and_frames(analysis: Analysis) -> None:
    groups = _ordered_groups(analysis.records)
    if not groups:
        return
    frames = list(groups)

    # Frame rollback is checked in row input order. Repeated frame numbers are
    # expected because a frame can have one row per detection/track.
    row_frames = [row.values.get("frame") for row in analysis.records if isinstance(row.values.get("frame"), int)]
    for previous, current in zip(row_frames, row_frames[1:]):
        if current < previous:
            analysis.error("frame_rollback", f"frame number rolled back from {previous} to {current}", value=current)

    # Missing frame IDs are useful evidence of dropped/filtered frames but are
    # a warning rather than a malformed CSV.  Keep the complete list bounded
    # for pathological input while retaining the total count.
    low, high = min(frames), max(frames)
    present = set(frames)
    span = high - low + 1
    if span <= 1_000_000:
        missing = [frame for frame in range(low, high + 1) if frame not in present]
    else:
        # Avoid allocating an attacker-controlled multi-billion-item list.
        missing = []
        analysis.warning("frame_span_too_large", "frame span is too large to enumerate missing IDs", span=span)
    if missing:
        analysis.warning("missing_frames", "one or more frame numbers are absent", count=len(missing), frames=missing[:100])

    # Timestamps are frame-level values.  Duplicate rows of one frame repeat
    # its timestamp by design; a repeated timestamp on another frame is an
    # anomaly.
    frame_stamps: list[tuple[int, int]] = []
    for frame, rows in groups.items():
        stamps = [row.values.get("stamp_ns") for row in rows if isinstance(row.values.get("stamp_ns"), int)]
        if not stamps:
            continue
        first = stamps[0]
        frame_stamps.append((frame, first))
        if len(set(stamps)) > 1:
            analysis.error("frame_timestamp_conflict", f"frame {frame} has inconsistent stamp_ns values", value=frame)
    seen_stamps: dict[int, int] = {}
    for frame, stamp in frame_stamps:
        if stamp in seen_stamps and seen_stamps[stamp] != frame:
            analysis.error("timestamp_repeat", f"stamp_ns {stamp} is used by multiple frames", value=stamp)
        seen_stamps[stamp] = frame
    for (frame_a, stamp_a), (frame_b, stamp_b) in zip(frame_stamps, frame_stamps[1:]):
        if stamp_b < stamp_a:
            analysis.error("timestamp_rollback", f"stamp_ns rolled back from {stamp_a} to {stamp_b}", value=stamp_b, frame=frame_b)

    # Fields emitted once per frame by auto_aim_offline must not disagree
    # across rows.  selected/state can legitimately differ across detections,
    # therefore they are aggregated below instead of being rejected here.
    frame_consistent_columns = (
        "stamp_ns",
        "detection_count",
        "valid_pnp_count",
        "target_lock",
        "fire_command",
        "test_only",
        "absolute_command_valid",
    )
    for frame, rows in groups.items():
        for column in frame_consistent_columns:
            values = [row.values.get(column) for row in rows if row.values.get(column) is not None]
            if len(set(values)) > 1:
                analysis.error(
                    "frame_field_conflict",
                    f"frame {frame} has conflicting {column} values",
                    value=frame,
                    column=column,
                )
        detection_count = next((row.values.get("detection_count") for row in rows if row.values.get("detection_count") is not None), None)
        if isinstance(detection_count, int):
            # A no-detection frame is emitted as one diagnostic row even
            # though detection_count is zero; otherwise one row represents one
            # detection pose.
            expected_rows = 1 if detection_count == 0 else detection_count
            if len(rows) != expected_rows:
                analysis.error(
                    "detection_count_conflict",
                    f"frame {frame} has detection_count={detection_count} but {len(rows)} CSV rows",
                    value=frame,
                )
        valid_pnp_count = next((row.values.get("valid_pnp_count") for row in rows if row.values.get("valid_pnp_count") is not None), None)
        if isinstance(valid_pnp_count, int):
            observed_valid_pnp = sum(1 for row in rows if row.values.get("pnp_valid") == 1)
            if observed_valid_pnp != valid_pnp_count:
                analysis.error(
                    "valid_pnp_count_conflict",
                    f"frame {frame} has valid_pnp_count={valid_pnp_count} but {observed_valid_pnp} pnp_valid rows",
                    value=frame,
                )
        states = {row.values.get("tracking_state") for row in rows if row.values.get("tracking_state") is not None}
        if len(states) > 1:
            # Multiple states can represent multiple tracks, but it is still
            # worth recording as a data-quality warning for human review.
            analysis.warning("frame_state_multiplicity", f"frame {frame} contains multiple tracking states", value=frame, states=sorted(states))
        lock_values = {row.values.get("target_lock") for row in rows if row.values.get("target_lock") is not None}
        if 49 in lock_values and "tracking" not in states:
            analysis.error(
                "state_lock_conflict",
                f"frame {frame} reports target_lock=49 without tracking state",
                value=frame,
            )
        for row in rows:
            pnp_valid = row.values.get("pnp_valid")
            if pnp_valid is not None and pnp_valid not in (0, 1):
                analysis.error(
                    "invalid_pnp_valid",
                    f"pnp_valid must be 0 or 1, got {pnp_valid!r}",
                    row.row_number,
                    value=pnp_valid,
                )


def _counter(values: Iterable[Any], *, stringify: bool = True) -> dict[str, int]:
    counter: Counter[str] = Counter()
    for value in values:
        if value is None:
            continue
        key = str(value) if stringify else value
        counter[key] += 1
    return dict(sorted(counter.items(), key=lambda item: item[0]))


def _numeric_summary(values: Iterable[Any]) -> dict[str, Any]:
    finite = [float(value) for value in values if isinstance(value, (int, float)) and math.isfinite(float(value))]
    if not finite:
        return {"count": 0, "min": None, "mean": None, "median": None, "max": None}
    return {
        "count": len(finite),
        "min": min(finite),
        "mean": statistics.fmean(finite),
        "median": statistics.median(finite),
        "max": max(finite),
    }


def _distribution_with_defaults(values: Iterable[Any], defaults: Sequence[Any]) -> dict[str, int]:
    result = _counter(values)
    for value in defaults:
        result.setdefault(str(value), 0)
    return dict(sorted(result.items(), key=lambda item: item[0]))


def _frame_summaries(groups: OrderedDict[int, list[ParsedRecord]]) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for frame, rows in groups.items():
        def first_value(column: str) -> Any:
            for row in rows:
                value = row.values.get(column)
                if value is not None:
                    return value
            return None

        selected_rows = [row for row in rows if row.values.get("selected") is True]
        # Prefer selected track ID, then any non-empty track ID.  The latter
        # keeps target-switch evidence useful even when selected was omitted.
        track_id = next((row.values.get("track_id") for row in selected_rows if row.values.get("track_id") is not None), None)
        if track_id is None:
            track_id = next((row.values.get("track_id") for row in rows if row.values.get("track_id") is not None), None)
        states = [row.values.get("tracking_state") for row in rows if row.values.get("tracking_state") is not None]
        selected_state = next((row.values.get("tracking_state") for row in selected_rows if row.values.get("tracking_state") is not None), None)
        state = selected_state or (states[0] if states else None)
        summaries.append(
            {
                "frame": frame,
                "stamp_ns": first_value("stamp_ns"),
                "detection_count": first_value("detection_count"),
                "valid_pnp_count": first_value("valid_pnp_count"),
                "tracking_state": state,
                "tracking_states": sorted(set(states)),
                "selected": bool(selected_rows),
                "selected_rows": len(selected_rows),
                "target_lock": first_value("target_lock"),
                "fire_command": first_value("fire_command"),
                "test_only": first_value("test_only"),
                "track_id": track_id,
                "rows": len(rows),
            }
        )
    return summaries


def _analyze_from_parsed(analysis: Analysis) -> dict[str, Any]:
    groups = _ordered_groups(analysis.records)
    frames = _frame_summaries(groups)
    unique_frames = [item["frame"] for item in frames]
    stamps = [item["stamp_ns"] for item in frames if isinstance(item["stamp_ns"], int)]
    intervals = [b - a for a, b in zip(stamps, stamps[1:])]
    duplicate_rows = sum(max(0, len(rows) - 1) for rows in groups.values())
    missing_frames: list[int] = []
    if unique_frames:
        present = set(unique_frames)
        low, high = min(unique_frames), max(unique_frames)
        if high - low + 1 <= 1_000_000:
            missing_frames = [frame for frame in range(low, high + 1) if frame not in present]

    state_values = [item["tracking_state"] for item in frames if item["tracking_state"] is not None]
    state_distribution = _distribution_with_defaults(state_values, KNOWN_TRACKING_STATES)
    lock_values = [item["target_lock"] for item in frames if item["target_lock"] is not None]
    lock_distribution = _distribution_with_defaults(lock_values, VALID_TARGET_LOCKS)
    fire_values = [item["fire_command"] for item in frames if item["fire_command"] is not None]
    fire_distribution = _counter(fire_values)
    test_values = [item["test_only"] for item in frames if item["test_only"] is not None]
    test_distribution = _counter(test_values)

    selected_track_ids = [item["track_id"] for item in frames if item["selected"] and item["track_id"] is not None]
    switch_count = sum(1 for previous, current in zip(selected_track_ids, selected_track_ids[1:]) if current != previous)
    lock_loss_count = sum(
        1
        for previous, current in zip(lock_values, lock_values[1:])
        if previous == 49 and current != 49
    )

    detection_values = [item["detection_count"] for item in frames if isinstance(item["detection_count"], int)]
    valid_pnp_values = [item["valid_pnp_count"] for item in frames if isinstance(item["valid_pnp_count"], int)]

    # pnp_valid and failure are row-level evidence (one row per detection).
    pnp_valid_values = [row.values.get("pnp_valid") for row in analysis.records]
    pnp_failure_values = [row.values.get("pnp_failure") for row in analysis.records]
    reprojection = [
        row.values.get("reprojection_error_px")
        for row in analysis.records
        if row.values.get("pnp_valid") == 1
    ]
    reprojection_summary = _numeric_summary(reprojection)

    coverage: dict[str, Any] = {
        "csv_total_rows": analysis.data_row_count,
        "total_rows": analysis.data_row_count,
        "csv_rows": analysis.data_row_count,
        "parsed_rows": len(analysis.records),
        "unique_frame_count": len(frames),
        "unique_frames": len(frames),
        "first_frame": unique_frames[0] if unique_frames else None,
        "last_frame": unique_frames[-1] if unique_frames else None,
        "first_timestamp_ns": stamps[0] if stamps else None,
        "last_timestamp_ns": stamps[-1] if stamps else None,
        "coverage_duration_ns": (stamps[-1] - stamps[0]) if len(stamps) >= 2 else 0 if stamps else None,
        "coverage_duration_s": ((stamps[-1] - stamps[0]) / 1_000_000_000) if len(stamps) >= 2 else 0.0 if stamps else None,
        "missing_frames": missing_frames,
        "missing_frame_count": len(missing_frames),
        "duplicate_frame_rows": duplicate_rows,
        "duplicate_frame_row_count": duplicate_rows,
        "timestamp_strictly_increasing": all(b > a for a, b in zip(stamps, stamps[1:])),
        "timestamp_anomaly_count": sum(1 for item in analysis.errors if item["code"] in {"timestamp_repeat", "timestamp_rollback", "frame_timestamp_conflict"}),
        "frame_interval_ns": _numeric_summary(intervals),
        "adjacent_frame_interval_ns": _numeric_summary(intervals),
    }
    interval_summary = coverage["adjacent_frame_interval_ns"]
    coverage["adjacent_frame_interval_min_ns"] = interval_summary["min"]
    coverage["adjacent_frame_interval_median_ns"] = interval_summary["median"]
    coverage["adjacent_frame_interval_max_ns"] = interval_summary["max"]
    detection_stats = _numeric_summary(detection_values)
    detection_stats["total"] = sum(detection_values) if detection_values else 0
    valid_pnp_stats = _numeric_summary(valid_pnp_values)
    valid_pnp_stats["total"] = sum(valid_pnp_values) if valid_pnp_values else 0

    tracker: dict[str, Any] = {
        "tracking_state_distribution": state_distribution,
        "tracking_state_frame_counts": state_distribution.copy(),
        "lost_frames": state_distribution.get("lost", 0),
        "detecting_frames": state_distribution.get("detecting", 0),
        "tracking_frames": state_distribution.get("tracking", 0),
        "temp_lost_frames": state_distribution.get("temp_lost", 0),
        "target_lock_frame_counts": lock_distribution,
        "target_lock_49_frames": lock_distribution.get("49", 0),
        "target_lock_50_frames": lock_distribution.get("50", 0),
        "selected_frames": sum(1 for item in frames if item["selected"]),
        "selected_rows": sum(item["selected_rows"] for item in frames),
        "track_id_count": len({row.values.get("track_id") for row in analysis.records if row.values.get("track_id") is not None}),
        "target_switch_count": switch_count,
        "lock_loss_count": lock_loss_count,
        "state_conflict_count": sum(1 for item in analysis.warnings if item["code"] == "frame_state_multiplicity"),
        "data_hole_count": coverage["missing_frame_count"],
    }

    safety_anomalies: list[str] = []
    nonzero_fire = {value for value in fire_values if isinstance(value, int) and value != 0}
    if nonzero_fire:
        safety_anomalies.append("nonzero_fire_command")
    false_test_frames = [item["frame"] for item in frames if item["test_only"] is False]
    if false_test_frames:
        safety_anomalies.append("test_only_false")
    safety: dict[str, Any] = {
        "fire_command_distribution": fire_distribution,
        "nonzero_fire_command_values": sorted(nonzero_fire),
        "nonzero_fire_command_count": sum(1 for value in fire_values if isinstance(value, int) and value != 0),
        "test_only_distribution": test_distribution,
        "test_only_false_frames": false_test_frames,
        "input_test_only": bool(test_values) and all(value is True for value in test_values),
        "dry_run": True,
        "serial_enabled": False,
        "allow_fire": False,
        "action_triggered": False,
        "anomalies": safety_anomalies,
        "software_output_lock_is_not_hardware_lock": True,
    }

    return {
        "coverage": coverage,
        "detection_pnp": {
            "detection_count": detection_stats,
            "valid_pnp_count": valid_pnp_stats,
            "pnp_valid_distribution": _counter(pnp_valid_values),
            "pnp_failure_distribution": _counter(pnp_failure_values),
            "valid_reprojection_error_px": reprojection_summary,
            "reprojection_error_px": reprojection_summary,
            "real_distance_accuracy_computed": False,
            "hit_rate_computed": False,
        },
        "tracker_target": tracker,
        "safety": safety,
        "_frames": frames,
    }


def _sanitize_metadata(metadata: Optional[Mapping[str, Any]]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    allowed = {"commit", "model_profile_id", "model_profile_version", "pnp_profile", "dataset_id", "source_label"}
    result: dict[str, Any] = {}
    warnings: list[dict[str, Any]] = []
    if not metadata:
        return result, warnings
    for key, value in metadata.items():
        if key not in allowed:
            warnings.append({"code": "metadata_field_ignored", "message": f"metadata field {key!r} is not in the allow-list", "field": key})
            continue
        if isinstance(value, (str, int, float, bool)) or value is None:
            if isinstance(value, str):
                text = value.strip()
                is_windows_absolute = len(text) >= 3 and text[1] == ":" and text[2] in {"/", "\\"}
                is_absolute = text.startswith(("/", "\\")) or is_windows_absolute
                if is_absolute:
                    result[key] = _as_name(text)
                    warnings.append({"code": "metadata_path_redacted", "message": f"metadata path in {key!r} was reduced to its basename", "field": key})
                else:
                    result[key] = value
            else:
                result[key] = value
        else:
            result[key] = str(value)
    return result, warnings


def build_report(analysis: Analysis, metadata: Optional[Mapping[str, Any]] = None) -> dict[str, Any]:
    """Build a JSON-compatible report from :class:`Analysis`."""

    safe_metadata, metadata_warnings = _sanitize_metadata(metadata if metadata is not None else analysis.metadata)
    for warning in metadata_warnings:
        if warning not in analysis.warnings:
            analysis.warnings.append(warning)
    sections = _analyze_from_parsed(analysis)
    # Safety violations and structural/data validation errors are FAIL.  Frame
    # gaps alone are WARN because a dropped frame can be an expected recording
    # condition worth reviewing rather than a parser failure.
    fatal_errors = list(analysis.errors)
    for anomaly in sections["safety"]["anomalies"]:
        fatal_errors.append({"code": anomaly, "message": anomaly.replace("_", " ")})
    status = "FAIL" if fatal_errors else ("WARN" if analysis.warnings else "PASS")
    sections.pop("_frames", None)

    units = {
        "external_position_angle": "degree",
        "external_angular_velocity": "degree/s",
        "external_angular_acceleration": "degree/s²",
        "internal_position_angle": "rad",
        "internal_angular_velocity": "rad/s",
        "internal_angular_acceleration": "rad/s²",
        "algorithm_internal_position_angle": "rad",
        "algorithm_internal_angular_velocity": "rad/s",
        "algorithm_internal_angular_acceleration": "rad/s²",
        "fire_command_baseline": 0,
    }
    evidence_boundary = {
        "software_structure_statistics_only": True,
        "real_hit_rate_computed": False,
        "hardware_validation": False,
        "gimbal_closed_loop_validated": False,
        "firing_validated": False,
    }
    report: dict[str, Any] = {
        "status": status,
        "report_status": status,
        "input": {
            "name": analysis.input_name,
            "columns": analysis.headers,
            "required_columns": list(REQUIRED_COLUMNS),
            "metadata": safe_metadata,
        },
        "errors": fatal_errors,
        "warnings": analysis.warnings,
        "validation": {
            "error_count": len(fatal_errors),
            "warning_count": len(analysis.warnings),
            "required_columns_present": all(name in analysis.headers for name in REQUIRED_COLUMNS),
            "duplicate_columns": [item.get("columns", []) for item in analysis.errors if item["code"] == "duplicate_columns"],
        },
        **sections,
        "units": units,
        "evidence_boundary": evidence_boundary,
        "metadata": safe_metadata,
    }
    return report


def markdown_report(report: Mapping[str, Any]) -> str:
    """Render a concise, deterministic Markdown report."""

    status = report.get("status", "FAIL")
    coverage = report.get("coverage", {})
    dp = report.get("detection_pnp", {})
    tracker = report.get("tracker_target", {})
    safety = report.get("safety", {})
    units = report.get("units", {})
    boundary = report.get("evidence_boundary", {})
    lines = [
        "# Offline auto-aim evidence report",
        "",
        f"**Report status: `{status}`**",
        "",
        "This report is read-only software-structure and CSV data-quality evidence. It does not connect to ROS, a camera, serial, a gimbal, or a shooter.",
        "",
        "## Data coverage",
        "",
        f"- CSV rows parsed: {coverage.get('csv_total_rows', 0)}",
        f"- Unique frames: {coverage.get('unique_frame_count', 0)} ({coverage.get('first_frame')} → {coverage.get('last_frame')})",
        f"- Timestamps: {coverage.get('first_timestamp_ns')} → {coverage.get('last_timestamp_ns')} ns",
        f"- Coverage duration: {coverage.get('coverage_duration_ns')} ns ({coverage.get('coverage_duration_s')} s)",
        f"- Missing frames: {coverage.get('missing_frame_count', 0)}",
        f"- Duplicate frame rows: {coverage.get('duplicate_frame_rows', 0)}",
        f"- Timestamps strictly increasing: {coverage.get('timestamp_strictly_increasing')}",
        f"- Timestamp anomalies: {coverage.get('timestamp_anomaly_count', 0)}",
        f"- Adjacent frame interval (ns): {coverage.get('adjacent_frame_interval_ns')}",
        "",
        "## Detection and PnP",
        "",
        f"- detection_count: {dp.get('detection_count')}",
        f"- valid_pnp_count: {dp.get('valid_pnp_count')}",
        f"- pnp_valid distribution: {dp.get('pnp_valid_distribution')}",
        f"- PnP failure distribution: {dp.get('pnp_failure_distribution')}",
        f"- Valid reprojection error (px): {dp.get('valid_reprojection_error_px')}",
        "- Real distance accuracy and hit rate are not computed.",
        "",
        "## Tracker and target",
        "",
        f"- tracking_state distribution: {tracker.get('tracking_state_distribution')}",
        f"- target_lock 49/50 frames: {tracker.get('target_lock_49_frames', 0)} / {tracker.get('target_lock_50_frames', 0)}",
        f"- selected frames / rows: {tracker.get('selected_frames', 0)} / {tracker.get('selected_rows', 0)}",
        f"- Distinct track IDs: {tracker.get('track_id_count', 0)}",
        f"- Target switches: {tracker.get('target_switch_count', 0)}; lock losses: {tracker.get('lock_loss_count', 0)}",
        f"- State conflicts / data holes: {tracker.get('state_conflict_count', 0)} / {tracker.get('data_hole_count', 0)}",
        "",
        "## Safety",
        "",
        f"- fire_command distribution: {safety.get('fire_command_distribution')}",
        f"- Non-zero fire commands: {safety.get('nonzero_fire_command_values')}",
        f"- test_only=false frames: {safety.get('test_only_false_frames')}",
        "- No action is triggered; fire_command is reported only.",
        "- A software lock value is not interpreted as a real gimbal lock.",
        "",
        "## Units",
        "",
        f"- External position angle: {units.get('external_position_angle')}",
        f"- External angular velocity: {units.get('external_angular_velocity')}",
        f"- External angular acceleration: {units.get('external_angular_acceleration')}",
        f"- Algorithm internal position angle: {units.get('algorithm_internal_position_angle')}",
        f"- Algorithm internal angular velocity: {units.get('algorithm_internal_angular_velocity')}",
        f"- Algorithm internal angular acceleration: {units.get('algorithm_internal_angular_acceleration')}",
        f"- fire_command baseline: {units.get('fire_command_baseline')}",
        f"- 外部位置角：{units.get('external_position_angle')}",
        f"- 外部角速度：{units.get('external_angular_velocity')}",
        f"- 外部角加速度：{units.get('external_angular_acceleration')}",
        f"- 算法内部位置角：{units.get('algorithm_internal_position_angle')}",
        f"- 算法内部角速度：{units.get('algorithm_internal_angular_velocity')}",
        f"- 算法内部角加速度：{units.get('algorithm_internal_angular_acceleration')}",
        f"- `fire_command` 基线：{units.get('fire_command_baseline')}",
        "",
        "## Evidence boundary",
        "",
    ]
    for key, value in boundary.items():
        lines.append(f"- `{key}`: {str(value).lower() if isinstance(value, bool) else value}")
    lines.extend(
        [
            "",
            "```yaml",
            "software_structure_statistics_only: true",
            "real_hit_rate_computed: false",
            "hardware_validation: false",
            "gimbal_closed_loop_validated: false",
            "firing_validated: false",
            "```",
            "",
            "The report is not a real hit-rate, competition-score, hardware, gimbal-closed-loop, or firing validation result.",
            "",
            "## Validation diagnostics",
            "",
            f"- Errors: {len(report.get('errors', []))}",
            f"- Warnings: {len(report.get('warnings', []))}",
            "",
            "```json",
            json.dumps({"errors": report.get("errors", []), "warnings": report.get("warnings", [])}, ensure_ascii=False, indent=2),
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def write_reports(
    report: Mapping[str, Any],
    json_path: Optional[Union[str, Path]] = None,
    markdown_path: Optional[Union[str, Path]] = None,
) -> None:
    """Write JSON/Markdown reports, creating parent directories if needed."""

    if json_path:
        path = Path(json_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if markdown_path:
        path = Path(markdown_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(markdown_report(report), encoding="utf-8")


def _load_metadata(path: Optional[Union[str, Path]]) -> tuple[dict[str, Any], Optional[str]]:
    if not path:
        return {}, None
    try:
        value = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return {}, f"unable to read metadata JSON: {exc}"
    if not isinstance(value, dict):
        return {}, "metadata JSON must contain an object"
    return value, None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate a read-only offline auto-aim CSV evidence report")
    parser.add_argument("--input-csv", "-i", required=True, help="auto_aim_offline CSV input")
    parser.add_argument("--json-report", "-j", help="path for JSON report")
    parser.add_argument("--markdown-report", "-m", help="path for Markdown report")
    parser.add_argument("--metadata-json", help="optional allow-listed run metadata JSON")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    metadata, metadata_error = _load_metadata(args.metadata_json)
    analysis = analyze_csv(args.input_csv, metadata)
    if metadata_error:
        analysis.error("metadata_error", metadata_error)
    report = build_report(analysis)
    try:
        write_reports(report, args.json_report, args.markdown_report)
    except OSError as exc:
        # The input diagnostics are still useful, but a failed requested
        # output is a report failure and should produce the FAIL exit status.
        report.setdefault("errors", []).append({"code": "report_write_error", "message": str(exc)})
        report["status"] = report["report_status"] = "FAIL"
        if args.json_report:
            try:
                Path(args.json_report).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            except OSError:
                pass
    if not args.json_report and not args.markdown_report:
        sys.stdout.write(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    else:
        sys.stdout.write(f"status={report['status']}\n")
    return {"PASS": 0, "WARN": 2, "FAIL": 1}.get(str(report.get("status")), 1)


# A small compatibility alias for callers that use the verb ``generate``.
generate_report = build_report
parse_csv = analyze_csv
render_markdown = markdown_report


if __name__ == "__main__":
    raise SystemExit(main())
