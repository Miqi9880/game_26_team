"""Pure input validation and report generation for the ROS preflight node."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from enum import IntEnum
import json
import math
import re
import time
from typing import Any, Dict, Iterable, List, Optional, Tuple


IMAGE_TOPIC = "/image_raw"
CAMERA_INFO_TOPIC = "/camera_info"
VISION_TOPIC = "/Vision_data"
INPUT_TOPICS = (IMAGE_TOPIC, CAMERA_INFO_TOPIC, VISION_TOPIC)
TIMED_TOPICS = (IMAGE_TOPIC, VISION_TOPIC)

PITCH_PROFILES = {
    "new_turtle": (-20.0, 19.0),
    "dog_leg": (-10.0, 31.0),
}


class Status(IntEnum):
    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass
class Finding:
    status: str
    check: str
    topic: str
    reason: str
    details: Dict[str, Any] = field(default_factory=dict)


@dataclass
class TopicStats:
    count: int = 0
    first_arrival_s: Optional[float] = None
    last_arrival_s: Optional[float] = None
    last_stamp_ns: Optional[int] = None
    timestamp_rollbacks: int = 0
    duplicate_timestamps: int = 0
    invalid_timestamps: int = 0
    unset_timestamps: int = 0

    def observe_arrival(self, arrival_s: float) -> None:
        self.count += 1
        if self.first_arrival_s is None:
            self.first_arrival_s = arrival_s
        self.last_arrival_s = arrival_s


def _status_name(status: Status) -> str:
    return status.name


def _all_finite(values: Iterable[Any]) -> bool:
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError, OverflowError):
        return False


def _stamp_ns(message: Any) -> Tuple[Optional[int], Optional[str]]:
    try:
        stamp = message.header.stamp
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        return None, f"Header timestamp is not readable: {error}"
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return None, "Header timestamp is outside canonical sec/nanosec bounds"
    return sec * 1_000_000_000 + nanosec, None


_FIXED_ENCODINGS = {
    "mono8": 1,
    "mono16": 2,
    "bgr8": 3,
    "rgb8": 3,
    "bgra8": 4,
    "rgba8": 4,
    "bayer_rggb8": 1,
    "bayer_bggr8": 1,
    "bayer_gbrg8": 1,
    "bayer_grbg8": 1,
    "bayer_rggb16": 2,
    "bayer_bggr16": 2,
    "bayer_gbrg16": 2,
    "bayer_grbg16": 2,
    "yuv422": 2,
    "yuyv": 2,
    "uyvy": 2,
}
_TYPE_ENCODING = re.compile(r"^(8|16|32|64)(?:U|S|F)C([1-9][0-9]*)$")


def bytes_per_pixel(encoding: str) -> Optional[int]:
    fixed = _FIXED_ENCODINGS.get(encoding.lower())
    if fixed is not None:
        return fixed
    match = _TYPE_ENCODING.fullmatch(encoding.upper())
    if match is None:
        return None
    return int(match.group(1)) // 8 * int(match.group(2))


class PreflightAnalyzer:
    """Accumulates observations without mutating or interpreting input messages."""

    def __init__(
        self,
        timeout_s: float = 1.0,
        vehicle_profile: str = "unselected",
        shared_clock_domain: bool = False,
        sync_tolerance_ms: float = 50.0,
        start_s: Optional[float] = None,
    ) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        if vehicle_profile != "unselected" and vehicle_profile not in PITCH_PROFILES:
            raise ValueError("unknown vehicle profile")
        if not math.isfinite(sync_tolerance_ms) or sync_tolerance_ms < 0.0:
            raise ValueError("sync_tolerance_ms must be finite and non-negative")
        self.timeout_s = timeout_s
        self.vehicle_profile = vehicle_profile
        self.shared_clock_domain = shared_clock_domain
        self.sync_tolerance_ms = sync_tolerance_ms
        self.start_s = time.monotonic() if start_s is None else start_s
        self.stats = {topic: TopicStats() for topic in INPUT_TOPICS}
        self._findings: Dict[str, Finding] = {}
        self._latest_image_size: Optional[Tuple[int, int]] = None
        self._latest_camera_size: Optional[Tuple[int, int]] = None

    def _record(
        self,
        key: str,
        status: Status,
        check: str,
        topic: str,
        reason: str,
        **details: Any,
    ) -> None:
        old = self._findings.get(key)
        if old is not None and Status[old.status] > status:
            return
        self._findings[key] = Finding(
            status=_status_name(status),
            check=check,
            topic=topic,
            reason=reason,
            details=details,
        )

    def _observe_common(self, topic: str, message: Any, arrival_s: float) -> None:
        stats = self.stats[topic]
        stats.observe_arrival(arrival_s)
        if topic not in TIMED_TOPICS:
            return
        stamp_ns, stamp_error = _stamp_ns(message)
        if stamp_error is not None:
            stats.invalid_timestamps += 1
            return
        assert stamp_ns is not None
        if stamp_ns == 0:
            stats.unset_timestamps += 1
        if stats.last_stamp_ns is not None:
            if stamp_ns < stats.last_stamp_ns:
                stats.timestamp_rollbacks += 1
            elif stamp_ns == stats.last_stamp_ns:
                stats.duplicate_timestamps += 1
        stats.last_stamp_ns = stamp_ns

    def observe_image(self, message: Any, arrival_s: Optional[float] = None) -> None:
        arrival = time.monotonic() if arrival_s is None else arrival_s
        self._observe_common(IMAGE_TOPIC, message, arrival)
        try:
            width = int(message.width)
            height = int(message.height)
            step = int(message.step)
            encoding = str(message.encoding)
            data_length = len(message.data)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self._record(
                "image.readable", Status.FAIL, "image.message_format", IMAGE_TOPIC,
                f"Image fields are not readable: {error}",
            )
            return

        if width <= 0 or height <= 0:
            self._record(
                "image.dimensions", Status.FAIL, "image.dimensions", IMAGE_TOPIC,
                "Image width and height must both be positive",
                width=width, height=height,
            )
        else:
            self._record(
                "image.dimensions", Status.PASS, "image.dimensions", IMAGE_TOPIC,
                "Image dimensions are positive", width=width, height=height,
            )
            self._latest_image_size = (width, height)

        pixel_bytes = bytes_per_pixel(encoding) if encoding else None
        if not encoding:
            self._record(
                "image.encoding", Status.FAIL, "image.encoding", IMAGE_TOPIC,
                "Image encoding is empty",
            )
        elif pixel_bytes is None:
            self._record(
                "image.encoding", Status.WARN, "image.encoding", IMAGE_TOPIC,
                (
                    "Encoding is not in the preflight byte-width table; "
                    "row width cannot be fully checked"
                ),
                encoding=encoding,
            )
        else:
            self._record(
                "image.encoding", Status.PASS, "image.encoding", IMAGE_TOPIC,
                "Encoding has a known byte width",
                encoding=encoding, bytes_per_pixel=pixel_bytes,
            )

        if step <= 0:
            self._record(
                "image.step", Status.FAIL, "image.step", IMAGE_TOPIC,
                "Image step must be positive", step=step,
            )
        elif pixel_bytes is not None and width > 0 and step < width * pixel_bytes:
            self._record(
                "image.step", Status.FAIL, "image.step", IMAGE_TOPIC,
                "Image step is smaller than width times bytes per pixel",
                step=step, minimum_step=width * pixel_bytes,
            )
        else:
            details = {"step": step}
            if pixel_bytes is not None and width > 0:
                details["minimum_step"] = width * pixel_bytes
            self._record(
                "image.step", Status.PASS, "image.step", IMAGE_TOPIC,
                "Image step is structurally valid", **details,
            )

        expected_length = step * height if step >= 0 and height >= 0 else -1
        if expected_length < 0 or data_length != expected_length:
            self._record(
                "image.data_length", Status.FAIL, "image.data_length", IMAGE_TOPIC,
                "Image data length must equal step times height",
                data_length=data_length, expected_length=expected_length,
            )
        elif data_length == 0:
            self._record(
                "image.data_length", Status.FAIL, "image.data_length", IMAGE_TOPIC,
                "Image data is empty", data_length=0,
            )
        else:
            self._record(
                "image.data_length", Status.PASS, "image.data_length", IMAGE_TOPIC,
                "Image data length matches step times height",
                data_length=data_length, expected_length=expected_length,
            )

    def observe_camera_info(self, message: Any, arrival_s: Optional[float] = None) -> None:
        arrival = time.monotonic() if arrival_s is None else arrival_s
        self._observe_common(CAMERA_INFO_TOPIC, message, arrival)
        try:
            width = int(message.width)
            height = int(message.height)
            k_values = list(message.k)
            d_values = list(message.d)
            model = str(message.distortion_model)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self._record(
                "camera.readable", Status.FAIL, "camera_info.message_format",
                CAMERA_INFO_TOPIC, f"CameraInfo fields are not readable: {error}",
            )
            return

        if width <= 0 or height <= 0:
            self._record(
                "camera.dimensions", Status.FAIL, "camera_info.dimensions",
                CAMERA_INFO_TOPIC, "CameraInfo width and height must both be positive",
                width=width, height=height,
            )
        else:
            self._record(
                "camera.dimensions", Status.PASS, "camera_info.dimensions",
                CAMERA_INFO_TOPIC, "CameraInfo dimensions are positive",
                width=width, height=height,
            )
            self._latest_camera_size = (width, height)

        k_valid = len(k_values) == 9 and _all_finite(k_values)
        if k_valid:
            k_valid = k_values[0] > 0.0 and k_values[4] > 0.0 and abs(k_values[8]) > 1e-12
        self._record(
            "camera.k", Status.PASS if k_valid else Status.FAIL, "camera_info.K",
            CAMERA_INFO_TOPIC,
            "K has 9 finite entries with positive fx/fy and non-zero K[8]"
            if k_valid else
            "K must have 9 finite entries, positive fx/fy, and non-zero K[8]",
            length=len(k_values),
        )

        expected_d_lengths = {
            "plumb_bob": {5},
            "rational_polynomial": {8},
            "equidistant": {4},
        }
        if not _all_finite(d_values):
            d_status = Status.FAIL
            d_reason = "D contains a non-finite or non-numeric value"
        elif model in expected_d_lengths and len(d_values) not in expected_d_lengths[model]:
            d_status = Status.FAIL
            d_reason = f"D length does not match distortion model {model}"
        elif model in expected_d_lengths:
            d_status = Status.PASS
            d_reason = "D length and finite values match the declared distortion model"
        elif not model:
            d_status = Status.FAIL
            d_reason = "distortion_model is empty, so D format cannot be established"
        else:
            d_status = Status.WARN
            d_reason = "Unknown distortion_model; D is finite but its length cannot be validated"
        self._record(
            "camera.d", d_status, "camera_info.D", CAMERA_INFO_TOPIC, d_reason,
            distortion_model=model, length=len(d_values),
        )

    def observe_vision(self, message: Any, arrival_s: Optional[float] = None) -> None:
        arrival = time.monotonic() if arrival_s is None else arrival_s
        self._observe_common(VISION_TOPIC, message, arrival)

        field_units = {
            "yaw": "degree",
            "pitch": "degree",
            "roll": "degree",
            "yaw_vel": "degree/s",
            "pitch_vel": "degree/s",
            "shoot_speed": "m/s",
        }
        values: Dict[str, float] = {}
        invalid_fields: List[str] = []
        for name in field_units:
            try:
                value = float(getattr(message, name))
            except (AttributeError, TypeError, ValueError, OverflowError):
                invalid_fields.append(name)
                continue
            values[name] = value
            if not math.isfinite(value):
                invalid_fields.append(name)
        self._record(
            "vision.scalars",
            Status.PASS if not invalid_fields else Status.FAIL,
            "vision.scalar_finite",
            VISION_TOPIC,
            (
                "Vision angle, angular velocity, and speed fields are finite; "
                "original units are unchanged"
            )
            if not invalid_fields else
            "Vision contains missing, non-numeric, NaN, or Inf scalar fields",
            units=field_units, values=values, invalid_fields=invalid_fields,
        )

        acceleration_fields = ("yaw_acc", "pitch_acc")
        missing_acceleration = [name for name in acceleration_fields if not hasattr(message, name)]
        invalid_acceleration = []
        acceleration_values: Dict[str, float] = {}
        for name in acceleration_fields:
            if name in missing_acceleration:
                continue
            try:
                value = float(getattr(message, name))
                acceleration_values[name] = value
                if not math.isfinite(value):
                    invalid_acceleration.append(name)
            except (TypeError, ValueError, OverflowError):
                invalid_acceleration.append(name)
        if invalid_acceleration:
            acc_status = Status.FAIL
            acc_reason = "Vision acceleration contains a non-finite or non-numeric value"
        elif missing_acceleration:
            acc_status = Status.WARN
            acc_reason = (
                "Installed Vision interface has no acceleration field; "
                "degree/s² check is unavailable"
            )
        else:
            acc_status = Status.PASS
            acc_reason = (
                "Vision acceleration fields are finite and reported unchanged "
                "in degree/s²"
            )
        self._record(
            "vision.acceleration", acc_status, "vision.acceleration_finite", VISION_TOPIC,
            acc_reason, units={"yaw_acc": "degree/s²", "pitch_acc": "degree/s²"},
            values=acceleration_values,
            missing_fields=missing_acceleration, invalid_fields=invalid_acceleration,
        )

        yaw = values.get("yaw")
        if yaw is not None and math.isfinite(yaw):
            yaw_valid = -180.0 <= yaw <= 180.0
            self._record(
                "vision.yaw_range", Status.PASS if yaw_valid else Status.FAIL,
                "vision.yaw_range", VISION_TOPIC,
                "yaw is within inclusive [-180 degree, 180 degree]"
                if yaw_valid else "yaw is outside inclusive [-180 degree, 180 degree]",
                yaw_degree=yaw,
            )

        pitch = values.get("pitch")
        if self.vehicle_profile == "unselected":
            self._record(
                "vision.pitch_range", Status.WARN, "vision.pitch_profile", VISION_TOPIC,
                "No vehicle profile selected: pitch range 无法判定",
                vehicle_profile="unselected",
            )
        elif pitch is not None and math.isfinite(pitch):
            minimum, maximum = PITCH_PROFILES[self.vehicle_profile]
            pitch_valid = minimum <= pitch <= maximum
            self._record(
                "vision.pitch_range", Status.PASS if pitch_valid else Status.FAIL,
                "vision.pitch_profile", VISION_TOPIC,
                "pitch is within the explicitly selected vehicle profile"
                if pitch_valid else "pitch is outside the explicitly selected vehicle profile",
                pitch_degree=pitch, minimum_degree=minimum, maximum_degree=maximum,
                vehicle_profile=self.vehicle_profile,
            )

        try:
            quaternion = list(message.quaternion)
        except (AttributeError, TypeError, ValueError) as error:
            quaternion = []
            quaternion_error = str(error)
        else:
            quaternion_error = ""
        quaternion_valid = len(quaternion) == 4 and _all_finite(quaternion)
        self._record(
            "vision.quaternion", Status.PASS if quaternion_valid else Status.FAIL,
            "vision.quaternion_format", VISION_TOPIC,
            (
                "Quaternion has 4 finite entries in declared wxyz order; "
                "IMU relative to power-on origin; no rotation or frame inference performed"
            )
            if quaternion_valid else
            "Quaternion must have exactly 4 finite entries in declared wxyz order",
            length=len(quaternion), order="wxyz", interpretation="format_only",
            error=quaternion_error,
        )

    def record_callback_error(self, topic: str, error: Exception) -> None:
        safe_topic = topic if topic in INPUT_TOPICS else "internal"
        self._record(
            f"callback.{safe_topic}", Status.FAIL, "callback.exception", safe_topic,
            f"Message validation raised {type(error).__name__}: {error}",
        )

    def _runtime_findings(self, now_s: float) -> List[Finding]:
        findings: List[Finding] = []
        for topic in INPUT_TOPICS:
            stats = self.stats[topic]
            if stats.count == 0:
                findings.append(Finding(
                    "FAIL", "topic.received", topic,
                    "No messages were received during the observation window",
                    {"count": 0},
                ))
                continue
            findings.append(Finding(
                "PASS", "topic.received", topic, "Messages were received",
                {"count": stats.count},
            ))
            if stats.count < 2 or stats.first_arrival_s == stats.last_arrival_s:
                findings.append(Finding(
                    "WARN", "topic.frequency", topic,
                    "At least two arrivals are required to calculate frequency",
                    {"count": stats.count, "hz": None},
                ))
            else:
                span = stats.last_arrival_s - stats.first_arrival_s
                hz = (stats.count - 1) / span if span > 0.0 else None
                findings.append(Finding(
                    "PASS" if hz is not None else "WARN", "topic.frequency", topic,
                    "Receive frequency calculated from local monotonic arrival times"
                    if hz is not None else "Receive frequency could not be calculated",
                    {"count": stats.count, "span_s": span, "hz": hz},
                ))

            if topic in TIMED_TOPICS:
                assert stats.last_arrival_s is not None
                age_s = max(0.0, now_s - stats.last_arrival_s)
                timed_out = age_s > self.timeout_s
                findings.append(Finding(
                    "FAIL" if timed_out else "PASS", "topic.timeout", topic,
                    "Last message exceeds the configured receive timeout"
                    if timed_out else "Last message is within the configured receive timeout",
                    {"age_s": age_s, "timeout_s": self.timeout_s},
                ))
                if stats.invalid_timestamps:
                    stamp_status = "FAIL"
                    stamp_reason = "One or more Header timestamps were not canonical"
                elif stats.timestamp_rollbacks:
                    stamp_status = "FAIL"
                    stamp_reason = "Header timestamp moved backwards"
                elif stats.unset_timestamps:
                    stamp_status = "WARN"
                    stamp_reason = "One or more Header timestamps were zero/unset"
                elif stats.duplicate_timestamps:
                    stamp_status = "WARN"
                    stamp_reason = (
                        "Header timestamps did not decrease but duplicate values "
                        "were observed"
                    )
                elif stats.count < 2:
                    stamp_status = "WARN"
                    stamp_reason = (
                        "At least two messages are required to establish timestamp "
                        "monotonicity"
                    )
                else:
                    stamp_status = "PASS"
                    stamp_reason = "Header timestamps are strictly increasing"
                findings.append(Finding(
                    stamp_status, "header.timestamp_monotonic", topic, stamp_reason,
                    {
                        "rollbacks": stats.timestamp_rollbacks,
                        "duplicates": stats.duplicate_timestamps,
                        "invalid": stats.invalid_timestamps,
                        "unset": stats.unset_timestamps,
                    },
                ))
        return findings

    def _relationship_findings(self) -> List[Finding]:
        findings: List[Finding] = []
        if self._latest_image_size is not None and self._latest_camera_size is not None:
            match = self._latest_image_size == self._latest_camera_size
            findings.append(Finding(
                "PASS" if match else "FAIL", "image_camera.dimensions", "cross_topic",
                "Image and CameraInfo dimensions match"
                if match else "Image and CameraInfo dimensions do not match",
                {"image": self._latest_image_size, "camera_info": self._latest_camera_size},
            ))

        image_stamp = self.stats[IMAGE_TOPIC].last_stamp_ns
        vision_stamp = self.stats[VISION_TOPIC].last_stamp_ns
        if not self.shared_clock_domain:
            findings.append(Finding(
                "WARN", "image_vision.clock_domain", "cross_topic",
                (
                    "Shared clock domain was not explicitly declared: 时间基准未确认; "
                    "timestamps were not compared"
                ),
                {"compared": False},
            ))
        elif image_stamp is None or vision_stamp is None:
            findings.append(Finding(
                "WARN", "image_vision.clock_domain", "cross_topic",
                "Shared clock domain was declared, but both valid timestamps are not available",
                {"compared": False},
            ))
        else:
            delta_ms = abs(image_stamp - vision_stamp) / 1_000_000.0
            within = delta_ms <= self.sync_tolerance_ms
            findings.append(Finding(
                "PASS" if within else "WARN", "image_vision.timestamp_delta", "cross_topic",
                "Timestamp delta is within the configured diagnostic tolerance"
                if within else "Timestamp delta exceeds the configured diagnostic tolerance",
                {
                    "compared": True,
                    "delta_ms": delta_ms,
                    "tolerance_ms": self.sync_tolerance_ms,
                    "synchronization_inferred": False,
                },
            ))
        return findings

    def build_report(self, now_s: Optional[float] = None) -> Dict[str, Any]:
        now = time.monotonic() if now_s is None else now_s
        findings = [
            Finding(
                "PASS", "safety.read_only", "node",
                (
                    "Preflight contract has no control publisher, serial access, "
                    "camera access, or message mutation"
                ),
                {
                    "subscriptions": list(INPUT_TOPICS),
                    "forbidden_topic": "/Robot_ctrl_data",
                },
            )
        ]
        findings.extend(self._findings.values())
        findings.extend(self._runtime_findings(now))
        findings.extend(self._relationship_findings())
        overall_value = max(Status[finding.status] for finding in findings)
        counts = {
            name: sum(item.status == name for item in findings)
            for name in Status.__members__
        }
        return {
            "schema_version": 1,
            "tool": "ros_input_preflight",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "observation_duration_s": max(0.0, now - self.start_s),
            "overall": _status_name(overall_value),
            "counts": counts,
            "configuration": {
                "timeout_s": self.timeout_s,
                "vehicle_profile": self.vehicle_profile,
                "shared_clock_domain_declared": self.shared_clock_domain,
                "sync_tolerance_ms": self.sync_tolerance_ms,
            },
            "findings": [asdict(finding) for finding in findings],
        }


def format_report_text(report: Dict[str, Any]) -> str:
    lines = [
        f"ROS 2 INPUT PREFLIGHT: {report['overall']}",
        (
            "configuration: "
            f"timeout={report['configuration']['timeout_s']}s, "
            f"vehicle_profile={report['configuration']['vehicle_profile']}, "
            "shared_clock_domain="
            f"{report['configuration']['shared_clock_domain_declared']}"
        ),
    ]
    for finding in report["findings"]:
        line = (
            f"[{finding['status']}] {finding['topic']} {finding['check']}: "
            f"{finding['reason']}"
        )
        if finding["details"]:
            line += " | " + json.dumps(finding["details"], ensure_ascii=False, sort_keys=True)
        lines.append(line)
    lines.append(
        "summary: "
        + ", ".join(f"{name}={report['counts'][name]}" for name in ("PASS", "WARN", "FAIL"))
    )
    return "\n".join(lines)


def format_report_json(report: Dict[str, Any]) -> str:
    return json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
