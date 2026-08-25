"""ROS 2 entry point for the read-only auto-aim input preflight."""

from __future__ import annotations

import argparse
import math
import sys
import time
from typing import Any, Callable, Optional, Sequence

from auto_aim_interfaces.msg import Vision
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import CameraInfo, Image

from .analyzer import (
    CAMERA_INFO_TOPIC,
    IMAGE_TOPIC,
    VISION_TOPIC,
    PITCH_PROFILES,
    PreflightAnalyzer,
    format_report_json,
    format_report_text,
)


class RosInputPreflightNode(Node):
    """A subscription-only adapter around :class:`PreflightAnalyzer`."""

    def __init__(self, analyzer: PreflightAnalyzer) -> None:
        super().__init__("ros_input_preflight")
        self.analyzer = analyzer
        self._image_subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            lambda message: self._observe(
                IMAGE_TOPIC, self.analyzer.observe_image, message
            ),
            qos_profile_sensor_data,
        )
        self._camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            lambda message: self._observe(
                CAMERA_INFO_TOPIC, self.analyzer.observe_camera_info, message
            ),
            qos_profile_sensor_data,
        )
        self._vision_subscription = self.create_subscription(
            Vision,
            VISION_TOPIC,
            lambda message: self._observe(
                VISION_TOPIC, self.analyzer.observe_vision, message
            ),
            qos_profile_sensor_data,
        )

    def _observe(
        self,
        topic: str,
        observer: Callable[[Any, Optional[float]], None],
        message: Any,
    ) -> None:
        try:
            observer(message, time.monotonic())
        except Exception as error:  # Keep diagnostics alive on malformed input.
            self.analyzer.record_callback_error(topic, error)


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def _non_negative_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and non-negative")
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe to /image_raw, /camera_info, and /Vision_data and emit "
            "a read-only preflight report."
        )
    )
    parser.add_argument(
        "--duration",
        type=_positive_float,
        default=5.0,
        help="observation duration in seconds (default: 5)",
    )
    parser.add_argument(
        "--timeout",
        type=_positive_float,
        default=1.0,
        help="image and Vision receive timeout in seconds (default: 1)",
    )
    parser.add_argument(
        "--format",
        choices=("text", "json"),
        default="text",
        help="report format (default: text)",
    )
    parser.add_argument(
        "--vehicle-profile",
        choices=("unselected", *PITCH_PROFILES.keys()),
        default="unselected",
        help="explicit profile used only for pitch range diagnostics",
    )
    parser.add_argument(
        "--assume-shared-clock-domain",
        action="store_true",
        help=(
            "explicitly declare that Image and Vision Header stamps use the "
            "same clock domain; absent this flag they are never compared"
        ),
    )
    parser.add_argument(
        "--sync-tolerance-ms",
        type=_non_negative_float,
        default=50.0,
        help="diagnostic timestamp delta tolerance after shared-clock declaration",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    raw_args = list(sys.argv if argv is None else argv)
    parser = build_argument_parser()
    try:
        application_args = remove_ros_args(args=raw_args)
        args = parser.parse_args(application_args[1:])
    except (argparse.ArgumentError, rclpy.exceptions.InvalidROSArgsError) as error:
        print(f"FAIL: invalid arguments: {error}", file=sys.stderr)
        return 2

    analyzer = PreflightAnalyzer(
        timeout_s=args.timeout,
        vehicle_profile=args.vehicle_profile,
        shared_clock_domain=args.assume_shared_clock_domain,
        sync_tolerance_ms=args.sync_tolerance_ms,
    )
    node: Optional[RosInputPreflightNode] = None
    initialized = False
    try:
        rclpy.init(args=raw_args)
        initialized = True
        node = RosInputPreflightNode(analyzer)
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        report = analyzer.build_report()
        rendered = (
            format_report_json(report)
            if args.format == "json"
            else format_report_text(report)
        )
        print(rendered)
        return 2 if report["overall"] == "FAIL" else 0
    except KeyboardInterrupt:
        report = analyzer.build_report()
        rendered = (
            format_report_json(report)
            if args.format == "json"
            else format_report_text(report)
        )
        print(rendered)
        return 130
    except Exception as error:
        print(f"FAIL: preflight runtime error: {type(error).__name__}: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
