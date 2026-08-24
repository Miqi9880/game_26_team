#!/usr/bin/env python3
"""Topic-level dry-run smoke test for the installed AutoAimNode.

This test starts the node itself, publishes only synthetic ROS messages, and
checks the published RobotCtrl safety boundary.  It never opens a serial
device, uses a camera, enables firing, or depends on a model/calibration
artifact.  The existing ros_safety_integration_probe.py remains an operator
probe for the larger offline_reference cases; this CTest covers the minimal
null/mock/timestamp path automatically.
"""

import csv
import os
import signal
import subprocess
import tempfile
import time

import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from auto_aim_interfaces.msg import Vision
from rclpy.node import Node
from sensor_msgs.msg import Image


class SafetyProbe(Node):
    def __init__(self):
        super().__init__("auto_aim_ros_safety_ctest_probe")
        self.publisher = self.create_publisher(Image, "/image_raw", 10)
        self.vision_publisher = self.create_publisher(Vision, "/Vision_data", 10)
        self.subscription = self.create_subscription(
            RobotCtrl, "/Robot_ctrl_data", self.on_control, 10
        )
        self.controls = []

    def on_control(self, message):
        self.controls.append(message)

    def publish_image(self, sec, nanosec=1):
        message = Image()
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.width = 2
        message.height = 2
        message.encoding = "bgr8"
        message.step = 6
        message.data = bytes([0, 0, 0] * 4)
        self.publisher.publish(message)

    def publish_vision(self, sec, nanosec=1, yaw=90.0):
        message = Vision()
        message.header.stamp.sec = sec
        message.header.stamp.nanosec = nanosec
        message.header.frame_id = "vision_header_clock"
        message.yaw = yaw
        message.pitch = -45.0
        message.roll = 5.0
        message.quaternion = [1.0, 0.0, 0.0, 0.0]
        message.shoot_speed = 35.0
        message.bullet_count = 2
        message.game_progress = 1
        self.vision_publisher.publish(message)


def spin_until(probe, predicate, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.02)
        if predicate():
            return True
    return False


def start_node(
    backend,
    mock_target=False,
    input_timeout_ms=100,
    mock_yaw_rad=0.0,
    assume_shared_ros_clock=False,
    alignment_tolerance_ns=-1,
    alignment_csv_path="",
):
    command = [
        "ros2",
        "run",
        "auto_aim_ros2",
        "auto_aim_node",
        "--ros-args",
        "-p",
        f"backend:={backend}",
        "-p",
        "dry_run:=true",
        "-p",
        "serial_enabled:=false",
        "-p",
        "allow_fire:=false",
        "-p",
        f"mock_target:={'true' if mock_target else 'false'}",
        "-p",
        f"mock_yaw_rad:={mock_yaw_rad}",
        "-p",
        f"vision_time_alignment_assume_shared_ros_clock:={'true' if assume_shared_ros_clock else 'false'}",
        "-p",
        f"vision_time_alignment_tolerance_ns:={alignment_tolerance_ns}",
        "-p",
        "mock_fire_request:=true",
        "-p",
        "require_camera_info:=false",
        "-p",
        f"input_timeout_ms:={input_timeout_ms}",
    ]
    if alignment_csv_path:
        command.extend(["-p", f"vision_time_alignment_csv_path:={alignment_csv_path}"])
    return subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        start_new_session=True,
    )


def stop_node(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5.0)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=2.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.kill()
            process.wait(timeout=2.0)


def wait_for_graph(probe, timeout_seconds=5.0):
    return spin_until(
        probe,
        lambda: probe.publisher.get_subscription_count() > 0 and
        probe.vision_publisher.get_subscription_count() > 0 and
        probe.count_publishers("/Robot_ctrl_data") > 0,
        timeout_seconds,
    )


def new_alignment_csv_path():
    descriptor, path = tempfile.mkstemp(prefix="vision_time_alignment_", suffix=".csv")
    os.close(descriptor)
    os.unlink(path)
    return path


def read_alignment_rows(path):
    if not os.path.exists(path):
        return []
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def alignment_row_matches(row, expected_status, expected_matched_ns,
                          expected_delta_ns, expected_latest_ns,
                          expected_latest_received_ns):
    return (
        row["status"] == expected_status and
        (expected_matched_ns is None or
         int(row["matched_stamp_ns"]) == expected_matched_ns) and
        (expected_delta_ns is None or int(row["delta_ns"]) == expected_delta_ns) and
        (expected_latest_ns is None or
         int(row["latest_history_vision_header_ns"]) == expected_latest_ns) and
        (expected_latest_received_ns is None or
         int(row["latest_received_vision_header_ns"]) == expected_latest_received_ns)
    )


def wait_for_alignment_status(probe, path, image_sec, expected_status,
                              expected_matched_ns=None, expected_delta_ns=None,
                              expected_latest_ns=None,
                              expected_latest_received_ns=None):
    deadline = time.monotonic() + 5.0
    rows = []
    while time.monotonic() < deadline:
        # Replaying the same synthetic image timestamp is safe in this
        # dry-run test and removes a cross-process DDS callback ordering race:
        # the assertion succeeds only after the child node has received the
        # expected Vision history.
        probe.publish_image(image_sec)
        rclpy.spin_once(probe, timeout_sec=0.02)
        rows = read_alignment_rows(path)
        for row in reversed(rows):
            if alignment_row_matches(
                    row, expected_status, expected_matched_ns, expected_delta_ns,
                    expected_latest_ns, expected_latest_received_ns):
                return row
        time.sleep(0.02)

    raise AssertionError(
        (expected_status, expected_matched_ns, expected_delta_ns,
         expected_latest_ns, expected_latest_received_ns, rows)
    )


def assert_all_safe(probe):
    assert probe.controls, "AutoAimNode did not publish /Robot_ctrl_data"
    assert all(int(message.fire_command) == 0 for message in probe.controls)


def run_mock_case():
    alignment_csv_path = new_alignment_csv_path()
    process = start_node(
        "mock",
        mock_target=True,
        input_timeout_ms=80,
        mock_yaw_rad=0.25,
        assume_shared_ros_clock=True,
        alignment_tolerance_ns=100_000_000,
        alignment_csv_path=alignment_csv_path,
    )
    try:
        assert wait_for_graph(probe), "ROS graph discovery did not complete"
        assert spin_until(probe, lambda: bool(probe.controls), 5.0)
        # This test explicitly asserts a shared ROS header domain only to
        # exercise the matched diagnostic path.  It is still diagnostic-only;
        # the 90-degree Vision yaw and shoot speed must not alter the mock
        # command or fire output.
        probe.publish_vision(1)
        probe.publish_vision(2)
        matched_row = wait_for_alignment_status(
            probe,
            alignment_csv_path,
            1,
            "matched",
            expected_matched_ns=1_000_000_001,
            expected_delta_ns=0,
            expected_latest_ns=2_000_000_001,
            expected_latest_received_ns=2_000_000_001,
        )
        assert matched_row["image_timestamp_domain"] == "shared_ros_header"
        assert matched_row["vision_timestamp_domain"] == "shared_ros_header"
        assert spin_until(
            probe,
            lambda: any(int(message.target_lock) == 49 for message in probe.controls),
            5.0,
        ), [int(message.target_lock) for message in probe.controls]
        assert_all_safe(probe)
        locked = [message for message in probe.controls if int(message.target_lock) == 49]
        assert locked, "mock target did not produce a locked command"
        assert all(abs(float(message.yaw) - 0.25 * 180.0 / 3.141592653589793) < 1e-3
                   for message in locked)

        # A malformed timestamp must not refresh the image watchdog.  After
        # the timeout, the node must return to the safe unlocked output.
        probe.publish_image(-1)
        assert spin_until(
            probe,
            lambda: probe.controls and int(probe.controls[-1].target_lock) == 50,
            1.0,
        ), [int(message.target_lock) for message in probe.controls]
        assert_all_safe(probe)
    finally:
        stop_node(process)
        if os.path.exists(alignment_csv_path):
            os.unlink(alignment_csv_path)


def run_null_case():
    process = start_node("null")
    try:
        assert wait_for_graph(probe), "ROS graph discovery did not complete"
        assert spin_until(probe, lambda: bool(probe.controls), 5.0)
        probe.publish_image(2, 1_000_000_000)
        # The image is rejected before backend processing; all controls remain
        # unlocked and fire-inhibited.
        assert spin_until(probe, lambda: len(probe.controls) >= 2, 1.0)
        assert_all_safe(probe)
        assert all(int(message.target_lock) == 50 for message in probe.controls)
    finally:
        stop_node(process)


def run_incomparable_alignment_case():
    alignment_csv_path = new_alignment_csv_path()
    process = start_node(
        "mock",
        mock_target=True,
        input_timeout_ms=200,
        mock_yaw_rad=0.25,
        # The default keeps Image and Vision headers in separate, unproven
        # clock domains.  This deliberately exercises the fail-closed
        # diagnostic path while the mock target remains independent of it.
        assume_shared_ros_clock=False,
        alignment_tolerance_ns=100_000_000,
        alignment_csv_path=alignment_csv_path,
    )
    try:
        assert wait_for_graph(probe), "ROS graph discovery did not complete"
        assert spin_until(probe, lambda: bool(probe.controls), 5.0)
        probe.publish_vision(3)
        incomparable_row = wait_for_alignment_status(
            probe, alignment_csv_path, 3, "incomparable")
        assert incomparable_row["matched_stamp_ns"] == "0"
        assert incomparable_row["image_timestamp_domain"] == "image_header"
        assert incomparable_row["vision_timestamp_domain"] == "vision_header"
        assert spin_until(
            probe,
            lambda: any(int(message.target_lock) == 49 for message in probe.controls),
            5.0,
        ), [int(message.target_lock) for message in probe.controls]
        # A cross-domain pairing failure is diagnostic-only and must not
        # introduce a fire request or alter the independent mock command.
        assert_all_safe(probe)
        locked = [message for message in probe.controls if int(message.target_lock) == 49]
        assert all(abs(float(message.yaw) - 0.25 * 180.0 / 3.141592653589793) < 1e-3
                   for message in locked)
    finally:
        stop_node(process)
        if os.path.exists(alignment_csv_path):
            os.unlink(alignment_csv_path)


def main():
    global probe
    # Keep this CTest independent of unrelated ROS nodes in the developer's
    # session.  The child auto_aim_node inherits the same temporary domain.
    # Keep the value in the broadly supported 0-101 DDS domain range and
    # restrict discovery to localhost for this process tree.
    os.environ["ROS_DOMAIN_ID"] = str(20 + (os.getpid() % 80))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init()
    probe = SafetyProbe()
    try:
        run_mock_case()
        probe.controls.clear()
        run_incomparable_alignment_case()
        probe.controls.clear()
        run_null_case()
        print(
            "ros_safety_integration_test: matched/incomparable diagnostics, "
            "mock lock, timeout, invalid timestamp, fire=0 OK"
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
