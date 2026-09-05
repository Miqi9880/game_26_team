#!/usr/bin/env python3
"""Topic-level dry-run smoke test for the installed AutoAimNode.

This test starts the node itself, publishes only synthetic ROS messages, and
checks the published RobotCtrl safety boundary.  It never opens a serial
device, uses a camera, enables firing, or depends on a model/calibration
artifact.  The existing ros_safety_integration_probe.py remains an operator
probe for the larger offline_reference cases; this CTest covers the minimal
null/mock/timestamp path automatically.
"""

import atexit
import csv
import os
import signal
import subprocess
import tempfile
import time
import uuid

import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from auto_aim_interfaces.msg import Vision
from rclpy.node import Node
from sensor_msgs.msg import Image


ACTIVE_NODE_PROCESSES = []
cleanup_in_progress = False


class SafetyTopics:
    """Unique per-case names so delayed DDS traffic cannot cross scenarios."""

    def __init__(self, label):
        root = f"/auto_aim_ros_safety_{os.getpid()}_{uuid.uuid4().hex}_{label}"
        self.image = f"{root}/image_raw"
        self.vision = f"{root}/Vision_data"
        self.control = f"{root}/Robot_ctrl_data"


class SafetyProbe(Node):
    def __init__(self, topics):
        super().__init__(f"auto_aim_ros_safety_probe_{uuid.uuid4().hex}")
        self.topics = topics
        self.publisher = self.create_publisher(Image, topics.image, 10)
        self.vision_publisher = self.create_publisher(Vision, topics.vision, 10)
        self.subscription = self.create_subscription(
            RobotCtrl, topics.control, self.on_control, 10
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
    topics,
    mock_target=False,
    input_timeout_ms=100,
    mock_yaw_rad=0.0,
    vehicle_profile="new_turtle",
    assume_shared_ros_clock=False,
    alignment_tolerance_ns=-1,
    alignment_csv_path="",
    allow_fire=False,
    output_hz=100.0,
    capture_output=False,
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
        f"allow_fire:={'true' if allow_fire else 'false'}",
        "-p",
        f"mock_target:={'true' if mock_target else 'false'}",
        "-p",
        f"mock_yaw_rad:={mock_yaw_rad}",
        "-p",
        f"vehicle_profile:={vehicle_profile}",
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
        "-p",
        f"output_hz:={output_hz}",
        "-r",
        f"/image_raw:={topics.image}",
        "-r",
        f"/Vision_data:={topics.vision}",
        "-r",
        f"/Robot_ctrl_data:={topics.control}",
        "-r",
        f"__node:=auto_aim_ros_safety_node_{uuid.uuid4().hex}",
    ]
    if alignment_csv_path:
        command.extend(["-p", f"vision_time_alignment_csv_path:={alignment_csv_path}"])
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE if capture_output else subprocess.DEVNULL,
        stderr=subprocess.STDOUT,
        env=os.environ.copy(),
        start_new_session=True,
        text=capture_output,
    )
    ACTIVE_NODE_PROCESSES.append(process)
    return process


def process_group_has_live_member(process_group_id):
    # killpg(..., 0) also succeeds for a group containing only zombies.  A
    # zombie cannot publish and will be reaped by the Popen wait below, so do
    # not turn that harmless transitional state into a teardown failure.
    try:
        os.killpg(process_group_id, 0)
    except ProcessLookupError:
        return False
    for entry in os.scandir("/proc"):
        if not entry.name.isdigit():
            continue
        try:
            with open(os.path.join(entry.path, "stat"), encoding="utf-8") as handle:
                fields = handle.read().rsplit(")", 1)[1].split()
            state = fields[0]
            member_process_group_id = int(fields[2])
        except (FileNotFoundError, IndexError, PermissionError, ValueError):
            continue
        if (member_process_group_id == process_group_id and
                state not in ("Z", "X")):
            return True
    return False


def wait_for_process_group_exit(process_group_id, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if not process_group_has_live_member(process_group_id):
            return True
        time.sleep(0.05)
    return not process_group_has_live_member(process_group_id)


def stop_node(process):
    # ros2 run can spawn the actual node below the launcher.  Always address
    # the original process group even after the launcher leader has exited;
    # otherwise a child could keep publishing into a later test case.
    for signal_number, timeout_seconds in (
        (signal.SIGINT, 5.0),
        (signal.SIGTERM, 2.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, signal_number)
        except ProcessLookupError:
            break
        if wait_for_process_group_exit(process.pid, timeout_seconds):
            break
    if process_group_has_live_member(process.pid):
        raise RuntimeError(f"AutoAimNode process group {process.pid} did not exit")
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("AutoAimNode launcher did not exit") from error
    try:
        ACTIVE_NODE_PROCESSES.remove(process)
    except ValueError:
        pass


def force_stop_node(process):
    """Best-effort final cleanup for a process left by an interrupted case."""
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    wait_for_process_group_exit(process.pid, 2.0)
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        pass
    if process_group_has_live_member(process.pid):
        return
    try:
        ACTIVE_NODE_PROCESSES.remove(process)
    except ValueError:
        pass


def cleanup_active_nodes():
    """Reap every detached ros2-run process group before the test exits."""
    global cleanup_in_progress
    if cleanup_in_progress:
        return
    cleanup_in_progress = True
    try:
        for process in list(ACTIVE_NODE_PROCESSES):
            try:
                stop_node(process)
            except Exception:
                # Do not mask the original test failure; still make a final
                # SIGKILL/reap attempt so a later CTest cannot see a stale node.
                force_stop_node(process)
    finally:
        cleanup_in_progress = False


def handle_shutdown_signal(signum, _frame):
    cleanup_active_nodes()
    raise SystemExit(128 + signum)


atexit.register(cleanup_active_nodes)


def wait_for_graph(probe, timeout_seconds=5.0):
    return spin_until(
        probe,
        lambda: probe.publisher.get_subscription_count() > 0 and
        probe.vision_publisher.get_subscription_count() > 0 and
        probe.count_publishers(probe.topics.control) == 1,
        timeout_seconds,
    )


def new_alignment_csv_path():
    descriptor, path = tempfile.mkstemp(prefix="vision_time_alignment_", suffix=".csv")
    os.close(descriptor)
    os.unlink(path)
    return path


def read_alignment_rows(path):
    try:
        with open(path, newline="", encoding="utf-8") as handle:
            return list(csv.DictReader(handle))
    except (OSError, csv.Error):
        # The child flushes every CSV row, but a read can still race a writer.
        # Treat a transient partial file as no diagnostic yet and retry.
        return []


def alignment_row_matches(row, expected_status, expected_matched_ns,
                          expected_delta_ns, expected_latest_ns,
                          expected_latest_received_ns):
    try:
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
    except (KeyError, TypeError, ValueError):
        return False


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
    assert all(abs(float(message.yaw_vel)) == 0.0 for message in probe.controls)
    assert all(abs(float(message.yaw_acc)) == 0.0 for message in probe.controls)
    assert all(abs(float(message.pitch_vel)) == 0.0 for message in probe.controls)
    assert all(abs(float(message.pitch_acc)) == 0.0 for message in probe.controls)


def close_case(process, probe, alignment_csv_path=""):
    try:
        stop_node(process)
    finally:
        probe.destroy_node()
        if alignment_csv_path and os.path.exists(alignment_csv_path):
            os.unlink(alignment_csv_path)


def run_mock_case(round_index):
    topics = SafetyTopics(f"mock_{round_index}")
    probe = SafetyProbe(topics)
    alignment_csv_path = new_alignment_csv_path()
    process = start_node(
        "mock",
        topics,
        mock_target=True,
        input_timeout_ms=80,
        mock_yaw_rad=0.25,
        assume_shared_ros_clock=True,
        alignment_tolerance_ns=100_000_000,
        alignment_csv_path=alignment_csv_path,
        # Exercise the same nanosecond period conversion used by RobotCtrlSub
        # at a non-integer millisecond frequency.
        output_hz=333.0,
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
        close_case(process, probe, alignment_csv_path)


def run_null_case(round_index):
    topics = SafetyTopics(f"null_{round_index}")
    probe = SafetyProbe(topics)
    process = start_node("null", topics)
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
        close_case(process, probe)


def run_incomparable_alignment_case(round_index):
    topics = SafetyTopics(f"incomparable_{round_index}")
    probe = SafetyProbe(topics)
    alignment_csv_path = new_alignment_csv_path()
    process = start_node(
        "mock",
        topics,
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
        close_case(process, probe, alignment_csv_path)


def run_unselected_profile_case(round_index):
    topics = SafetyTopics(f"unselected_{round_index}")
    probe = SafetyProbe(topics)
    process = start_node(
        "mock", topics, mock_target=True, vehicle_profile="unselected")
    try:
        assert wait_for_graph(probe), "ROS graph discovery did not complete"
        assert spin_until(probe, lambda: bool(probe.controls), 5.0)
        probe.publish_image(4)
        assert spin_until(probe, lambda: len(probe.controls) >= 2, 1.0)
        assert_all_safe(probe)
        assert all(int(message.target_lock) == 50 for message in probe.controls)
    finally:
        close_case(process, probe)


def run_allow_fire_rejected_case(round_index):
    topics = SafetyTopics(f"fire_rejected_{round_index}")
    process = start_node("null", topics, allow_fire=True, capture_output=True)
    try:
        output, _ = process.communicate(timeout=5.0)
        assert process.returncode != 0, output
        assert "allow_fire requires a reviewed production fire authorization" in output, output
    finally:
        stop_node(process)


def main():
    # Keep this CTest independent of unrelated ROS nodes in the developer's
    # session.  The child auto_aim_node inherits the same temporary domain,
    # localhost-only discovery, ROS_HOME, and log directory.  Every case also
    # uses unique remapped topics, so a late or orphaned publisher cannot be
    # mistaken for the node started by a later case.
    repeat_count = int(os.environ.get("AUTO_AIM_ROS_SAFETY_REPEAT", "2"))
    if repeat_count <= 0:
        raise ValueError("AUTO_AIM_ROS_SAFETY_REPEAT must be positive")
    saved_environment = {
        key: os.environ.get(key)
        for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "ROS_HOME", "ROS_LOG_DIR")
    }
    saved_signal_handlers = {
        signal_number: signal.getsignal(signal_number)
        for signal_number in (signal.SIGINT, signal.SIGTERM)
    }
    try:
        for signal_number in saved_signal_handlers:
            signal.signal(signal_number, handle_shutdown_signal)
        with tempfile.TemporaryDirectory(prefix="auto_aim_ros_safety_runtime_") as runtime_dir:
            os.environ["ROS_DOMAIN_ID"] = str(20 + (os.getpid() % 80))
            os.environ["ROS_LOCALHOST_ONLY"] = "1"
            os.environ["ROS_HOME"] = runtime_dir
            os.environ["ROS_LOG_DIR"] = os.path.join(runtime_dir, "log")
            os.makedirs(os.environ["ROS_LOG_DIR"], exist_ok=True)
            rclpy.init()
            try:
                for round_index in range(repeat_count):
                    run_mock_case(round_index)
                    run_incomparable_alignment_case(round_index)
                    run_null_case(round_index)
                    run_unselected_profile_case(round_index)
                    run_allow_fire_rejected_case(round_index)
                print(
                    "ros_safety_integration_test: "
                    f"{repeat_count} isolated rounds; matched/incomparable diagnostics, "
                    "mock lock, timeout, invalid timestamp, profile fail-closed, "
                    "allow_fire rejection, fire=0 OK"
                )
            finally:
                cleanup_active_nodes()
                if rclpy.ok():
                    rclpy.shutdown()
    finally:
        cleanup_active_nodes()
        for signal_number, handler in saved_signal_handlers.items():
            signal.signal(signal_number, handler)
        for key, value in saved_environment.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


if __name__ == "__main__":
    main()
