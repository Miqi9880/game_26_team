#!/usr/bin/env python3
"""Topic-level dry-run smoke test for the installed AutoAimNode.

This test starts the node itself, publishes only synthetic ROS messages, and
checks the published RobotCtrl safety boundary.  It never opens a serial
device, uses a camera, enables firing, or depends on a model/calibration
artifact.  The existing ros_safety_integration_probe.py remains an operator
probe for the larger offline_reference cases; this CTest covers the minimal
null/mock/timestamp path automatically.
"""

import os
import signal
import subprocess
import time

import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from rclpy.node import Node
from sensor_msgs.msg import Image


class SafetyProbe(Node):
    def __init__(self):
        super().__init__("auto_aim_ros_safety_ctest_probe")
        self.publisher = self.create_publisher(Image, "/image_raw", 10)
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


def spin_until(probe, predicate, timeout_seconds):
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(probe, timeout_sec=0.02)
        if predicate():
            return True
    return False


def start_node(backend, mock_target=False, input_timeout_ms=100):
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
        "mock_fire_request:=true",
        "-p",
        "require_camera_info:=false",
        "-p",
        f"input_timeout_ms:={input_timeout_ms}",
    ]
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
        probe.count_publishers("/Robot_ctrl_data") > 0,
        timeout_seconds,
    )


def assert_all_safe(probe):
    assert probe.controls, "AutoAimNode did not publish /Robot_ctrl_data"
    assert all(int(message.fire_command) == 0 for message in probe.controls)


def run_mock_case():
    process = start_node("mock", mock_target=True, input_timeout_ms=80)
    try:
        assert wait_for_graph(probe), "ROS graph discovery did not complete"
        assert spin_until(probe, lambda: bool(probe.controls), 5.0)
        probe.publish_image(1)
        assert spin_until(
            probe,
            lambda: any(int(message.target_lock) == 49 for message in probe.controls),
            5.0,
        ), [int(message.target_lock) for message in probe.controls]
        assert_all_safe(probe)

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
        run_null_case()
        print("ros_safety_integration_test: mock lock, timeout, invalid timestamp, fire=0 OK")
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
