"""ROS integration coverage driven only by fake input publishers."""

import time

from auto_aim_interfaces.msg import Vision
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from auto_aim_tools.analyzer import PreflightAnalyzer
from auto_aim_tools.ros_input_preflight import RosInputPreflightNode


class FakeInputPublisher(Node):
    def __init__(self, publish_image=True, publish_info=True, publish_vision=True):
        super().__init__("preflight_fake_input_publisher")
        self.image_publisher = (
            self.create_publisher(Image, "/image_raw", qos_profile_sensor_data)
            if publish_image else None
        )
        self.info_publisher = (
            self.create_publisher(CameraInfo, "/camera_info", qos_profile_sensor_data)
            if publish_info else None
        )
        self.vision_publisher = (
            self.create_publisher(Vision, "/Vision_data", qos_profile_sensor_data)
            if publish_vision else None
        )

    @staticmethod
    def image(stamp_sec, empty=False):
        message = Image()
        message.header.stamp.sec = stamp_sec
        message.encoding = "bgr8"
        if not empty:
            message.width = 2
            message.height = 2
            message.step = 6
            message.data = bytes(12)
        return message

    @staticmethod
    def camera_info(stamp_sec, valid=True):
        message = CameraInfo()
        message.header.stamp.sec = stamp_sec
        message.width = 2
        message.height = 2
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [100.0, 0.0, 1.0, 0.0, 100.0, 1.0, 0.0, 0.0, 1.0]
        if not valid:
            message.k[0] = float("nan")
        return message

    @staticmethod
    def vision(stamp_sec, valid=True):
        message = Vision()
        message.header.stamp.sec = stamp_sec
        message.yaw = 10.0 if valid else 181.0
        message.yaw_vel = 1.0
        message.pitch = 5.0
        message.pitch_vel = 1.0
        message.roll = 0.0
        message.quaternion = [1.0, 0.0, 0.0, 0.0]
        message.shoot_speed = 20.0
        return message

    def publish(self, stamp_sec, empty_image=False, valid_info=True, valid_vision=True):
        if self.image_publisher is not None:
            self.image_publisher.publish(self.image(stamp_sec, empty_image))
        if self.info_publisher is not None:
            self.info_publisher.publish(self.camera_info(stamp_sec, valid_info))
        if self.vision_publisher is not None:
            self.vision_publisher.publish(self.vision(stamp_sec, valid_vision))


def finding(report, check, topic=None):
    matches = [item for item in report["findings"] if item["check"] == check]
    if topic is not None:
        matches = [item for item in matches if item["topic"] == topic]
    assert matches, f"missing finding {check} for {topic}"
    return matches[0]


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.02, deadline - time.monotonic()))


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


def run_scenario(
    scenario,
    publish_image=True,
    publish_info=True,
    publish_vision=True,
    timeout_s=0.3,
):
    analyzer = PreflightAnalyzer(
        timeout_s=timeout_s,
        vehicle_profile="new_turtle",
        shared_clock_domain=True,
    )
    preflight = RosInputPreflightNode(analyzer)
    publisher = FakeInputPublisher(publish_image, publish_info, publish_vision)
    executor = SingleThreadedExecutor()
    executor.add_node(preflight)
    executor.add_node(publisher)
    try:
        spin_for(executor, 0.2)
        if scenario == "timestamp_rollback":
            stamps = (10, 9, 11)
        else:
            stamps = (1, 2, 3)
        for stamp_sec in stamps:
            publisher.publish(
                stamp_sec,
                empty_image=scenario == "empty_image",
                valid_info=scenario != "invalid_camera_info",
                valid_vision=scenario != "invalid_vision",
            )
            spin_for(executor, 0.06)
        if scenario == "timeout":
            spin_for(executor, timeout_s + 0.08)
        report = analyzer.build_report()
        assert preflight.count_publishers("/Robot_ctrl_data") == 0
        subscriptions = {name for name, _ in preflight.get_topic_names_and_types()}
        assert "/Robot_ctrl_data" not in subscriptions
        return report
    finally:
        executor.remove_node(publisher)
        executor.remove_node(preflight)
        publisher.destroy_node()
        preflight.destroy_node()
        executor.shutdown()


def test_fake_publishers_normal_input_is_accepted():
    report = run_scenario("normal")
    assert finding(report, "image.data_length")["status"] == "PASS"
    assert finding(report, "camera_info.K")["status"] == "PASS"
    assert finding(report, "vision.yaw_range")["status"] == "PASS"
    # Current main has no Vision acceleration fields, which remains explicit.
    assert finding(report, "vision.acceleration_finite")["status"] == "WARN"


def test_fake_publishers_missing_topic_is_a_failure():
    report = run_scenario("missing_topic", publish_vision=False)
    assert finding(report, "topic.received", "/Vision_data")["status"] == "FAIL"


def test_fake_publishers_empty_image_is_a_failure():
    report = run_scenario("empty_image")
    assert finding(report, "image.data_length")["status"] == "FAIL"


def test_fake_publishers_invalid_camera_info_is_a_failure():
    report = run_scenario("invalid_camera_info")
    assert finding(report, "camera_info.K")["status"] == "FAIL"


def test_fake_publishers_timestamp_rollback_is_a_failure():
    report = run_scenario("timestamp_rollback")
    assert finding(
        report, "header.timestamp_monotonic", "/image_raw"
    )["status"] == "FAIL"
    assert finding(
        report, "header.timestamp_monotonic", "/Vision_data"
    )["status"] == "FAIL"


def test_fake_publishers_timeout_is_a_failure():
    report = run_scenario("timeout", timeout_s=0.1)
    assert finding(report, "topic.timeout", "/image_raw")["status"] == "FAIL"
    assert finding(report, "topic.timeout", "/Vision_data")["status"] == "FAIL"


def test_fake_publishers_invalid_vision_is_a_failure():
    report = run_scenario("invalid_vision")
    assert finding(report, "vision.yaw_range")["status"] == "FAIL"
