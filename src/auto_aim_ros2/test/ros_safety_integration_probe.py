#!/usr/bin/env python3
"""ROS topic-level safety probe for an already running AutoAimNode.

The node must be launched separately with its intended backend.  This probe
only publishes synthetic image/CameraInfo messages and observes
``/Robot_ctrl_data``; it never opens a serial device or enables firing.

Cases cover the dry-run gates that are easy to regress at the ROS boundary:
``no_target``, ``mock_target``, ``missing_camera_info``, ``invalid_camera_info``,
``input_timeout`` and ``offline_reference``.  For short/long target loss, use
``input_timeout`` with ``--frames-before-stop`` set to a small or large value.

The minimal null/mock/invalid-timestamp path is also registered as the
automatic ``ros_safety_integration_test`` CTest.  This script remains an
operator-side probe for camera-info and offline_reference cases that require
explicit runtime artifacts and a separately launched node.
"""

import argparse
import time

import cv2
import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class SyntheticFrame:
    shape = (2, 2, 3)

    @staticmethod
    def tobytes():
        return bytes([0, 0, 0] * 4)


class Probe(Node):
    def __init__(self, frames, publish_info, valid_info, frames_before_stop):
        super().__init__("auto_aim_ros_safety_integration_probe")
        self.frames = frames
        self.publish_info = publish_info
        self.valid_info = valid_info
        self.frames_before_stop = frames_before_stop
        self.index = 0
        self.controls = []
        self.publisher = self.create_publisher(Image, "/image_raw", 10)
        self.info_publisher = self.create_publisher(CameraInfo, "/camera_info", 10)
        self.subscription = self.create_subscription(
            RobotCtrl, "/Robot_ctrl_data", self.on_control, 10
        )
        self.timer = self.create_timer(0.02, self.publish_frame)

    def publish_frame(self):
        if self.frames_before_stop >= 0 and self.index >= self.frames_before_stop:
            return
        frame = self.frames[self.index % len(self.frames)]
        self.index += 1
        if self.publish_info:
            info = CameraInfo()
            info.width = int(frame.shape[1])
            info.height = int(frame.shape[0])
            info.k = [1200.0, 0.0, 720.0, 0.0, 1200.0, 540.0, 0.0, 0.0, 1.0]
            info.d = [0.0] * 5
            if not self.valid_info:
                info.k[0] = 0.0
            self.info_publisher.publish(info)

        message = Image()
        message.header.stamp.sec = self.index
        message.header.stamp.nanosec = 1
        message.width = int(frame.shape[1])
        message.height = int(frame.shape[0])
        message.encoding = "bgr8"
        message.step = message.width * 3
        message.data = frame.tobytes()
        self.publisher.publish(message)

    def on_control(self, message):
        self.controls.append(message)


def read_frames(video_path, count):
    if video_path:
        capture = cv2.VideoCapture(video_path)
        frames = []
        for _ in range(count):
            ok, frame = capture.read()
            if not ok:
                break
            frames.append(frame)
        capture.release()
        if frames:
            return frames
    # A null/mock backend still needs a valid image message.  A 2x2 BGR image
    # is enough and keeps this probe independent of detector model artifacts.
    return [SyntheticFrame()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--case",
        choices=(
            "no_target",
            "mock_target",
            "missing_camera_info",
            "invalid_camera_info",
            "input_timeout",
            "offline_reference",
        ),
        required=True,
    )
    parser.add_argument("--video", default="")
    parser.add_argument("--frames", type=int, default=8)
    parser.add_argument("--frames-before-stop", type=int, default=-1)
    parser.add_argument("--seconds", type=float, default=1.0)
    args = parser.parse_args()
    if args.frames <= 0 or args.seconds <= 0:
        raise SystemExit("--frames and --seconds must be positive")

    frames = read_frames(args.video, args.frames)
    publish_info = args.case not in ("missing_camera_info",)
    valid_info = args.case not in ("invalid_camera_info",)
    stop_after = args.frames_before_stop
    if args.case == "input_timeout" and stop_after < 0:
        stop_after = 2

    rclpy.init()
    node = Probe(frames, publish_info, valid_info, stop_after)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)
        if not node.controls:
            raise RuntimeError("no /Robot_ctrl_data messages received")
        if any(int(message.fire_command) != 0 for message in node.controls):
            raise RuntimeError("observed non-zero fire_command")

        locks = [int(message.target_lock) for message in node.controls]
        if args.case in ("no_target", "missing_camera_info", "invalid_camera_info"):
            if any(lock != 50 for lock in locks):
                raise RuntimeError(f"unsafe lock state for {args.case}: {locks}")
        if args.case == "mock_target" and 49 not in locks:
            raise RuntimeError(f"mock target never reached target_lock=49: {locks}")
        if args.case == "input_timeout" and locks[-1] != 50:
            raise RuntimeError(f"timeout did not end unlocked: {locks}")

        print(
            f"case={args.case} messages={len(node.controls)} "
            f"lock_values={sorted(set(locks))} last_lock={locks[-1]} fire=0"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
