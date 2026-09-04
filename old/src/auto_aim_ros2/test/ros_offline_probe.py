#!/usr/bin/env python3
"""Replay a short reference-video segment through AutoAimNode ROS topics.

This is an operator-side dry-run probe, not a production launch.  It publishes
the test CameraInfo and BGR frames only; it never enables serial or firing.
"""

import argparse
import time

import cv2
import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class Probe(Node):
    def __init__(self, frames):
        super().__init__("auto_aim_ros_offline_probe")
        self.frames = frames
        self.index = 0
        self.controls = []
        self.image_publisher = self.create_publisher(Image, "/image_raw", 10)
        self.info_publisher = self.create_publisher(CameraInfo, "/camera_info", 10)
        self.subscription = self.create_subscription(
            RobotCtrl, "/Robot_ctrl_data", self.on_control, 10
        )
        self.timer = self.create_timer(0.04, self.publish_frame)

    def publish_frame(self):
        frame = self.frames[self.index % len(self.frames)]
        self.index += 1
        info = CameraInfo()
        info.width = int(frame.shape[1])
        info.height = int(frame.shape[0])
        info.k = [1200.0, 0.0, 720.0, 0.0, 1200.0, 540.0, 0.0, 0.0, 1.0]
        info.d = [0.0] * 5
        self.info_publisher.publish(info)

        message = Image()
        message.header.stamp.sec = self.index
        message.header.stamp.nanosec = 1
        message.width = int(frame.shape[1])
        message.height = int(frame.shape[0])
        message.encoding = "bgr8"
        message.step = message.width * 3
        message.data = frame.tobytes()
        self.image_publisher.publish(message)

    def on_control(self, message):
        self.controls.append(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--video", required=True)
    parser.add_argument("--frames", type=int, default=12)
    parser.add_argument("--seconds", type=float, default=5.0)
    args = parser.parse_args()
    if args.frames <= 0 or args.seconds <= 0:
        raise SystemExit("--frames and --seconds must be positive")

    capture = cv2.VideoCapture(args.video)
    frames = []
    for _ in range(args.frames):
        ok, frame = capture.read()
        if not ok:
            break
        frames.append(frame)
    capture.release()
    if not frames:
        raise RuntimeError("reference video produced no frames")

    rclpy.init()
    node = Probe(frames)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        if not node.controls:
            raise RuntimeError("no /Robot_ctrl_data messages received")
        if any(int(message.fire_command) != 0 for message in node.controls):
            raise RuntimeError("observed non-zero fire_command")
        print(
            f"received_control_messages={len(node.controls)} "
            f"last_lock={int(node.controls[-1].target_lock)} "
            f"last_fire={int(node.controls[-1].fire_command)}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
