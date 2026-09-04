#!/usr/bin/env python3
"""Small ROS 2 dry-run probe for AutoAimNode's image/control topics.

The probe only publishes synthetic BGR images and observes RobotCtrl.  It
never opens a serial device and fails unless a locked mock command is seen
with fire_command=0.
"""

import argparse
import time

import rclpy
from auto_aim_interfaces.msg import RobotCtrl
from rclpy.node import Node
from sensor_msgs.msg import Image


class Probe(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("auto_aim_ros_topic_probe")
        self.publisher = self.create_publisher(Image, topic, 10)
        self.received = []
        self.index = 1
        self.timer = self.create_timer(0.02, self.publish_image)
        self.subscription = self.create_subscription(
            RobotCtrl, "/Robot_ctrl_data", self.on_control, 10
        )

    def publish_image(self) -> None:
        message = Image()
        message.header.stamp.nanosec = self.index
        message.width = 2
        message.height = 2
        message.encoding = "bgr8"
        message.step = 6
        message.data = bytes([0, 0, 0] * 4)
        self.publisher.publish(message)
        self.index += 1

    def on_control(self, message: RobotCtrl) -> None:
        self.received.append(message)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=0.5)
    parser.add_argument("--image-topic", default="/image_raw")
    args = parser.parse_args()
    if args.seconds <= 0:
        raise SystemExit("--seconds must be positive")

    rclpy.init()
    node = Probe(args.image_topic)
    try:
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.03)
        if not node.received:
            raise RuntimeError("no /Robot_ctrl_data message received")
        if not any(int(message.target_lock) == 49 for message in node.received):
            raise RuntimeError("mock target never reached target_lock=49")
        if any(int(message.fire_command) != 0 for message in node.received):
            raise RuntimeError("probe observed a non-zero fire_command")
        print(
            "received_control_messages="
            f"{len(node.received)} last_lock={int(node.received[-1].target_lock)} "
            f"last_fire={int(node.received[-1].fire_command)}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
