#!/usr/bin/env python3
"""Command the gimbal to an absolute (yaw, pitch) pose in degrees.

The target direction is published to /Robot_ctrl_data (RobotCtrl msg);
robot_ctrl_node forwards it to the MCU.  fire_command stays 0 (no fire).

Usage:
  python3 tools/cmd_gimbal.py YAW_DEG PITCH_DEG
"""

import sys
import time

import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import RobotCtrl


def main() -> None:
    if len(sys.argv) != 3:
        sys.exit("usage: cmd_gimbal.py YAW_DEG PITCH_DEG")
    yaw, pitch = float(sys.argv[1]), float(sys.argv[2])
    rclpy.init()
    node = Node("cmd_gimbal")
    publisher = node.create_publisher(RobotCtrl, "/Robot_ctrl_data", 10)
    # Wait for the serial bridge to subscribe; discovery can take longer
    # than the publish window.
    deadline = time.monotonic() + 5.0
    while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
        time.sleep(0.1)
    subscribers = publisher.get_subscription_count()
    message = RobotCtrl()
    message.yaw = yaw
    message.pitch = pitch
    message.target_lock = 49
    message.fire_command = 0
    sent = 0
    for _ in range(50):
        publisher.publish(message)
        sent += 1
        time.sleep(0.2)
    node.get_logger().info(
        f"commanded yaw={yaw} pitch={pitch} lock=49 fire=0 "
        f"subscribers={subscribers} sent={sent}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
