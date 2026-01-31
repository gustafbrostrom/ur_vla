#!/usr/bin/env python3
# Copyright 2024 UR10 Gazebo Harmonic Stage 1
# SPDX-License-Identifier: BSD-3-Clause
"""Subscribe once to /wrist_camera/image_raw, convert with cv_bridge, save to file."""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


def main():
    rclpy.init()
    node = Node("save_wrist_image")
    bridge = CvBridge()
    received = []

    def cb(msg: Image) -> None:
        received.append(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    node.create_subscription(Image, "/wrist_camera/image_raw", cb, 10)
    timeout_sec = 30.0
    start = node.get_clock().now()
    while rclpy.ok() and len(received) == 0:
        rclpy.spin_once(node, timeout_sec=1.0)
        if (node.get_clock().now() - start).nanoseconds * 1e-9 > timeout_sec:
            node.get_logger().error("Timeout waiting for /wrist_camera/image_raw")
            rclpy.shutdown()
            sys.exit(1)

    rclpy.shutdown()
    if not received:
        sys.exit(1)

    out_path = sys.argv[1] if len(sys.argv) > 1 else "wrist_camera.png"
    cv2.imwrite(out_path, received[0])
    print("Saved", out_path)


if __name__ == "__main__":
    main()
