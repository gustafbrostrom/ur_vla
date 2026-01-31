#!/usr/bin/env python3
# Copyright 2024 UR10 Gazebo Harmonic Stage 1
# SPDX-License-Identifier: BSD-3-Clause
"""Capture one frame from wrist and external camera, concatenate into one image, save to file."""

import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

WRIST_TOPIC = "/wrist_camera/image_raw"
EXTERNAL_TOPIC = "/external_camera/image_raw"
TIMEOUT_SEC = 30.0


def main():
    rclpy.init()
    node = Node("save_camera_images")
    bridge = CvBridge()
    wrist_img = []
    external_img = []

    def wrist_cb(msg: Image) -> None:
        if not wrist_img:
            wrist_img.append(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    def external_cb(msg: Image) -> None:
        if not external_img:
            external_img.append(bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))

    node.create_subscription(Image, WRIST_TOPIC, wrist_cb, 10)
    node.create_subscription(Image, EXTERNAL_TOPIC, external_cb, 10)

    start = node.get_clock().now()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=1.0)
        if wrist_img and external_img:
            break
        if (node.get_clock().now() - start).nanoseconds * 1e-9 > TIMEOUT_SEC:
            if not wrist_img:
                node.get_logger().error(f"Timeout waiting for {WRIST_TOPIC}")
            if not external_img:
                node.get_logger().error(f"Timeout waiting for {EXTERNAL_TOPIC}")
            rclpy.shutdown()
            sys.exit(1)

    rclpy.shutdown()
    if not wrist_img or not external_img:
        sys.exit(1)

    # Concatenate side by side (wrist left, external right); resize if heights differ
    a, b = wrist_img[0], external_img[0]
    if a.shape[0] != b.shape[0]:
        target_h = max(a.shape[0], b.shape[0])
        a = cv2.resize(a, (int(a.shape[1] * target_h / a.shape[0]), target_h))
        b = cv2.resize(b, (int(b.shape[1] * target_h / b.shape[0]), target_h))
    combined = cv2.hconcat([a, b])

    out_path = sys.argv[1] if len(sys.argv) > 1 else "cameras.png"
    cv2.imwrite(out_path, combined)
    print("Saved", out_path)


if __name__ == "__main__":
    main()
