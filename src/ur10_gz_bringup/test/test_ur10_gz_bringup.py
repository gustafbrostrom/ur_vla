# Copyright 2024 UR10 Gazebo Harmonic Stage 1
# SPDX-License-Identifier: BSD-3-Clause

import os
import time
import unittest

import launch
import launch.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.decorator
import rclpy
from control_msgs.action import FollowJointTrajectory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from launch.actions import IncludeLaunchDescription
except ImportError:
    from launch_ros.actions import IncludeLaunchDescription

EXPECTED_JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "robotiq_85_left_knuckle_joint",
]
JOINT_STATES_TIMEOUT = 60.0
CAMERA_TIMEOUT = 30.0
ACTION_RESULT_TIMEOUT = 20.0
# Must be < launch_testing's ~15s "wait for processes to start" timeout.
# Tests then wait for conditions (e.g. joint_states up to JOINT_STATES_TIMEOUT).
READY_DELAY = 10.0


def generate_test_description():
    # Use environment GZ_GUI (default true): allow GUI when DISPLAY is available
    # to avoid "Unable to open display" / "Authorization required" in headless.
    # For headless/CI, set GZ_GUI=false before running the test.
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur10_gz_bringup"),
                "launch",
                "ur10_gz_bringup.launch.py",
            ])
        ),
    )
    return (
        launch.LaunchDescription([
            bringup_launch,
            launch.actions.TimerAction(
                period=READY_DELAY,
                actions=[launch_testing.actions.ReadyToTest()],
            ),
        ]),
        {},
    )


class TestUr10GzBringup(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_ur10_gz_bringup")

    def tearDown(self):
        self.node.destroy_node()

    def test_robot_spawns(self, proc_output):
        """Check that /joint_states is published with UR10 arm and Robotiq gripper joints."""
        msgs = []

        def cb(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            cb,
            10,
        )
        end_time = time.monotonic() + JOINT_STATES_TIMEOUT
        try:
            while time.monotonic() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1.0)
                if msgs:
                    break
            self.assertGreater(len(msgs), 0, "No /joint_states message received")
            msg = msgs[0]
            names = set(msg.name)
            for name in EXPECTED_JOINT_NAMES:
                self.assertIn(name, names, f"Joint {name} not in joint_states")
            name_to_pos = dict(zip(msg.name, msg.position))
            start_pos = [name_to_pos[n] for n in EXPECTED_JOINT_NAMES]
            print(f"\n[test] Robot initial joint positions (rad): {dict(zip(EXPECTED_JOINT_NAMES, start_pos))}")
        finally:
            self.node.destroy_subscription(sub)

    def test_camera_delivers_images(self, proc_output):
        """Check that /wrist_camera/image_raw publishes with valid dimensions and data."""
        msgs = []

        def cb(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(
            Image,
            "/wrist_camera/image_raw",
            cb,
            10,
        )
        end_time = time.monotonic() + CAMERA_TIMEOUT
        try:
            while time.monotonic() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1.0)
                if msgs:
                    break
            self.assertGreater(len(msgs), 0, "No /wrist_camera/image_raw message received")
            img = msgs[0]
            self.assertEqual(img.height, 480, "Unexpected image height")
            self.assertEqual(img.width, 640, "Unexpected image width")
            self.assertGreater(len(img.data), 0, "Image data is empty")
            print(f"\n[test] Image size: {img.width} x {img.height}, data length: {len(img.data)} bytes")
        finally:
            self.node.destroy_subscription(sub)

    def test_external_camera_delivers_images(self, proc_output):
        """Check that /external_camera/image_raw publishes with valid dimensions and data."""
        msgs = []

        def cb(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(
            Image,
            "/external_camera/image_raw",
            cb,
            10,
        )
        end_time = time.monotonic() + CAMERA_TIMEOUT
        try:
            while time.monotonic() < end_time:
                rclpy.spin_once(self.node, timeout_sec=1.0)
                if msgs:
                    break
            self.assertGreater(len(msgs), 0, "No /external_camera/image_raw message received")
            img = msgs[0]
            self.assertEqual(img.height, 480, "Unexpected image height")
            self.assertEqual(img.width, 640, "Unexpected image width")
            self.assertGreater(len(img.data), 0, "Image data is empty")
            print(f"\n[test] External camera image size: {img.width} x {img.height}, data length: {len(img.data)} bytes")
        finally:
            self.node.destroy_subscription(sub)

    def _get_joint_positions(self, timeout_sec=5.0):
        """Subscribe to /joint_states and return positions for EXPECTED_JOINT_NAMES (in order)."""
        msgs = []

        def cb(msg):
            msgs.append(msg)

        sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            cb,
            10,
        )
        end_time = time.monotonic() + timeout_sec
        try:
            while time.monotonic() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.5)
                if not msgs:
                    continue
                msg = msgs[-1]
                name_to_pos = dict(zip(msg.name, msg.position))
                if all(n in name_to_pos for n in EXPECTED_JOINT_NAMES):
                    return [name_to_pos[n] for n in EXPECTED_JOINT_NAMES]
            return None
        finally:
            self.node.destroy_subscription(sub)

    def test_robot_moves_on_command(self, proc_output):
        """Send a FollowJointTrajectory goal and assert it succeeds."""
        from rclpy.action import ActionClient

        start_positions = self._get_joint_positions()
        self.assertIsNotNone(start_positions, "Could not read joint_states before command")
        print(f"\n[test] Robot started at (rad): {dict(zip(EXPECTED_JOINT_NAMES, start_positions))}")

        client = ActionClient(
            self.node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.assertTrue(
            client.wait_for_server(timeout_sec=10.0),
            "Action server not available",
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(EXPECTED_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]
        point.time_from_start.sec = 2
        point.time_from_start.nanosec = 0
        goal.trajectory.points = [point]
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        self.assertTrue(send_future.done(), "Send goal did not complete")
        goal_handle = send_future.result()
        self.assertIsNotNone(goal_handle, "Goal was rejected")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self.node, result_future, timeout_sec=ACTION_RESULT_TIMEOUT
        )
        self.assertTrue(result_future.done(), "Result did not complete")
        result = result_future.result().result
        self.assertIsNotNone(result, "Result is None")
        self.assertEqual(
            result.error_code,
            FollowJointTrajectory.Result.SUCCESSFUL,
            f"Trajectory failed with error_code {result.error_code}",
        )

        end_positions = self._get_joint_positions()
        self.assertIsNotNone(end_positions, "Could not read joint_states after command")
        print(f"[test] Robot ended at (rad):   {dict(zip(EXPECTED_JOINT_NAMES, end_positions))}")
        print(f"[test] Goal was (rad):         {dict(zip(EXPECTED_JOINT_NAMES, point.positions))}")


@launch_testing.decorator.post_shutdown_test()
class TestUr10GzBringupShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        """Check that the launched processes exited normally or with SIGTERM."""
        # Allow 0 (clean) and -15 (SIGTERM): Gazebo often does not exit on SIGINT
        # within the shutdown timeout and is killed with SIGTERM.
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -15],
        )
