# Copyright 2024 UR10 Gazebo Harmonic Stage 1
# SPDX-License-Identifier: BSD-3-Clause

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_file = PathJoinSubstitution([
        FindPackageShare("ur10_gz_bringup"),
        "worlds",
        "simple_ground_light.sdf",
    ])

    # Headless (no GUI) when GZ_GUI is 0/false/no - avoids "could not connect to display"
    headless = os.environ.get("GZ_GUI", "true").lower() in ("0", "false", "no")
    gz_args_prefix = "-s -r -v 1 " if headless else "-r -v 1 "

    # Robot description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur10_description"),
                "urdf",
                "ur10_gz.urdf.xacro",
            ]),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Gazebo
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            ])
        ),
        launch_arguments=[
            ("gz_args", [TextSubstitution(text=gz_args_prefix), world_file]),
        ],
    )

    # Robot state publisher
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
    )

    # Spawn UR10 in Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "ur10",
            "-allow_renaming", "true",
        ],
    )

    # Controllers: broadcaster first, then trajectory controller
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("ur10_gz_bringup"), "config", "controllers.yaml",
    ])
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--param-file", robot_controllers],
    )

    # Clock bridge and camera CameraInfo bridges (Gazebo -> ROS 2)
    gz_camera_info = "/model/ur10/link/wrist_camera_link/sensor/wrist_camera/camera_info"
    # External camera uses world-scoped topic (same as image)
    gz_external_camera_info = "/world/default/model/external_camera/link/link/sensor/camera/camera_info"
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"{gz_camera_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            f"{gz_external_camera_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        remappings=[
            (gz_camera_info, "/wrist_camera/camera_info"),
            (gz_external_camera_info, "/external_camera/camera_info"),
        ],
        output="screen",
    )

    # Wrist camera image bridge (Gazebo -> ROS 2). Gazebo publishes the camera to /image.
    # image_bridge subscribes to the Gazebo topic and publishes to ROS under the same name;
    # remap the ROS publisher so rqt_image_view / RViz see /wrist_camera/image_raw.
    gz_image_topic = "/image"
    wrist_camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[gz_image_topic],
        remappings=[(gz_image_topic, "/wrist_camera/image_raw")],
        parameters=[{"use_sim_time": use_sim_time, "qos": "default"}],
        output="screen",
    )

    # External camera image bridge (VLA-style third-person view).
    # gz-sim scopes sensor topics with world name: /world/default/model/...
    gz_external_image_topic = "/world/default/model/external_camera/link/link/sensor/camera/image"
    external_camera_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[gz_external_image_topic],
        remappings=[(gz_external_image_topic, "/external_camera/image_raw")],
        parameters=[{"use_sim_time": use_sim_time, "qos": "default"}],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value=use_sim_time, description="Use sim time"),
        gz_sim_launch,
        clock_bridge,
        wrist_camera_image_bridge,
        external_camera_image_bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner],
            )
        ),
    ])
