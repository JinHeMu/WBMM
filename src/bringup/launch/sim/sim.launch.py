#!/usr/bin/env python3
"""Pure MuJoCo simulation bringup.

Starts only the MuJoCo bridge and its robot_state_publisher. No localization,
navigation, OCS2, REMANI, RViz, or real hardware is started here.

Sensor rates, camera resolution and F/T filtering remain in
``tracer_jaka_mujoco/config/sensors.yaml``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mujoco_share = FindPackageShare("tracer_jaka_mujoco")

    return LaunchDescription([
        DeclareLaunchArgument(
            "viewer", default_value="true",
            description="Open the MuJoCo native viewer."),
        DeclareLaunchArgument(
            "camera", default_value="false",
            description="Publish simulated D455 RGB-D streams."),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("fts_enable", default_value="true"),
        DeclareLaunchArgument("fts_zero_on_start", default_value="true"),
        DeclareLaunchArgument(
            "model",
            default_value=PathJoinSubstitution([
                mujoco_share, "models", "scene.xml",
            ]),
            description="MuJoCo scene XML."),
        DeclareLaunchArgument(
            "init_keyframe", default_value="home",
            description="MuJoCo keyframe used for the initial state."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                mujoco_share, "launch", "bridge.launch.py",
            ])),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "camera": LaunchConfiguration("camera"),
                "camera_rate": LaunchConfiguration("camera_rate"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "fts_enable": LaunchConfiguration("fts_enable"),
                "fts_zero_on_start": LaunchConfiguration("fts_zero_on_start"),
                "model": LaunchConfiguration("model"),
                "init_keyframe": LaunchConfiguration("init_keyframe"),
            }.items(),
        ),
    ])
