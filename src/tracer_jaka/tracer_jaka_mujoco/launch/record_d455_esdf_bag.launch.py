#!/usr/bin/env python3
"""Record the minimum real-robot data needed for offline D455 ESDF mapping."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Record depth, calibration, localization TF, and diagnostic topics."""
    share = get_package_share_directory("tracer_jaka_mujoco")
    qos_file = os.path.join(share, "config", "d455_esdf_bag_qos.yaml")

    topics = [
        "/camera/d455/depth/image_rect_raw",
        "/camera/d455/depth/camera_info",
        "/tf",
        "/tf_static",
        "/odom",
        "/odometry/filtered",
        "/joint_states",
        "/map",
        "/map_metadata",
        "/scan",
        "/IMU_data",
    ]

    recorder = ExecuteProcess(
        cmd=[
            "ros2", "bag", "record",
            "--storage", "sqlite3",
            "--output", LaunchConfiguration("output"),
            "--max-cache-size", "268435456",
            "--compression-mode", "file",
            "--compression-format", "zstd",
            "--qos-profile-overrides-path", qos_file,
            *topics,
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "output",
            default_value="d455_esdf_bag",
            description=(
                "Output directory; choose a new name for every recording")),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "20"),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        recorder,
    ])
