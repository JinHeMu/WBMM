#!/usr/bin/env python3
"""Compatibility entry point for the former jaka_driver MoveIt launch."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    unified_launch = os.path.join(
        get_package_share_directory("tracer_jaka_bringup"),
        "launch", "moveit.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="real",
            description="Compatibility default; use moveit.launch.py directly."),
        LogInfo(msg=(
            "moveit_real.launch.py is a compatibility entry point; prefer "
            "moveit.launch.py backend:=real."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified_launch),
            launch_arguments={"backend": backend}.items(),
        ),
    ])
