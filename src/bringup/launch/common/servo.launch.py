#!/usr/bin/env python3
"""Convenience entry point for the shared MoveIt Servo algorithm nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_launch = os.path.join(
        get_package_share_directory("tracer_jaka_bringup"),
        "launch", "moveit.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_joy", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit_launch),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_servo": "true",
                "use_joy": LaunchConfiguration("use_joy"),
                "hardware_write": LaunchConfiguration("hardware_write"),
            }.items(),
        ),
    ])
