#!/usr/bin/env python3
"""Compatibility entry point for MoveIt Servo on either backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    backend = LaunchConfiguration("backend")
    use_servo = LaunchConfiguration("use_servo")
    use_joy = LaunchConfiguration("use_joy")
    unified_launch = os.path.join(
        get_package_share_directory("tracer_jaka_bringup"),
        "launch", "moveit.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument(
            "backend", default_value="sim",
            description="Servo backend: sim (MuJoCo) or real (JAKA)."),
        DeclareLaunchArgument(
            "use_servo", default_value="true",
            description="Compatibility default; starts MoveIt Servo."),
        DeclareLaunchArgument(
            "use_joy", default_value="true",
            description="Compatibility default; starts joystick conversion."),
        LogInfo(msg=(
            "servo.launch.py is a compatibility entry point; prefer "
            "moveit.launch.py use_servo:=true use_joy:=true."
        )),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(unified_launch),
            launch_arguments={
                "backend": backend,
                "use_servo": use_servo,
                "use_joy": use_joy,
            }.items(),
        ),
    ])
