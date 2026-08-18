#!/usr/bin/env python3
"""MuJoCo sensor/TF side of the nvblox 3D ESDF simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start MuJoCo, localization, SLAM and the fixed D455 RGB-D camera."""
    share = get_package_share_directory("tracer_jaka_mujoco")
    slam_launch = os.path.join(share, "launch", "slam_sim.launch.py")
    default_model = os.path.join(share, "models", "scene.xml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "viewer",
            default_value="false",
            description="Open the MuJoCo viewer.",
        ),
        DeclareLaunchArgument(
            "camera_rate",
            default_value="30.0",
            description="RGB-D rate. Lower to 15 if offscreen rendering is slow.",
        ),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("model", default_value=default_model),
        DeclareLaunchArgument("init_keyframe", default_value="home"),
        DeclareLaunchArgument(
            "rviz",
            default_value="false",
            description="Start the SLAM RViz. nvblox has a separate ESDF RViz.",
        ),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="20",
            description="ROS domain shared with the Isaac ROS container.",
        ),
        DeclareLaunchArgument(
            "rmw_implementation",
            default_value="rmw_fastrtps_cpp",
            description="DDS implementation shared with Isaac ROS.",
        ),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            LaunchConfiguration("rmw_implementation"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "camera": "true",
                "camera_rate": LaunchConfiguration("camera_rate"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "model": LaunchConfiguration("model"),
                "init_keyframe": LaunchConfiguration("init_keyframe"),
                "rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
