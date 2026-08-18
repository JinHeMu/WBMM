#!/usr/bin/env python3
"""Start the arm-mounted RealSense D435 with robot-compatible frame names."""

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
    """Publish D435 RGB-D streams rooted at the URDF's d435i_link."""
    realsense_launch = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_no", default_value="''"),
        DeclareLaunchArgument("camera_namespace", default_value="camera"),
        DeclareLaunchArgument("camera_name", default_value="d435i"),
        DeclareLaunchArgument("ros_domain_id", default_value="20"),
        DeclareLaunchArgument(
            "rmw_implementation", default_value="rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION",
            LaunchConfiguration("rmw_implementation"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                "serial_no": LaunchConfiguration("serial_no"),
                "camera_namespace": LaunchConfiguration("camera_namespace"),
                "camera_name": LaunchConfiguration("camera_name"),
                # camera_name=d435i + base_frame_id=link => d435i_link.
                "base_frame_id": "link",
                "enable_color": "true",
                "enable_depth": "true",
                "enable_sync": "true",
                "publish_tf": "true",
                "pointcloud.enable": "false",
                "align_depth.enable": "false",
                "depth_module.depth_profile": "640,480,30",
                "rgb_camera.color_profile": "640,480,30",
            }.items(),
        ),
    ])
