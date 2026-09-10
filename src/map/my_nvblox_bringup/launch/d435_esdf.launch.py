#!/usr/bin/env python3
"""Consume a host-published arm-mounted D435 and build a 3D ESDF."""

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
    """Map D435 RGB-D streams; the camera driver runs on the host."""
    share = get_package_share_directory('my_nvblox_bringup')
    core_launch = os.path.join(share, 'launch', 'nvblox_core.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('ros_domain_id', default_value='20'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/d435i/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/d435i/depth/camera_info'),
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/d435i/color/image_raw'),
        DeclareLaunchArgument(
            'color_camera_info_topic',
            default_value='/camera/d435i/color/camera_info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'use_sim_time': 'false',
                'global_frame': LaunchConfiguration('global_frame'),
                'map_clearing_frame_id': 'base_footprint',
                'input_qos': 'SENSOR_DATA',
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'depth_camera_info_topic':
                    LaunchConfiguration('depth_camera_info_topic'),
                'color_image_topic': LaunchConfiguration('color_image_topic'),
                'color_camera_info_topic':
                    LaunchConfiguration('color_camera_info_topic'),
                'use_color': 'true',
                'use_lidar': 'false',
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
