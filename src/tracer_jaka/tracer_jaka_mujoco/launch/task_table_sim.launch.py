#!/usr/bin/env python3
"""Start the dedicated polishing/machining task-table MuJoCo scene."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch the normal bridge with the task-table scene selected."""
    share = get_package_share_directory('tracer_jaka_mujoco')
    bridge_launch = os.path.join(share, 'launch', 'bridge.launch.py')
    scene = os.path.join(share, 'models', 'scene_task_table.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Whether to open the MuJoCo viewer.'),
        DeclareLaunchArgument(
            'camera', default_value='false',
            description='Whether to publish the fixed D455 RGB-D streams.'),
        DeclareLaunchArgument(
            'camera_rate', default_value='30.0',
            description='RGB-D publication rate in Hz.'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument(
            'init_keyframe', default_value='home',
            description='Use task_contact for the built-in contact demo.'),
        DeclareLaunchArgument(
            'fts_zero_on_start', default_value='true',
            description=(
                'Use false with task_contact so its contact load is visible.')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                'model': scene,
                'viewer': LaunchConfiguration('viewer'),
                'camera': LaunchConfiguration('camera'),
                'camera_rate': LaunchConfiguration('camera_rate'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'init_keyframe': LaunchConfiguration('init_keyframe'),
                'fts_enable': 'true',
                'fts_zero_on_start': LaunchConfiguration('fts_zero_on_start'),
            }.items(),
        ),
    ])
