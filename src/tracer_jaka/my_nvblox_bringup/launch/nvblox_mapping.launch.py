#!/usr/bin/env python3
"""Backward-compatible alias for nvblox_core.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Forward legacy users to the sensor-independent core launch."""
    share = get_package_share_directory('my_nvblox_bringup')
    core_launch = os.path.join(share, 'launch', 'nvblox_core.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
        ),
    ])
