#!/usr/bin/env python3
"""Launch the cylindrical full-coverage TA-WBMP demonstration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    planner_share = get_package_share_directory('ta_wbmp')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(planner_share, 'launch', 'demo.launch.py')),
            launch_arguments={
                'task_file': os.path.join(
                    planner_share, 'config', 'curved_wipe_demo.yaml'),
            }.items(),
        ),
    ])
