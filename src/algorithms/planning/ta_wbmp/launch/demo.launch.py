#!/usr/bin/env python3
"""Launch the TA-WBMP planning-only demo, its unified /wbmm/* visualization,
and the dedicated RViz view."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    planner_share = get_package_share_directory('ta_wbmp')
    robot_share = get_package_share_directory('tracer_jaka_description')
    robot_urdf = os.path.join(
        robot_share, 'urdf', 'tracer_jaka_zu5.urdf')
    use_rviz = LaunchConfiguration('use_rviz')
    task_file = LaunchConfiguration('task_file')

    planner = Node(
        package='ta_wbmp',
        executable='ta_wbmp_demo_node',
        name='ta_wbmp_demo',
        output='screen',
        parameters=[{
            'task_file': task_file,
            'urdf_file': robot_urdf,
            'ee_frame': 'tool0',
            'publish_delay': 0.5,
        }],
    )

    # Unified visualization: the planner only publishes the data contract
    # (9D JointTrajectory + phase schedule + live phase); wbmm_visualization
    # renders robot mesh, time segments and the playback animation.
    viz = Node(
        package='wbmm_visualization',
        executable='wbmm_viz_node',
        name='wbmm_viz',
        output='screen',
        parameters=[{
            'urdf_file': robot_urdf,
            'ee_frame': 'tool0',
            # Time-segment snapshots replace the old all-trajectory ghosts.
            'time_segment_duration': 15.0,
            'segment_snapshots': 2,
            # Play the ~127 s nominal plan at 5x wall time and loop in RViz.
            'playback_enabled': True,
            'playback_rate': 5.0,
            'playback_period': 0.10,
            'playback_loop': True,
        }],
        remappings=[
            ('/wbmm/whole_body_trajectory', '/ta_wbmp/whole_body_trajectory'),
            ('/wbmm/phase_schedule', '/ta_wbmp/phase_schedule'),
            ('/wbmm/live_phase', '/ta_wbmp/phases'),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='ta_wbmp_rviz',
        arguments=['-d', os.path.join(planner_share, 'rviz', 'demo.rviz')],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the TA-WBMP planning preview.'),
        DeclareLaunchArgument(
            'task_file',
            default_value=os.path.join(
                planner_share, 'config', 'wipe_demo.yaml'),
            description='TA-WBMP task YAML file.'),
        planner,
        viz,
        rviz,
    ])
