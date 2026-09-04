#!/usr/bin/env python3
"""Animated NAVIGATE-to-TASK table-wiping plan with time-segment snapshots."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    planner_share = get_package_share_directory('ta_wbmp')
    robot_share = get_package_share_directory('tracer_jaka_description')
    use_rviz = LaunchConfiguration('use_rviz')
    snapshots = LaunchConfiguration('snapshots')
    task_config = LaunchConfiguration('task_config')
    playback_rate = LaunchConfiguration('playback_rate')
    playback_loop = LaunchConfiguration('playback_loop')

    planner = Node(
        package='ta_wbmp',
        executable='ta_wbmp_demo_node',
        name='table_rviz_reproduction',
        output='screen',
        parameters=[{
            'task_file': PathJoinSubstitution([
                FindPackageShare('ta_wbmp'), 'config', task_config]),
            'urdf_file': os.path.join(
                robot_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'ee_frame': 'tool0',
            'publish_delay': 0.5,
            'show_table_fixture': True,
            'publish_execution_interfaces': False,
        }],
    )
    viz = Node(
        package='wbmm_visualization',
        executable='wbmm_viz_node',
        name='wbmm_viz',
        output='screen',
        parameters=[{
            'urdf_file': os.path.join(
                robot_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'ee_frame': 'tool0',
            'time_segment_duration': 15.0,
            'segment_snapshots': ParameterValue(snapshots, value_type=int),
            # The opaque current robot traverses every nominal phase while the
            # translucent time-segment snapshots keep the plan visible.
            'playback_enabled': True,
            'playback_rate': ParameterValue(playback_rate, value_type=float),
            'playback_period': 0.10,
            'playback_loop': ParameterValue(playback_loop, value_type=bool),
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
        name='table_rviz_reproduction_rviz',
        arguments=['-d', os.path.join(
            planner_share, 'rviz', 'table_rviz_reproduction.rviz')],
        condition=IfCondition(use_rviz),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the reference-style RViz view.'),
        DeclareLaunchArgument(
            'snapshots', default_value='2',
            description='Translucent whole-body snapshots per time segment.'),
        DeclareLaunchArgument(
            'playback_rate', default_value='2.0',
            description='Nominal trajectory seconds advanced per real second.'),
        DeclareLaunchArgument(
            'playback_loop', default_value='true',
            description='Replay NAVIGATE through TASK after reaching the end.'),
        DeclareLaunchArgument(
            'task_config', default_value='table_wipe.yaml',
            description=(
                'Task YAML. Use wipe_demo.yaml to match the large raster '
                'composition in the reference screenshot.')),
        planner,
        viz,
        rviz,
    ])
