#!/usr/bin/env python3
"""Launch the generic Task -> Whole-body Plan -> Execution-contract preview."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    planner_share = get_package_share_directory('ta_wbmp')
    robot_share = get_package_share_directory('tracer_jaka_description')
    scenario = LaunchConfiguration('scenario').perform(context)
    tasks = {
        'table': 'table_wipe.yaml',
        'blackboard': 'blackboard_wipe.yaml',
        'ras': 'ras_drawing.yaml',
    }
    if scenario not in tasks:
        raise RuntimeError(
            f"Unknown scenario '{scenario}', expected one of {sorted(tasks)}")
    planner = Node(
        package='ta_wbmp',
        executable='ta_wbmp_demo_node',
        name='task_pipeline',
        output='screen',
        parameters=[{
            'task_file': os.path.join(
                planner_share, 'config', tasks[scenario]),
            'urdf_file': os.path.join(
                robot_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'ee_frame': 'tool0',
            'publish_delay': 0.5,
            'robot_snapshots': 0,
            'time_segment_duration': 15.0,
            'segment_snapshots': 2,
            'playback_enabled': True,
            'playback_rate': 5.0,
            'playback_period': 0.10,
            'playback_loop': True,
            # These are namespaced contracts, not live actuator topics.
            'publish_execution_interfaces': True,
        }],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='task_pipeline_rviz',
        arguments=['-d', os.path.join(planner_share, 'rviz', 'demo.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )
    return [planner, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='blackboard',
            description='Task scenario: table, blackboard or ras.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the task pipeline preview.'),
        OpaqueFunction(function=_nodes),
    ])
