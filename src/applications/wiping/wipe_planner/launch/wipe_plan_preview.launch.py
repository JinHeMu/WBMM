#!/usr/bin/env python3
"""Plan and visualize a complete wipe trajectory without starting any robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    wipe_share = get_package_share_directory('wipe_planner')
    mujoco_share = get_package_share_directory('tracer_jaka_description')

    use_rviz = LaunchConfiguration('use_rviz')
    coverage_snapshots = LaunchConfiguration('coverage_snapshots')

    preview = Node(
        package='wipe_planner',
        executable='wipe_plan_preview_node',
        name='wipe_plan_preview',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'world_frame': 'odom',
            'ee_frame': 'tool0',
            'urdf_file': os.path.join(
                mujoco_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'task_file': os.path.join(
                wipe_share, 'config', 'wipe_task.yaml'),
            # Planning seed only; no simulated or real robot starts.
            'initial_state': [
                0.75, 2.06, 0.0,
                0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398,
            ],
            'coverage_snapshots': ParameterValue(
                coverage_snapshots, value_type=int),
            'coverage_alpha': 0.16,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='wipe_plan_preview_rviz',
        arguments=['-d', os.path.join(
            wipe_share, 'rviz', 'wipe_plan_preview.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    # Blue board-region marker, re-read from wipe_task.yaml every second, so
    # the write surface shows in RViz without any extra terminal.
    board_marker = Node(
        package='grid_map',
        executable='board_marker',
        name='board_marker',
        output='log',
        arguments=[
            '--task-file', os.path.join(
                wipe_share, 'config', 'wipe_task.yaml'),
            '--frame', 'odom',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the dedicated WipePlanner RViz view.'),
        DeclareLaunchArgument(
            'coverage_snapshots', default_value='14',
            description='Number of REMANI-style whole-body trajectory ghosts.'),
        preview,
        rviz,
        board_marker,
    ])
