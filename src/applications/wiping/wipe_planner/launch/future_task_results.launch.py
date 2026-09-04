#!/usr/bin/env python3
"""Show the 50-entry future-task experiment in RViz without robot control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wipe_share = get_package_share_directory('wipe_planner')
    mujoco_share = get_package_share_directory('tracer_jaka_description')
    use_rviz = LaunchConfiguration('use_rviz')
    results_csv = LaunchConfiguration('results_csv')

    preview = Node(
        package='wipe_planner',
        executable='wipe_plan_preview_node',
        name='future_task_results',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'world_frame': 'odom',
            'ee_frame': 'tool0',
            'urdf_file': os.path.join(
                mujoco_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'task_file': os.path.join(
                wipe_share, 'config', 'wipe_task.yaml'),
            'initial_state': [
                0.75, 2.06, 0.0,
                0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398,
            ],
            'coverage_snapshots': 0,
            'show_plan_summary': False,
            'future_task_results_csv': results_csv,
        }],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='future_task_results_rviz',
        arguments=['-d', os.path.join(
            wipe_share, 'rviz', 'future_task_results.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open the dedicated future-task result view.'),
        DeclareLaunchArgument(
            'results_csv',
            default_value='',
            description='Absolute path to wipe_future_task_batch rollouts.csv.'),
        preview,
        rviz,
    ])
