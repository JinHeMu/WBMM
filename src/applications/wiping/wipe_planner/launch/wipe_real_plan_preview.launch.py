#!/usr/bin/env python3
"""Preview the front-board real task without starting any controller."""

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
    description_share = get_package_share_directory('tracer_jaka_description')
    use_rviz = LaunchConfiguration('use_rviz')
    task_file = LaunchConfiguration('wipe_task_file')
    urdf_file = LaunchConfiguration('urdf_file')
    world_frame = LaunchConfiguration('world_frame')

    # This launch is deliberately standalone: AMCL/SLAM is not running, so no
    # localization node exists to publish map -> odom.  The preview geometry is
    # expressed in map while its reusable RViz profile uses odom as the fixed
    # frame.  Publish an identity transform only inside preview mode so both
    # frame names refer to the same offline coordinate system.
    preview_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='wipe_preview_map_to_odom',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
            '--frame-id', 'map', '--child-frame-id', 'odom',
        ],
        condition=IfCondition(LaunchConfiguration(
            'publish_preview_map_to_odom')),
        output='screen',
    )

    preview = Node(
        package='wipe_planner',
        executable='wipe_plan_preview_node',
        name='wipe_real_plan_preview',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'world_frame': world_frame,
            'ee_frame': 'tool0',
            'urdf_file': urdf_file,
            'task_file': task_file,
            'initial_state': [
                0.0, 0.0, 0.0,
                0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398,
            ],
            'coverage_snapshots': ParameterValue(
                LaunchConfiguration('coverage_snapshots'), value_type=int),
            'coverage_alpha': 0.16,
        }],
    )

    board_marker = Node(
        package='grid_map',
        executable='board_marker',
        name='wipe_real_preview_board_marker',
        output='log',
        arguments=['--task-file', task_file, '--frame', world_frame],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='wipe_real_plan_preview_rviz',
        arguments=['-d', os.path.join(
            wipe_share, 'rviz', 'wipe_plan_preview.rviz')],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('coverage_snapshots', default_value='14'),
        DeclareLaunchArgument('world_frame', default_value='map'),
        DeclareLaunchArgument(
            'publish_preview_map_to_odom',
            default_value='true',
            description=(
                'Publish identity map->odom for this standalone offline '
                'preview. Disable it if an external localization stack is '
                'already publishing map->odom.')),
        DeclareLaunchArgument(
            'wipe_task_file',
            default_value=os.path.join(
                wipe_share, 'config', 'wipe_task_real_front.yaml')),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                description_share, 'urdf', 'tracer_jaka_zu5.urdf')),
        preview,
        preview_map_to_odom,
        board_marker,
        rviz,
    ])
