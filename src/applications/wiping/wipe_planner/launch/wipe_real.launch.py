#!/usr/bin/env python3
"""Start only the TF-aware real-robot WipePlanner reference owner.

This launch deliberately does not start MuJoCo, OCS2 or REMANI.  The caller
must run the localized real bringup with start_bridge:=false so exactly one
node owns /mobile_manipulator_mpc_target.
"""

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

    task_file = LaunchConfiguration('wipe_task_file')
    urdf_file = LaunchConfiguration('urdf_file')
    planner_frame = LaunchConfiguration('planner_frame')
    control_frame = LaunchConfiguration('control_frame')
    auto_goal = LaunchConfiguration('auto_goal')
    force_control = LaunchConfiguration('force_control_enabled')
    board_marker_enabled = LaunchConfiguration('board_marker_enabled')

    wipe_node = Node(
        package='wipe_planner',
        executable='wipe_planner_node',
        name='wipe_planner',
        output='screen',
        parameters=[
            os.path.join(wipe_share, 'config', 'wipe_planner.yaml'),
            {
                'use_sim_time': False,
                'world_frame': planner_frame,
                'planner_frame': planner_frame,
                'control_frame': control_frame,
                'use_tf_transform': True,
                'urdf_file': urdf_file,
                'task_file': task_file,
                'auto_navigation_goal': ParameterValue(
                    auto_goal, value_type=bool),
                'force_control_enabled': ParameterValue(
                    force_control, value_type=bool),
            },
        ],
    )

    board_marker = Node(
        package='grid_map',
        executable='board_marker',
        name='wipe_real_board_marker',
        output='log',
        arguments=['--task-file', task_file, '--frame', planner_frame],
        condition=IfCondition(board_marker_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'wipe_task_file',
            default_value=os.path.join(
                wipe_share, 'config', 'wipe_task_real_front.yaml')),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                description_share, 'urdf', 'tracer_jaka_zu5.urdf')),
        DeclareLaunchArgument('planner_frame', default_value='map'),
        DeclareLaunchArgument('control_frame', default_value='odom'),
        DeclareLaunchArgument(
            'auto_goal', default_value='false',
            description='Automatically send the planned 9D pre-contact goal.'),
        DeclareLaunchArgument(
            'force_control_enabled', default_value='false',
            description='Keep false for pure geometric tracking.'),
        DeclareLaunchArgument('board_marker_enabled', default_value='true'),
        wipe_node,
        board_marker,
    ])
