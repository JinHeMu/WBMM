#!/usr/bin/env python3
"""Physical task-table loop: REMANI -> OCS2/MRT -> MuJoCo -> feedback."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    wipe_share = get_package_share_directory('wipe_planner')
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    grid_map_share = get_package_share_directory('grid_map')
    bringup_share = get_package_share_directory('tracer_jaka_bringup')

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'wipe_sim.launch.py')),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'start_slam': 'false',
            'auto_goal': LaunchConfiguration('auto_goal'),
            'wipe_task_file': LaunchConfiguration('wipe_task_file'),
            'mujoco_model': LaunchConfiguration('mujoco_model'),
            'remani_static_esdf_file': LaunchConfiguration(
                'remani_static_esdf_file'),
            # The physical pad has 80 mm clearance at q_pre. The static ESDF
            # retains the physical 45-mm tool sphere and adds 20 mm clearance;
            # this is not an RViz-only or collision-disabled run.
            'remani_manipulator_safe_margin': '0.02',
            'remani_general_safe_margin': '0.02',
            'remani_self_safe_margin': '0.02',
            'remani_max_consecutive_planning_failures': '20',
            'remani_goal_replan_position_threshold': LaunchConfiguration(
                'remani_goal_replan_position_threshold'),
            'remani_goal_replan_yaw_threshold': LaunchConfiguration(
                'remani_goal_replan_yaw_threshold'),
            'remani_goal_replan_joint_threshold': LaunchConfiguration(
                'remani_goal_replan_joint_threshold'),
            'remani_goal_replan_persistence': LaunchConfiguration(
                'remani_goal_replan_persistence'),
            'force_control_enabled': 'false',
            # TASK_EXEC ownership is forbidden until measured base pose and all
            # six measured joints reach and hold q_pre.
            'navigation_partial_handoff_enabled': 'false',
            'navigation_whole_body_squared_tolerance': '0.02',
            'navigation_arrival_hold': '0.50',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument(
            'remani_goal_replan_position_threshold', default_value='0.40'),
        DeclareLaunchArgument(
            'remani_goal_replan_yaw_threshold', default_value='0.70'),
        DeclareLaunchArgument(
            'remani_goal_replan_joint_threshold', default_value='0.45'),
        DeclareLaunchArgument(
            'remani_goal_replan_persistence', default_value='1.00'),
        DeclareLaunchArgument(
            'wipe_task_file',
            default_value=os.path.join(
                wipe_share, 'config', 'wipe_task_mujoco_table.yaml')),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=os.path.join(
                mujoco_share, 'models', 'scene_task_table.xml')),
        DeclareLaunchArgument(
            'remani_static_esdf_file',
            default_value=os.path.join(
                grid_map_share, 'maps', 'task_table_esdf.npz'),
            description=(
                'Static ESDF generated from exactly scene_task_table.xml.')),
        pipeline,
    ])
