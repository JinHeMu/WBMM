#!/usr/bin/env python3
"""Prepare the three-stage pipeline coordinator; live execution is opt-in."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    planner_share = get_package_share_directory('ta_wbmp')
    robot_share = get_package_share_directory('tracer_jaka_description')
    scenario = LaunchConfiguration('scenario').perform(context)
    tasks = {
        'table': 'table_wipe.yaml',
        'mujoco_table': 'mujoco_table_task.yaml',
        'blackboard': 'blackboard_wipe.yaml',
        'ras': 'ras_drawing.yaml',
    }
    task_file = LaunchConfiguration('task_file').perform(context)
    if not task_file and scenario not in tasks:
        raise RuntimeError(
            f"Unknown scenario '{scenario}', expected one of {sorted(tasks)}")
    if not task_file:
        task_file = os.path.join(planner_share, 'config', tasks[scenario])
    coordinator = Node(
        package='ta_wbmp',
        executable='ta_wbmp_execution_coordinator_node',
        name='task_execution_coordinator',
        output='screen',
        parameters=[{
            'task_file': task_file,
            'urdf_file': os.path.join(
                robot_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'ee_frame': 'tool0',
            'control_frame': LaunchConfiguration('control_frame'),
            'execution_enabled': ParameterValue(
                LaunchConfiguration('execution_enabled'), value_type=bool),
            'auto_start': ParameterValue(
                LaunchConfiguration('auto_start'), value_type=bool),
            'force_control_enabled': ParameterValue(
                LaunchConfiguration('force_control_enabled'), value_type=bool),
        }],
    )
    return [coordinator]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='blackboard',
            description='Task scenario: table, mujoco_table, blackboard or ras.'),
        DeclareLaunchArgument(
            'task_file', default_value='',
            description='Optional absolute unified task YAML; overrides scenario.'),
        DeclareLaunchArgument(
            'control_frame', default_value='odom',
            description='Frame used by the OCS2 9D observation/reference.'),
        DeclareLaunchArgument(
            'execution_enabled', default_value='false',
            description='Permit writes to live REMANI/OCS2 topics.'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Start immediately instead of waiting for the service.'),
        DeclareLaunchArgument(
            'force_control_enabled', default_value='false',
            description='Enable online force correction in the MPC executor.'),
        OpaqueFunction(function=_nodes),
    ])
