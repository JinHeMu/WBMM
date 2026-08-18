#!/usr/bin/env python3
"""Run REMANI+OCS2 in the scene previously mapped by nvblox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ocs2_share = get_package_share_directory('tracer_jaka_ocs2')
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    validation_launch = os.path.join(
        ocs2_share, 'launch', 'ocs2_esdf_validation.launch.py')
    demo_scene = os.path.join(
        mujoco_share, 'models', 'scene_nvblox_remani_demo.xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_goal', default_value='true',
            description='Automatically send the demo goal after startup.'),
        # Stable default: the base must detour around the central crate while
        # the arm remains close to its initial posture.  The optional
        # goal_x:=3.8 goal_y:=0.0 challenge additionally passes the low portal
        # and therefore exercises whole-body arm folding.
        DeclareLaunchArgument('goal_x', default_value='2.3'),
        DeclareLaunchArgument('goal_y', default_value='-1.1'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=EnvironmentVariable(
                'NVBLOX_OUTPUT_DIR',
                default_value='/home/a/workspaces/isaac_ros-dev/bag_export'),
            description='Host directory containing exported nvblox maps.'),
        DeclareLaunchArgument(
            'esdf_file',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'mujoco_demo_remani_esdf.npz',
            ])),
        DeclareLaunchArgument(
            'map2d_yaml',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'mujoco_demo_2d.yaml',
            ])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(validation_launch),
            launch_arguments={
                'mujoco_model': demo_scene,
                'esdf_file': LaunchConfiguration('esdf_file'),
                'map2d_yaml': LaunchConfiguration('map2d_yaml'),
                'viewer': LaunchConfiguration('viewer'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'esdf_display_distance': '0.45',
                'esdf_display_stride': '2',
                # MuJoCo joint position actuators track this conservative
                # profile without tripping the 0.5 rad MRT safety gate.
                'remani_manipulator_max_vel': '0.35',
                'remani_manipulator_max_acc': '0.70',
                'remani_freeze_manipulator': 'true',
            }.items(),
        ),
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='tracer_jaka_mujoco',
                    executable='demo_goal_publisher',
                    name='mujoco_esdf_demo_goal',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'goal_x': ParameterValue(
                            LaunchConfiguration('goal_x'), value_type=float),
                        'goal_y': ParameterValue(
                            LaunchConfiguration('goal_y'), value_type=float),
                        'goal_yaw': ParameterValue(
                            LaunchConfiguration('goal_yaw'), value_type=float),
                        'frame_id': 'odom',
                    }],
                    condition=IfCondition(LaunchConfiguration('auto_goal')),
                ),
            ],
        ),
    ])
