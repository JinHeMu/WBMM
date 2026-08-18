#!/usr/bin/env python3
"""Validate a saved real-scene nvblox ESDF in MuJoCo with REMANI+OCS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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
    sim_launch = os.path.join(ocs2_share, 'launch', 'ocs2_sim.launch.py')
    task_file = os.path.join(ocs2_share, 'config', 'task_esdf_only.info')
    default_scene_file = os.path.join(
        mujoco_share, 'models', 'scene_esdf_validation.xml')
    rviz_file = os.path.join(
        ocs2_share, 'rviz', 'tracer_jaka_esdf_validation.rviz')

    esdf_file = LaunchConfiguration('esdf_file')
    map2d_yaml = LaunchConfiguration('map2d_yaml')
    mujoco_model = LaunchConfiguration('mujoco_model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    simulator_and_planner = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'viewer': LaunchConfiguration('viewer'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': rviz_file,
            'use_joy': 'false',
            'start_slam': 'false',
            'start_remani': 'true',
            'task_file': task_file,
            'mujoco_model': mujoco_model,
            'map_to_odom_x': '0.0',
            'remani_static_esdf_file': esdf_file,
            'remani_static_esdf_offset_x': '0.0',
            'remani_static_esdf_offset_y': '0.0',
            'remani_static_esdf_offset_z': '0.0',
            'remani_manipulator_max_vel': LaunchConfiguration(
                'remani_manipulator_max_vel'),
            'remani_manipulator_max_acc': LaunchConfiguration(
                'remani_manipulator_max_acc'),
            'remani_freeze_manipulator': LaunchConfiguration(
                'remani_freeze_manipulator'),
        }.items(),
    )

    saved_esdf_visualization = Node(
        package='grid_map',
        executable='esdf_rviz_publisher',
        name='saved_esdf_rviz_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'esdf_file': ParameterValue(esdf_file, value_type=str),
            'frame_id': 'odom',
            'z_slice': -1.0,
            'max_distance': ParameterValue(
                LaunchConfiguration('esdf_display_distance'),
                value_type=float),
            'stride': ParameterValue(
                LaunchConfiguration('esdf_display_stride'),
                value_type=int),
            'publish_period': 1.0,
            'z_min_2d': 0.05,
            'z_max_2d': 0.60,
        }],
    )

    saved_2d_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='saved_2d_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'yaml_filename': ParameterValue(map2d_yaml, value_type=str),
            'topic_name': 'map',
            'frame_id': 'map',
        }],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='saved_map_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': True,
            'node_names': ['saved_2d_map_server'],
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'esdf_file',
            default_value=PathJoinSubstitution([
                EnvironmentVariable(
                    'NVBLOX_OUTPUT_DIR',
                    default_value=(
                        '/home/a/workspaces/isaac_ros-dev/bag_export')),
                'd455_bag_remani_esdf.npz',
            ]),
            description='REMANI-format NPZ exported from nvblox.'),
        DeclareLaunchArgument(
            'map2d_yaml',
            default_value='/home/a/WBMM/src/bringup/tracer_jaka_bringup/maps/factory_map.yaml',
            description='2D map saved by slam_toolbox.'),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=default_scene_file,
            description='MuJoCo scene used for ESDF planning validation.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'esdf_display_distance', default_value='0.5'),
        DeclareLaunchArgument(
            'esdf_display_stride', default_value='2'),
        DeclareLaunchArgument('remani_manipulator_max_vel', default_value='1.57'),
        DeclareLaunchArgument('remani_manipulator_max_acc', default_value='3.14'),
        DeclareLaunchArgument('remani_freeze_manipulator', default_value='false'),
        simulator_and_planner,
        saved_esdf_visualization,
        saved_2d_map_server,
        map_lifecycle_manager,
    ])
