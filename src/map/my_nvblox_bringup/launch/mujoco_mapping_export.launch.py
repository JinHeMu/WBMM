#!/usr/bin/env python3
"""Build and export the MuJoCo demo ESDF for REMANI."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('my_nvblox_bringup')
    nvblox_launch = os.path.join(share, 'launch', 'mujoco_esdf.launch.py')

    exporter = Node(
        package='my_nvblox_bringup',
        executable='nvblox_map_exporter',
        name='mujoco_nvblox_map_exporter',
        output='screen',
        parameters=[{
            'trigger_topic': '/esdf_mapping_scan_done',
            'map_output': ParameterValue(
                LaunchConfiguration('map_output'), value_type=str),
            'esdf_output': ParameterValue(
                LaunchConfiguration('esdf_output'), value_type=str),
            'frame_id': 'odom',
            'esdf_min_x': -0.5,
            'esdf_min_y': -2.5,
            'esdf_min_z': -0.2,
            'esdf_size_x': 5.5,
            'esdf_size_y': 5.0,
            'esdf_size_z': 2.4,
            # The scripted camera pass observes the test corridor and all
            # obstacle surfaces, but a pinhole camera cannot observe every
            # voxel around the initial robot body. Keep unknown space free
            # for this repeatable simulation demo; real-bag export remains
            # conservative by default.
            'unknown_is_occupied': False,
        }],
    )

    shutdown_after_export = RegisterEventHandler(
        OnProcessExit(
            target_action=exporter,
            on_exit=[EmitEvent(event=Shutdown(
                reason='MuJoCo mapping export completed'))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument('ros_domain_id', default_value='20'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'output_dir',
            default_value=EnvironmentVariable(
                'NVBLOX_OUTPUT_DIR',
                default_value='/workspaces/isaac_ros-dev/bag_export'),
            description=(
                'Directory for generated maps; NVBLOX_OUTPUT_DIR may also '
                'set this default')),
        DeclareLaunchArgument(
            'map_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'mujoco_demo_map.nvblx',
            ])),
        DeclareLaunchArgument(
            'esdf_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'mujoco_demo_remani_esdf.npz',
            ])),
        DeclareLaunchArgument(
            'map2d_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'mujoco_demo_2d.yaml',
            ])),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nvblox_launch),
            launch_arguments={
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'rviz': LaunchConfiguration('rviz'),
                'voxel_size': '0.08',
                'map_clearing_radius_m': '-1.0',
                'esdf_viz_follow_robot': 'false',
                'esdf_viz_size_x': '5.5',
                'esdf_viz_size_y': '5.0',
                'esdf_viz_min_z': '-0.2',
                'esdf_viz_size_z': '2.4',
                'esdf_viz_rate': '0.5',
                'esdf_viz_subsampling': '2',
                'esdf_viz_max_distance': '0.5',
            }.items(),
        ),
        Node(
            package='my_nvblox_bringup',
            executable='map_snapshot_saver',
            name='mujoco_2d_map_snapshot_saver',
            output='screen',
            parameters=[{
                'map_topic': '/map',
                'output_yaml': ParameterValue(
                    LaunchConfiguration('map2d_output'), value_type=str),
            }],
        ),
        exporter,
        shutdown_after_export,
    ])
