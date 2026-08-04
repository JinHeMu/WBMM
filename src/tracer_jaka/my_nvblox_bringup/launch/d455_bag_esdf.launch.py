#!/usr/bin/env python3
"""Build a D455 ESDF offline by replaying a NUC-recorded ROS bag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start nvblox first, then replay depth and TF with simulated time."""
    share = get_package_share_directory("my_nvblox_bringup")
    d455_launch = os.path.join(share, "launch", "d455_esdf.launch.py")
    qos_file = os.path.join(share, "config", "d455_esdf_bag_qos.yaml")

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(d455_launch),
        launch_arguments={
            "use_sim_time": "true",
            "use_color": LaunchConfiguration("use_color"),
            "global_frame": "odom",
            "rviz": LaunchConfiguration("rviz"),
            "ros_domain_id": LaunchConfiguration("ros_domain_id"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "map_clearing_radius_m": "-1.0",
            "esdf_viz_follow_robot": "false",
            "esdf_viz_size_x": LaunchConfiguration("esdf_viz_size_x"),
            "esdf_viz_size_y": LaunchConfiguration("esdf_viz_size_y"),
            "esdf_viz_rate": LaunchConfiguration("esdf_viz_rate"),
            "esdf_viz_subsampling":
                LaunchConfiguration("esdf_viz_subsampling"),
            "esdf_viz_max_distance":
                LaunchConfiguration("esdf_viz_max_distance"),
        }.items(),
    )

    player_process = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", LaunchConfiguration("bag"),
            "--storage", "sqlite3",
            "--clock", "100.0",
            "--rate", LaunchConfiguration("rate"),
            "--read-ahead-queue-size", "2000",
            "--qos-profile-overrides-path", qos_file,
            "--disable-keyboard-controls",
        ],
        output="screen",
    )
    player = TimerAction(
        period=5.0,
        actions=[player_process],
    )

    map_snapshot = Node(
        package='my_nvblox_bringup',
        executable='map_snapshot_saver',
        name='bag_map_snapshot_saver',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_yaml': LaunchConfiguration('map2d_output'),
        }],
    )

    export_after_playback = RegisterEventHandler(
        OnProcessExit(
            target_action=player_process,
            on_exit=[
                TimerAction(
                    period=3.0,
                    actions=[Node(
                        package='my_nvblox_bringup',
                        executable='nvblox_map_exporter',
                        name='nvblox_map_exporter',
                        output='screen',
                        parameters=[{
                            'use_sim_time': False,
                            'map_output': LaunchConfiguration('map_output'),
                            'esdf_output': LaunchConfiguration('esdf_output'),
                            'frame_id': 'odom',
                            'esdf_min_x': ParameterValue(
                                LaunchConfiguration('esdf_min_x'),
                                value_type=float),
                            'esdf_min_y': ParameterValue(
                                LaunchConfiguration('esdf_min_y'),
                                value_type=float),
                            'esdf_min_z': ParameterValue(
                                LaunchConfiguration('esdf_min_z'),
                                value_type=float),
                            'esdf_size_x': ParameterValue(
                                LaunchConfiguration('esdf_size_x'),
                                value_type=float),
                            'esdf_size_y': ParameterValue(
                                LaunchConfiguration('esdf_size_y'),
                                value_type=float),
                            'esdf_size_z': ParameterValue(
                                LaunchConfiguration('esdf_size_z'),
                                value_type=float),
                            'unknown_is_occupied': ParameterValue(
                                LaunchConfiguration('unknown_is_occupied'),
                                value_type=bool),
                        }],
                    )],
                ),
            ],
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Absolute path to the ROS bag directory"),
        DeclareLaunchArgument(
            "rate",
            default_value="0.5",
            description="Playback rate; 0.5 turns a 30 Hz stream into 15 Hz"),
        DeclareLaunchArgument(
            "ros_domain_id",
            default_value="21",
            description="Offline-only domain isolated from the live robot"),
        DeclareLaunchArgument(
            "use_color",
            default_value="true",
            description="Fuse recorded RGB into the nvblox mesh"),
        DeclareLaunchArgument(
            "voxel_size",
            default_value="0.10",
            description=(
                "Offline persistent-map voxel size; 10 cm is safe for "
                "4 GB GPUs")),
        DeclareLaunchArgument("rviz", default_value="true"),
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
                'd455_bag_map.nvblx',
            ])),
        DeclareLaunchArgument(
            'esdf_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'd455_bag_remani_esdf.npz',
            ])),
        DeclareLaunchArgument(
            'map2d_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'd455_bag_2d.yaml',
            ])),
        DeclareLaunchArgument('esdf_min_x', default_value='-6.0'),
        DeclareLaunchArgument('esdf_min_y', default_value='-6.0'),
        DeclareLaunchArgument('esdf_min_z', default_value='-0.2'),
        DeclareLaunchArgument('esdf_size_x', default_value='12.0'),
        DeclareLaunchArgument('esdf_size_y', default_value='12.0'),
        DeclareLaunchArgument('esdf_size_z', default_value='3.0'),
        DeclareLaunchArgument(
            'unknown_is_occupied',
            default_value='true',
            description='Treat unobserved ESDF voxels as collision space'),
        DeclareLaunchArgument("esdf_viz_size_x", default_value="20.0"),
        DeclareLaunchArgument("esdf_viz_size_y", default_value="20.0"),
        DeclareLaunchArgument("esdf_viz_rate", default_value="0.2"),
        DeclareLaunchArgument("esdf_viz_subsampling", default_value="3"),
        DeclareLaunchArgument(
            "esdf_viz_max_distance",
            default_value="0.5",
            description=(
                "Only show ESDF voxels within this distance of obstacles")),
        SetEnvironmentVariable(
            "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")),
        SetEnvironmentVariable(
            "RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        nvblox,
        map_snapshot,
        player,
        export_after_playback,
    ])
