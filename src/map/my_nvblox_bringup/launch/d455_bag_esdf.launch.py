#!/usr/bin/env python3
"""Build a D455 ESDF offline by replaying a NUC-recorded ROS bag."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    RegisterEventHandler,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
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
    """Start nvblox first, then replay depth and TF with simulated time."""
    share = get_package_share_directory("my_nvblox_bringup")
    d455_launch = os.path.join(share, "launch", "d455_esdf.launch.py")
    qos_file = os.path.join(share, "config", "d455_esdf_bag_qos.yaml")
    global_frame = LaunchConfiguration("global_frame")

    nvblox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(d455_launch),
        launch_arguments={
            "use_sim_time": "true",
            "use_color": LaunchConfiguration("use_color"),
            "global_frame": global_frame,
            "rviz": LaunchConfiguration("rviz"),
            "ros_domain_id": LaunchConfiguration("ros_domain_id"),
            "voxel_size": LaunchConfiguration("voxel_size"),
            "map_clearing_radius_m": "-1.0",
            "input_qos": "DEFAULT",
            "maximum_input_queue_length":
                LaunchConfiguration("maximum_input_queue_length"),
            "tick_period_ms": LaunchConfiguration("tick_period_ms"),
            "integrate_depth_rate_hz":
                LaunchConfiguration("integrate_depth_rate_hz"),
            "decay_tsdf_rate_hz":
                LaunchConfiguration("decay_tsdf_rate_hz"),
            "update_mesh_rate_hz":
                LaunchConfiguration("update_mesh_rate_hz"),
            "update_esdf_rate_hz":
                LaunchConfiguration("update_esdf_rate_hz"),
            "publish_layer_rate_hz":
                LaunchConfiguration("publish_layer_rate_hz"),
            "publish_debug_vis_rate_hz":
                LaunchConfiguration("publish_debug_vis_rate_hz"),
            "esdf_viz": LaunchConfiguration("esdf_viz"),
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

    exporter = Node(
        package='my_nvblox_bringup',
        executable='nvblox_map_exporter',
        name='nvblox_map_exporter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'trigger_topic': LaunchConfiguration('trigger_topic'),
            'service_call_timeout_sec': ParameterValue(
                LaunchConfiguration('service_call_timeout_sec'),
                value_type=float),
            'map_output': LaunchConfiguration('map_output'),
            'ply_output': LaunchConfiguration('ply_output'),
            'timings_output': LaunchConfiguration('timings_output'),
            'bag_path': LaunchConfiguration('bag'),
            'depth_topic': '/camera/d455/depth/image_rect_raw',
            'esdf_output': LaunchConfiguration('esdf_output'),
            'drain_timeout_sec': ParameterValue(
                LaunchConfiguration('drain_timeout_sec'), value_type=float),
            'drain_stable_polls': ParameterValue(
                LaunchConfiguration('drain_stable_polls'), value_type=int),
            'drain_max_pending_frames': ParameterValue(
                LaunchConfiguration('drain_max_pending_frames'),
                value_type=int),
            'esdf_use_aabb': ParameterValue(
                LaunchConfiguration('esdf_use_aabb'), value_type=bool),
            'require_all_depth_integrated': True,
            'frame_id': global_frame,
            'esdf_min_x': ParameterValue(
                LaunchConfiguration('esdf_min_x'), value_type=float),
            'esdf_min_y': ParameterValue(
                LaunchConfiguration('esdf_min_y'), value_type=float),
            'esdf_min_z': ParameterValue(
                LaunchConfiguration('esdf_min_z'), value_type=float),
            'esdf_size_x': ParameterValue(
                LaunchConfiguration('esdf_size_x'), value_type=float),
            'esdf_size_y': ParameterValue(
                LaunchConfiguration('esdf_size_y'), value_type=float),
            'esdf_size_z': ParameterValue(
                LaunchConfiguration('esdf_size_z'), value_type=float),
            'unknown_is_occupied': ParameterValue(
                LaunchConfiguration('unknown_is_occupied'), value_type=bool),
        }],
    )

    export_trigger = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--once',
            '--qos-reliability', 'reliable',
            '--qos-durability', 'transient_local',
            LaunchConfiguration('trigger_topic'),
            'std_msgs/msg/Bool', '{data: true}',
        ],
        output='screen',
    )

    export_after_playback = RegisterEventHandler(
        OnProcessExit(
            target_action=player_process,
            on_exit=[
                TimerAction(
                    period=LaunchConfiguration('export_settle_time'),
                    actions=[export_trigger],
                ),
            ],
        ),
    )
    shutdown_after_export = RegisterEventHandler(
        OnProcessExit(
            target_action=exporter,
            on_exit=[EmitEvent(event=Shutdown(
                reason='Offline nvblox export finished'))],
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
            description="Absolute path to the ROS bag directory"),
        DeclareLaunchArgument(
            "rate",
            default_value="0.25",
            description="Playback rate; slow replay prevents offline drops"),
        DeclareLaunchArgument(
            'maximum_input_queue_length',
            default_value='500',
            description='Large offline TF/sensor queue; prevents silent loss'),
        DeclareLaunchArgument('tick_period_ms', default_value='5'),
        DeclareLaunchArgument(
            'integrate_depth_rate_hz',
            default_value='1000.0',
            description='Offline mode integrates every unique depth stamp'),
        DeclareLaunchArgument(
            'decay_tsdf_rate_hz',
            default_value='0.0',
            description=(
                '0.0 disables TSDF decay so the offline map stays complete; '
                'decay erases everything not re-observed within ~30 s')),
        DeclareLaunchArgument(
            'update_mesh_rate_hz',
            default_value='0.5',
            description='Final save_ply forces a complete mesh update'),
        DeclareLaunchArgument(
            'update_esdf_rate_hz',
            default_value='1.0',
            description='Final ESDF service forces a complete update'),
        DeclareLaunchArgument(
            'publish_layer_rate_hz', default_value='0.5'),
        DeclareLaunchArgument(
            'publish_debug_vis_rate_hz', default_value='0.5'),
        DeclareLaunchArgument(
            'export_settle_time',
            default_value='5.0',
            description=(
                'Wall-time pause after playback before publishing the export '
                'trigger; the exporter itself starts before playback and '
                'discovers nvblox services during the replay')),
        DeclareLaunchArgument(
            'trigger_topic',
            default_value='/nvblox_export_trigger',
            description='Topic that releases the pre-started map exporter'),
        DeclareLaunchArgument(
            'service_call_timeout_sec',
            default_value='300.0',
            description='Maximum time for one nvblox export service call'),
        DeclareLaunchArgument(
            'drain_timeout_sec',
            default_value='120.0',
            description='Abort instead of exporting if input is not drained'),
        DeclareLaunchArgument(
            'drain_stable_polls',
            default_value='3',
            description='Consecutive equal counters required before export'),
        DeclareLaunchArgument(
            'drain_max_pending_frames',
            default_value='3',
            description=(
                'Frames allowed to stay unprocessed at the bag end (final '
                'depth frame has no later TF and the sim clock stops)')),
        DeclareLaunchArgument(
            'esdf_use_aabb',
            default_value='false',
            description=(
                'false exports the ESDF over all allocated blocks (complete '
                'map); true uses the esdf_min_*/esdf_size_* box below')),
        DeclareLaunchArgument(
            "global_frame",
            default_value="map",
            description=(
                "Coordinate frame used for offline nvblox and the exported "
                "ESDF. Use map when the bag contains map->odom TF and the "
                "ESDF should align with the saved 2D map; use odom for "
                "old odom-only workflows.")),
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
            "esdf_viz",
            default_value="false",
            description=(
                "Enable the live 3D ESDF visualizer node. Turn on together "
                "with rviz:=true to observe /nvblox_node/esdf_3d_pointcloud "
                "during offline replay.")),
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
            'ply_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'd455_bag_mesh.ply',
            ])),
        DeclareLaunchArgument(
            'timings_output',
            default_value=PathJoinSubstitution([
                LaunchConfiguration('output_dir'),
                'd455_bag_nvblox_timings.txt',
            ]),
            description='Audit file containing callback/processed counts'),
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
        exporter,
        export_after_playback,
        shutdown_after_export,
    ])
