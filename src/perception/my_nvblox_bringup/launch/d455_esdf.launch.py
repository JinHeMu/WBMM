#!/usr/bin/env python3
"""Consume a host-published base-mounted D455 and build a 3D ESDF."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Map D455 RGB-D streams; the camera driver runs on the host."""
    share = get_package_share_directory('my_nvblox_bringup')
    core_launch = os.path.join(share, 'launch', 'nvblox_core.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_color', default_value='true'),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.05',
            description='Real-time local ESDF voxel size in metres'),
        DeclareLaunchArgument(
            'map_clearing_radius_m',
            default_value='-1.0',
            description='Negative keeps the real-robot map persistent'),
        DeclareLaunchArgument(
            'maximum_input_queue_length', default_value='20'),
        DeclareLaunchArgument('tick_period_ms', default_value='10'),
        DeclareLaunchArgument(
            'integrate_depth_rate_hz', default_value='30.0'),
        DeclareLaunchArgument(
            'decay_tsdf_rate_hz',
            default_value='0.0',
            description='0.0 keeps the persistent static map forever'),
        DeclareLaunchArgument('update_mesh_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('update_esdf_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_layer_rate_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'publish_debug_vis_rate_hz', default_value='2.0'),
        DeclareLaunchArgument(
            'esdf_viz_follow_robot',
            default_value='false',
            description='False keeps a fixed world-aligned ESDF view'),
        DeclareLaunchArgument('esdf_viz_size_x', default_value='12.0'),
        DeclareLaunchArgument('esdf_viz_size_y', default_value='12.0'),
        DeclareLaunchArgument('esdf_viz_min_z', default_value='-0.2'),
        DeclareLaunchArgument('esdf_viz_size_z', default_value='3.0'),
        DeclareLaunchArgument('esdf_viz_rate', default_value='0.2'),
        DeclareLaunchArgument('esdf_viz_subsampling', default_value='3'),
        DeclareLaunchArgument(
            'esdf_viz_max_distance',
            default_value='0.5',
            description=(
                'Only visualize ESDF voxels within this distance of '
                'an obstacle')),
        DeclareLaunchArgument('ros_domain_id', default_value='20'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_fastrtps_cpp'),
        DeclareLaunchArgument(
            'input_qos',
            default_value='SENSOR_DATA',
            description='Use DEFAULT for lossless reliable bag replay'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/d455/depth/image_rect_raw'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/d455/depth/camera_info'),
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/d455/color/image_raw'),
        DeclareLaunchArgument(
            'color_camera_info_topic',
            default_value='/camera/d455/color/camera_info'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'global_frame': LaunchConfiguration('global_frame'),
                'voxel_size': LaunchConfiguration('voxel_size'),
                'map_clearing_frame_id': 'base_footprint',
                'map_clearing_radius_m':
                    LaunchConfiguration('map_clearing_radius_m'),
                'maximum_input_queue_length':
                    LaunchConfiguration('maximum_input_queue_length'),
                'tick_period_ms': LaunchConfiguration('tick_period_ms'),
                'integrate_depth_rate_hz':
                    LaunchConfiguration('integrate_depth_rate_hz'),
                'decay_tsdf_rate_hz':
                    LaunchConfiguration('decay_tsdf_rate_hz'),
                'update_mesh_rate_hz':
                    LaunchConfiguration('update_mesh_rate_hz'),
                'update_esdf_rate_hz':
                    LaunchConfiguration('update_esdf_rate_hz'),
                'publish_layer_rate_hz':
                    LaunchConfiguration('publish_layer_rate_hz'),
                'publish_debug_vis_rate_hz':
                    LaunchConfiguration('publish_debug_vis_rate_hz'),
                'esdf_viz_follow_robot':
                    LaunchConfiguration('esdf_viz_follow_robot'),
                'esdf_viz_size_x':
                    LaunchConfiguration('esdf_viz_size_x'),
                'esdf_viz_size_y':
                    LaunchConfiguration('esdf_viz_size_y'),
                'esdf_viz_min_z':
                    LaunchConfiguration('esdf_viz_min_z'),
                'esdf_viz_size_z':
                    LaunchConfiguration('esdf_viz_size_z'),
                'esdf_viz_rate':
                    LaunchConfiguration('esdf_viz_rate'),
                'esdf_viz_subsampling':
                    LaunchConfiguration('esdf_viz_subsampling'),
                'esdf_viz_max_distance':
                    LaunchConfiguration('esdf_viz_max_distance'),
                'input_qos': LaunchConfiguration('input_qos'),
                'depth_image_topic': LaunchConfiguration('depth_image_topic'),
                'depth_camera_info_topic':
                    LaunchConfiguration('depth_camera_info_topic'),
                'color_image_topic': LaunchConfiguration('color_image_topic'),
                'color_camera_info_topic':
                    LaunchConfiguration('color_camera_info_topic'),
                'use_color': LaunchConfiguration('use_color'),
                'use_lidar': 'false',
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
