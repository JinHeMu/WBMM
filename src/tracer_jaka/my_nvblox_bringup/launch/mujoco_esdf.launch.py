#!/usr/bin/env python3
"""nvblox side of the MuJoCo RGB-D/SLAM simulation."""

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
    """Consume the topics published by esdf_sensor_sim.launch.py."""
    share = get_package_share_directory('my_nvblox_bringup')
    core_launch = os.path.join(share, 'launch', 'nvblox_core.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('voxel_size', default_value='0.08'),
        DeclareLaunchArgument('map_clearing_radius_m', default_value='7.0'),
        DeclareLaunchArgument('esdf_viz_follow_robot', default_value='true'),
        DeclareLaunchArgument('esdf_viz_size_x', default_value='4.0'),
        DeclareLaunchArgument('esdf_viz_size_y', default_value='4.0'),
        DeclareLaunchArgument('esdf_viz_min_z', default_value='-0.2'),
        DeclareLaunchArgument('esdf_viz_size_z', default_value='3.0'),
        DeclareLaunchArgument('esdf_viz_rate', default_value='1.0'),
        DeclareLaunchArgument('esdf_viz_subsampling', default_value='2'),
        DeclareLaunchArgument('esdf_viz_max_distance', default_value='0.5'),
        DeclareLaunchArgument('ros_domain_id', default_value='20'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'global_frame': LaunchConfiguration('global_frame'),
                'map_clearing_frame_id': 'base_footprint',
                'voxel_size': LaunchConfiguration('voxel_size'),
                'map_clearing_radius_m': LaunchConfiguration(
                    'map_clearing_radius_m'),
                'input_qos': 'SENSOR_DATA',
                'depth_image_topic': '/camera/d455/depth/image_raw',
                'depth_camera_info_topic':
                    '/camera/d455/depth/camera_info',
                'color_image_topic': '/camera/d455/color/image_raw',
                'color_camera_info_topic':
                    '/camera/d455/color/camera_info',
                'use_color': 'true',
                'use_lidar': 'false',
                'rviz': LaunchConfiguration('rviz'),
                'esdf_viz_follow_robot': LaunchConfiguration(
                    'esdf_viz_follow_robot'),
                'esdf_viz_size_x': LaunchConfiguration('esdf_viz_size_x'),
                'esdf_viz_size_y': LaunchConfiguration('esdf_viz_size_y'),
                'esdf_viz_min_z': LaunchConfiguration('esdf_viz_min_z'),
                'esdf_viz_size_z': LaunchConfiguration('esdf_viz_size_z'),
                'esdf_viz_rate': LaunchConfiguration('esdf_viz_rate'),
                'esdf_viz_subsampling': LaunchConfiguration(
                    'esdf_viz_subsampling'),
                'esdf_viz_max_distance': LaunchConfiguration(
                    'esdf_viz_max_distance'),
            }.items(),
        ),
    ])
