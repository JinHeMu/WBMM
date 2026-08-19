#!/usr/bin/env python3
"""Sensor-agnostic nvblox 3D ESDF bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch one RGB-D nvblox mapper with configurable ROS interfaces."""
    share = get_package_share_directory('my_nvblox_bringup')
    config = os.path.join(share, 'config', 'nvblox_3d.yaml')
    rviz_config = os.path.join(share, 'config', 'nvblox_esdf.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    clearing_frame = LaunchConfiguration('map_clearing_frame_id')
    remappings = [
        ('camera_0/depth/image', LaunchConfiguration('depth_image_topic')),
        ('camera_0/depth/camera_info',
         LaunchConfiguration('depth_camera_info_topic')),
        ('camera_0/color/image', LaunchConfiguration('color_image_topic')),
        ('camera_0/color/camera_info',
         LaunchConfiguration('color_camera_info_topic')),
        ('pointcloud', LaunchConfiguration('pointcloud_topic')),
    ]

    nvblox_node = ComposableNode(
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        name='nvblox_node',
        remappings=remappings,
        parameters=[
            config,
            {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'use_color': ParameterValue(
                    LaunchConfiguration('use_color'), value_type=bool),
                'use_lidar': ParameterValue(
                    LaunchConfiguration('use_lidar'), value_type=bool),
                'global_frame': LaunchConfiguration('global_frame'),
                'voxel_size': ParameterValue(
                    LaunchConfiguration('voxel_size'), value_type=float),
                'map_clearing_frame_id': clearing_frame,
                'map_clearing_radius_m': ParameterValue(
                    LaunchConfiguration('map_clearing_radius_m'),
                    value_type=float),
                'esdf_slice_bounds_visualization_attachment_frame_id':
                    clearing_frame,
                'workspace_height_bounds_visualization_attachment_frame_id':
                    clearing_frame,
                'input_qos': LaunchConfiguration('input_qos'),
                'maximum_input_queue_length': ParameterValue(
                    LaunchConfiguration('maximum_input_queue_length'),
                    value_type=int),
                'tick_period_ms': ParameterValue(
                    LaunchConfiguration('tick_period_ms'), value_type=int),
                'integrate_depth_rate_hz': ParameterValue(
                    LaunchConfiguration('integrate_depth_rate_hz'),
                    value_type=float),
                'decay_tsdf_rate_hz': ParameterValue(
                    LaunchConfiguration('decay_tsdf_rate_hz'),
                    value_type=float),
                'update_mesh_rate_hz': ParameterValue(
                    LaunchConfiguration('update_mesh_rate_hz'),
                    value_type=float),
                'update_esdf_rate_hz': ParameterValue(
                    LaunchConfiguration('update_esdf_rate_hz'),
                    value_type=float),
                'publish_layer_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_layer_rate_hz'),
                    value_type=float),
                'publish_debug_vis_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_debug_vis_rate_hz'),
                    value_type=float),
            },
        ],
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container_mt',
        name='nvblox_container',
        namespace='',
        output='screen',
        composable_node_descriptions=[nvblox_node],
        arguments=['--ros-args', '--log-level',
                   LaunchConfiguration('log_level')],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='nvblox_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
        }],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    esdf_visualizer = Node(
        package='my_nvblox_bringup',
        executable='esdf_visualizer',
        name='esdf_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'global_frame': LaunchConfiguration('global_frame'),
            'tracking_frame': clearing_frame,
            'follow_tracking_frame': ParameterValue(
                LaunchConfiguration('esdf_viz_follow_robot'),
                value_type=bool),
            'query_size_x_m': ParameterValue(
                LaunchConfiguration('esdf_viz_size_x'), value_type=float),
            'query_size_y_m': ParameterValue(
                LaunchConfiguration('esdf_viz_size_y'), value_type=float),
            'query_min_z_m': ParameterValue(
                LaunchConfiguration('esdf_viz_min_z'), value_type=float),
            'query_size_z_m': ParameterValue(
                LaunchConfiguration('esdf_viz_size_z'), value_type=float),
            'publish_rate_hz': ParameterValue(
                LaunchConfiguration('esdf_viz_rate'), value_type=float),
            'max_visualized_distance_m': ParameterValue(
                LaunchConfiguration('esdf_viz_max_distance'),
                value_type=float),
            'voxel_subsampling': ParameterValue(
                LaunchConfiguration('esdf_viz_subsampling'), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration('esdf_viz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('global_frame', default_value='odom'),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.05',
            description='TSDF/ESDF voxel size in metres'),
        DeclareLaunchArgument(
            'map_clearing_frame_id', default_value='base_footprint'),
        DeclareLaunchArgument(
            'map_clearing_radius_m',
            default_value='7.0',
            description='Set below zero to keep all integrated map blocks'),
        DeclareLaunchArgument('input_qos', default_value='SENSOR_DATA'),
        DeclareLaunchArgument(
            'maximum_input_queue_length',
            default_value='20',
            description='Maximum queued sensor messages awaiting TF'),
        DeclareLaunchArgument('tick_period_ms', default_value='10'),
        DeclareLaunchArgument(
            'integrate_depth_rate_hz', default_value='30.0'),
        DeclareLaunchArgument(
            'decay_tsdf_rate_hz',
            default_value='0.0',
            description=(
                'TSDF weight decay rate. 0.0 disables decay so static_tsdf '
                'maps stay persistent; nvblox default 5 Hz erases anything '
                'not re-observed within ~30 s')),
        DeclareLaunchArgument('update_mesh_rate_hz', default_value='5.0'),
        DeclareLaunchArgument('update_esdf_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_layer_rate_hz', default_value='5.0'),
        DeclareLaunchArgument(
            'publish_debug_vis_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('use_color', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='false'),
        DeclareLaunchArgument(
            'depth_image_topic',
            default_value='/camera/d455/depth/image_raw'),
        DeclareLaunchArgument(
            'depth_camera_info_topic',
            default_value='/camera/d455/depth/camera_info'),
        DeclareLaunchArgument(
            'color_image_topic',
            default_value='/camera/d455/color/image_raw'),
        DeclareLaunchArgument(
            'color_camera_info_topic',
            default_value='/camera/d455/color/camera_info'),
        DeclareLaunchArgument(
            'pointcloud_topic', default_value='/lidar/points'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('esdf_viz', default_value='true'),
        DeclareLaunchArgument(
            'esdf_viz_follow_robot',
            default_value='true',
            description=(
                'Move the ESDF query window with the robot; false freezes '
                'the window at the robot start pose for persistent display')),
        DeclareLaunchArgument('esdf_viz_size_x', default_value='4.0'),
        DeclareLaunchArgument('esdf_viz_size_y', default_value='4.0'),
        DeclareLaunchArgument('esdf_viz_min_z', default_value='-0.2'),
        DeclareLaunchArgument('esdf_viz_size_z', default_value='3.0'),
        DeclareLaunchArgument('esdf_viz_rate', default_value='1.0'),
        DeclareLaunchArgument(
            'esdf_viz_max_distance', default_value='1.5'),
        DeclareLaunchArgument('esdf_viz_subsampling', default_value='2'),
        DeclareLaunchArgument('log_level', default_value='info'),
        container,
        esdf_visualizer,
        rviz,
    ])
