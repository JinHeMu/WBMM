#!/usr/bin/env python3
"""Real-robot AMCL + map_server localization in the map frame.

This launch provides map -> odom. It is intended to be used together with
real_slam.launch.py(start_slam:=false) so slam_toolbox mapping does not
compete for the map->odom transform.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('tracer_jaka_localization')
    default_map = os.path.join(pkg_share, 'maps', 'factory_map.yaml')

    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = LaunchConfiguration('scan_topic')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    map_frame = LaunchConfiguration('map_frame')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    start_esdf_visualization = LaunchConfiguration(
        'start_esdf_visualization')
    esdf_file = LaunchConfiguration('esdf_file')
    esdf_offset_x = LaunchConfiguration('esdf_offset_x')
    esdf_offset_y = LaunchConfiguration('esdf_offset_y')
    esdf_offset_z = LaunchConfiguration('esdf_offset_z')

    amcl_config = os.path.join(pkg_share, 'config', 'amcl_real.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file,
        }],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            # AMCL is a lifecycle node too.  If it is omitted here it stays
            # unconfigured and never publishes map -> odom.
            'node_names': ['map_server', 'amcl'],
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': use_sim_time,
                'scan_topic': scan_topic,
                'base_frame_id': base_frame,
                'odom_frame_id': odom_frame,
                'global_frame_id': map_frame,
                # Humble does not consume initial_pose.* automatically unless
                # this switch is enabled.
                'set_initial_pose': True,
                'initial_pose.x': ParameterValue(initial_x, value_type=float),
                'initial_pose.y': ParameterValue(initial_y, value_type=float),
                'initial_pose.yaw': ParameterValue(
                    initial_yaw, value_type=float),
            },
        ],
        remappings=[('scan', scan_topic)],
    )

    # Static, read-only visualization of the saved 3D ESDF.  This does not
    # start nvblox or modify the map used by AMCL/REMANI.
    esdf_visualization = Node(
        package='grid_map',
        executable='esdf_rviz_publisher',
        name='saved_esdf_rviz_publisher',
        output='screen',
        parameters=[{
            'esdf_file': esdf_file,
            'frame_id': map_frame,
            'z_slice': -1.0,
            'max_distance': 0.35,
            'stride': 2,
            'publish_period': 1.0,
            'include_unknown': False,
            'publish_surface_mesh': False,
            'publish_ply_mesh': False,
            'offset_x': ParameterValue(esdf_offset_x, value_type=float),
            'offset_y': ParameterValue(esdf_offset_y, value_type=float),
            'offset_z': ParameterValue(esdf_offset_z, value_type=float),
        }],
        condition=IfCondition(start_esdf_visualization),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'start_esdf_visualization',
            default_value='false',
            description='Publish the saved 3D ESDF as /esdf_cloud for RViz.'),
        DeclareLaunchArgument(
            'esdf_file', default_value='',
            description=(
                'Optional saved ESDF used only when '
                'start_esdf_visualization:=true.')),
        DeclareLaunchArgument('esdf_offset_x', default_value='0.0'),
        DeclareLaunchArgument('esdf_offset_y', default_value='0.0'),
        DeclareLaunchArgument('esdf_offset_z', default_value='0.0'),
        map_server,
        lifecycle,
        amcl,
        esdf_visualization,
    ])
