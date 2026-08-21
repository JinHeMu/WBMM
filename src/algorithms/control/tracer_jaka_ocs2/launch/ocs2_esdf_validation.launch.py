#!/usr/bin/env python3
"""Validate a saved real-scene nvblox ESDF in MuJoCo with REMANI+OCS2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ocs2_share = get_package_share_directory('tracer_jaka_ocs2')
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    local_maps_dir = '/home/a/WBMM/maps'
    sim_launch = os.path.join(ocs2_share, 'launch', 'ocs2_sim.launch.py')
    task_file = os.path.join(ocs2_share, 'config', 'task_esdf_only.info')
    default_scene_file = os.path.join(
        mujoco_share, 'models', 'scene_esdf_validation.xml')
    rviz_file = os.path.join(
        ocs2_share, 'rviz', 'tracer_jaka_esdf_validation.rviz')

    esdf_file = LaunchConfiguration('esdf_file')
    ply_file = LaunchConfiguration('ply_file')
    map2d_yaml = LaunchConfiguration('map2d_yaml')
    frame_id = LaunchConfiguration('frame_id')
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
            'remani_planner_frame': frame_id,
            'remani_target_frame': 'odom',
            'remani_use_tf_transform': 'true',
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
            'frame_id': frame_id,
            'z_slice': -1.0,
            'max_distance': ParameterValue(
                LaunchConfiguration('esdf_display_distance'),
                value_type=float),
            'stride': ParameterValue(
                LaunchConfiguration('esdf_display_stride'),
                value_type=int),
            'include_unknown': ParameterValue(
                LaunchConfiguration('esdf_display_unknown'),
                value_type=bool),
            'publish_surface_mesh': ParameterValue(
                LaunchConfiguration('publish_esdf_mesh'),
                value_type=bool),
            'ply_file': ParameterValue(ply_file, value_type=str),
            'publish_ply_mesh': ParameterValue(
                LaunchConfiguration('publish_ply_mesh'),
                value_type=bool),
            'publish_period': 1.0,
            'z_min_2d': ParameterValue(
                LaunchConfiguration('esdf_2d_min_z'),
                value_type=float),
            'z_max_2d': ParameterValue(
                LaunchConfiguration('esdf_2d_max_z'),
                value_type=float),
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
            'frame_id',
            default_value='map',
            description=(
                'Frame in which the saved ESDF/PLY is expressed. Use map '
                'for the persistent-map pipeline, odom for legacy.')),
        DeclareLaunchArgument(
            'esdf_file',
            default_value=os.path.join(local_maps_dir, 'site_remani.npz'),
            description='REMANI-format NPZ exported from nvblox.'),
        DeclareLaunchArgument(
            'map2d_yaml',
            default_value=os.path.join(local_maps_dir, 'site_2d.yaml'),
            description='2D map saved by slam_toolbox.'),
        DeclareLaunchArgument(
            'ply_file',
            default_value=os.path.join(local_maps_dir, 'site_mesh.ply'),
            description=(
                'Saved nvblox mesh PLY for RViz visualization.')),
        DeclareLaunchArgument(
            'mujoco_model',
            default_value=default_scene_file,
            description='MuJoCo scene used for ESDF planning validation.'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'esdf_display_distance',
            default_value='0.5',
            description=(
                'Only show observed ESDF voxels within this distance of an '
                'obstacle; <= 0 disables filtering.')),
        DeclareLaunchArgument(
            'esdf_display_stride',
            default_value='2',
            description='3D ESDF voxel subsampling; 2 reduces RViz clutter.'),
        DeclareLaunchArgument(
            'esdf_display_unknown',
            default_value='false',
            description=(
                'Also render conservative unknown voxels. Normally false to '
                'avoid filling the complete query volume.')),
        DeclareLaunchArgument(
            'publish_esdf_mesh',
            default_value='false',
            description=(
                'Publish the legacy occupancy-derived surface mesh.')),
        DeclareLaunchArgument(
            'publish_ply_mesh',
            default_value='true',
            description='Publish the saved native nvblox PLY in RViz.'),
        DeclareLaunchArgument(
            'esdf_2d_min_z',
            default_value='0.05',
            description=(
                'Minimum height used by the 2D occupancy projection.')),
        DeclareLaunchArgument(
            'esdf_2d_max_z',
            default_value='0.60',
            description=(
                'Maximum height used by the 2D occupancy projection.')),
        DeclareLaunchArgument(
            'remani_manipulator_max_vel', default_value='1.57'),
        DeclareLaunchArgument(
            'remani_manipulator_max_acc', default_value='3.14'),
        DeclareLaunchArgument(
            'remani_freeze_manipulator', default_value='false'),
        simulator_and_planner,
        saved_esdf_visualization,
        saved_2d_map_server,
        map_lifecycle_manager,
    ])
