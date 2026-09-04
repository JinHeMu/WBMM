#!/usr/bin/env python3
"""MuJoCo mapping stage for the nvblox -> REMANI demonstration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('tracer_jaka_bringup')
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    sensor_launch = os.path.join(
        bringup_share, 'launch', 'mujoco_esdf_sensor.launch.py')
    demo_scene = os.path.join(
        mujoco_share, 'models', 'scene_nvblox_remani_demo.xml')

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'camera_rate', default_value='15.0',
            description='RGB-D rate; 15 Hz is sufficient and reduces GPU load.'),
        DeclareLaunchArgument('camera_width', default_value='128'),
        DeclareLaunchArgument('camera_height', default_value='96'),
        DeclareLaunchArgument('ros_domain_id', default_value='20'),
        DeclareLaunchArgument(
            'rmw_implementation', default_value='rmw_fastrtps_cpp'),
        SetEnvironmentVariable(
            'ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            LaunchConfiguration('rmw_implementation')),
        # Host-network Docker can discover FastDDS endpoints while its shared
        # memory data path still fails. UDPv4 makes RGB-D delivery explicit.
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sensor_launch),
            launch_arguments={
                'model': demo_scene,
                # Mapping uses the low keyframe so the scripted sensor pass can
                # safely travel below the portal before REMANI plans it.
                'init_keyframe': 'low',
                'viewer': LaunchConfiguration('viewer'),
                'camera_rate': LaunchConfiguration('camera_rate'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'rviz': LaunchConfiguration('rviz'),
                'ros_domain_id': LaunchConfiguration('ros_domain_id'),
                'rmw_implementation': LaunchConfiguration(
                    'rmw_implementation'),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='tracer_jaka_mujoco',
                    executable='esdf_mapping_scan',
                    name='esdf_mapping_scan',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'start_delay': 4.0,
                    }],
                ),
            ],
        ),
    ])
