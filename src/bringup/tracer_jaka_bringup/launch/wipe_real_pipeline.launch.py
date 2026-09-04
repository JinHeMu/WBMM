#!/usr/bin/env python3
"""Canonical localized real-robot navigation -> wipe system composition."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('tracer_jaka_bringup')
    wipe_share = get_package_share_directory('wipe_planner')
    description_share = get_package_share_directory('tracer_jaka_description')
    localization_share = get_package_share_directory(
        'tracer_jaka_localization')
    args = {
        name: LaunchConfiguration(name) for name in [
            'use_rviz', 'can_port', 'serial_port', 'robot_ip', 'local_ip',
            'lidar_host_ip', 'lidar_sensor_ip', 'map_file',
            'static_esdf_file', 'lib_folder', 'initial_x', 'initial_y',
            'initial_yaw', 'tracking_error_replan_enabled',
            'freeze_manipulator', 'manipulator_max_vel',
            'manipulator_max_acc', 'mobile_base_max_wheel_omega',
            'mobile_base_max_wheel_alpha', 'mobile_base_non_singul_vel',
            'jaka_read_only', 'command_output_enabled', 'safety_release',
            'start_arm_pose', 'arm_max_delta_per_step',
            'arm_max_command_velocity', 'ocs2_task_file', 'urdf_file',
            'wipe_task_file', 'auto_goal', 'force_control_enabled',
            'board_marker_enabled', 'wipe_start_delay',
        ]
    }

    localized_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'remani_mpc_localized_real.launch.py')),
        launch_arguments={
            'use_rviz': args['use_rviz'],
            'use_joy': 'false',
            'can_port': args['can_port'],
            'serial_port': args['serial_port'],
            'robot_ip': args['robot_ip'],
            'local_ip': args['local_ip'],
            'lidar_host_ip': args['lidar_host_ip'],
            'lidar_sensor_ip': args['lidar_sensor_ip'],
            'map_file': args['map_file'],
            'static_esdf_file': args['static_esdf_file'],
            'lib_folder': args['lib_folder'],
            'initial_x': args['initial_x'],
            'initial_y': args['initial_y'],
            'initial_yaw': args['initial_yaw'],
            'tracking_error_replan_enabled':
                args['tracking_error_replan_enabled'],
            'freeze_manipulator': args['freeze_manipulator'],
            'manipulator_max_vel': args['manipulator_max_vel'],
            'manipulator_max_acc': args['manipulator_max_acc'],
            'mobile_base_max_wheel_omega':
                args['mobile_base_max_wheel_omega'],
            'mobile_base_max_wheel_alpha':
                args['mobile_base_max_wheel_alpha'],
            'mobile_base_non_singul_vel':
                args['mobile_base_non_singul_vel'],
            'jaka_read_only': args['jaka_read_only'],
            'command_output_enabled': args['command_output_enabled'],
            'safety_release': args['safety_release'],
            'start_ocs2': 'true',
            'start_remani': 'true',
            # WipePlanner owns navigation relay and task reference end-to-end.
            'start_bridge': 'false',
            'start_arm_pose': args['start_arm_pose'],
            'arm_max_delta_per_step': args['arm_max_delta_per_step'],
            'arm_max_command_velocity': args['arm_max_command_velocity'],
            'task_file': args['ocs2_task_file'],
            'urdf_file': args['urdf_file'],
        }.items(),
    )
    wipe_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            wipe_share, 'launch', 'wipe_real.launch.py')),
        launch_arguments={
            'wipe_task_file': args['wipe_task_file'],
            'urdf_file': args['urdf_file'],
            'planner_frame': 'map',
            'control_frame': 'odom',
            'auto_goal': args['auto_goal'],
            'force_control_enabled': args['force_control_enabled'],
            'board_marker_enabled': args['board_marker_enabled'],
        }.items(),
    )
    declarations = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument(
            'serial_port',
            default_value=(
                '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_'
                'Bridge_Controller_e6872e3dafebed119ff7429aa88ea882-'
                'if00-port0')),
        DeclareLaunchArgument('robot_ip', default_value='10.5.5.100'),
        DeclareLaunchArgument('local_ip', default_value='10.5.5.127'),
        DeclareLaunchArgument('lidar_host_ip', default_value='192.168.8.1'),
        DeclareLaunchArgument('lidar_sensor_ip', default_value='192.168.8.2'),
        DeclareLaunchArgument(
            'map_file', default_value=os.path.join(
                localization_share, 'maps', 'factory_map.yaml')),
        DeclareLaunchArgument(
            'static_esdf_file',
            default_value='',
            description=(
                'Required site-specific REMANI ESDF. The localized real '
                'pipeline fails closed when this file is absent.')),
        DeclareLaunchArgument(
            'lib_folder',
            default_value='/tmp/ocs2_tracer_jaka_conservative/auto_generated'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'tracking_error_replan_enabled', default_value='false'),
        DeclareLaunchArgument('freeze_manipulator', default_value='false'),
        DeclareLaunchArgument('manipulator_max_vel', default_value='0.10'),
        DeclareLaunchArgument('manipulator_max_acc', default_value='0.20'),
        DeclareLaunchArgument(
            'mobile_base_max_wheel_omega', default_value='1.5'),
        DeclareLaunchArgument(
            'mobile_base_max_wheel_alpha', default_value='3.0'),
        DeclareLaunchArgument(
            'mobile_base_non_singul_vel', default_value='0.05'),
        DeclareLaunchArgument('jaka_read_only', default_value='true'),
        DeclareLaunchArgument(
            'command_output_enabled', default_value='false'),
        DeclareLaunchArgument('safety_release', default_value='false'),
        DeclareLaunchArgument('start_arm_pose', default_value='false'),
        DeclareLaunchArgument('arm_max_delta_per_step', default_value='0.05'),
        DeclareLaunchArgument(
            'arm_max_command_velocity', default_value='0.10'),
        DeclareLaunchArgument(
            'ocs2_task_file',
            default_value=os.path.join(
                bringup_share, 'config', 'task_real_conservative.info')),
        DeclareLaunchArgument(
            'urdf_file',
            default_value=os.path.join(
                description_share, 'urdf', 'tracer_jaka_zu5.urdf')),
        DeclareLaunchArgument(
            'wipe_task_file',
            default_value=os.path.join(
                wipe_share, 'config', 'wipe_task_real_front.yaml')),
        DeclareLaunchArgument(
            'auto_goal', default_value='false',
            description='Manual opt-in only after readiness checks.'),
        DeclareLaunchArgument(
            'force_control_enabled', default_value='false'),
        DeclareLaunchArgument('board_marker_enabled', default_value='true'),
        DeclareLaunchArgument('wipe_start_delay', default_value='20.0'),
    ]
    return LaunchDescription(declarations + [
        localized_real,
        TimerAction(period=args['wipe_start_delay'], actions=[wipe_real]),
    ])
