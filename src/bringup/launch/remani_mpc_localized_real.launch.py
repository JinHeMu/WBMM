#!/usr/bin/env python3
# =============================================================================
#  remani_mpc_localized_real.launch.py
#
#  Real-robot REMANI + OCS2 MPC with persistent 2D/3D map localization.
#
#  Coordinate-frame architecture:
#    * map        : saved 2D SLAM map and exported 3D ESDF (persistent global)
#    * odom       : continuous EKF frame used by OCS2/MRT/control
#    * map -> odom: provided by AMCL (or slam_toolbox localization)
#
#  This launch composes:
#    1. real_slam.launch.py(start_slam:=false)
#       -> Tracer CAN, IMU, LiDAR, EKF, RSP (no slam_toolbox map->odom)
#    2. localization_real.launch.py
#       -> map_server + AMCL, publishes map -> odom
#    3. odom_to_map_relay
#       -> republishes /odometry/filtered as /odometry/filtered_map in map frame
#    4. ocs2_real.launch.py
#       -> JAKA ros2_control, MPC, MRT (continues controlling in odom)
#    5. remani_mpc_tracking.launch.py
#       -> REMANI plans in map frame, bridge dynamically transforms map -> odom
# =============================================================================

import os

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _enforce_safety_gate(context):
    """Fail closed before constructing any real command-producing nodes."""
    release = LaunchConfiguration('safety_release').perform(context).lower()
    read_only = LaunchConfiguration('jaka_read_only').perform(context).lower()
    command_output = LaunchConfiguration(
        'command_output_enabled').perform(context).lower()
    released = release in ('1', 'true', 'yes', 'on')
    arm_writable = read_only not in ('1', 'true', 'yes', 'on')
    commands_enabled = command_output in ('1', 'true', 'yes', 'on')
    if (arm_writable or commands_enabled) and not released:
        raise RuntimeError(
            'Real motion gates require safety_release:=true in addition to '
            'jaka_read_only:=false/command_output_enabled:=true')
    if commands_enabled and not arm_writable:
        raise RuntimeError(
            'command_output_enabled:=true is inconsistent with '
            'jaka_read_only:=true')

    map_file = LaunchConfiguration('map_file').perform(context)
    if not os.path.isfile(map_file):
        raise RuntimeError(
            f'Real localization map does not exist: {map_file}')

    start_remani = LaunchConfiguration('start_remani').perform(context).lower()
    if start_remani in ('1', 'true', 'yes', 'on'):
        esdf_file = LaunchConfiguration('static_esdf_file').perform(context)
        if not esdf_file or not os.path.isfile(esdf_file):
            raise RuntimeError(
                'start_remani:=true requires an existing static_esdf_file. '
                'Pass the site-specific .npz explicitly; no unsafe default '
                'environment map is selected.')
        try:
            with np.load(esdf_file, allow_pickle=False) as archive:
                if 'frame_id' not in archive.files:
                    raise RuntimeError(
                        'Static ESDF is missing required frame_id metadata')
                frame_value = archive['frame_id']
                if frame_value.shape != ():
                    raise RuntimeError(
                        'Static ESDF frame_id metadata must be scalar')
                esdf_frame = str(frame_value.item())
        except RuntimeError:
            raise
        except Exception as exception:
            raise RuntimeError(
                f'Cannot read static ESDF contract from {esdf_file}: '
                f'{exception}') from exception
        if esdf_frame != 'map':
            raise RuntimeError(
                'Localized real pipeline requires static ESDF frame_id=map, '
                f"but archive declares frame_id={esdf_frame!r}. Re-export or "
                'transform the ESDF; do not relabel it.')
    return []


def generate_launch_description():
    description_share = get_package_share_directory('tracer_jaka_description')
    remani_share = get_package_share_directory('remani_planner')
    localization_share = get_package_share_directory('tracer_jaka_localization')
    bringup_share = get_package_share_directory('tracer_jaka_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    can_port = LaunchConfiguration('can_port')
    serial_port = LaunchConfiguration('serial_port')
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    lidar_host_ip = LaunchConfiguration('lidar_host_ip')
    lidar_sensor_ip = LaunchConfiguration('lidar_sensor_ip')
    task_file = LaunchConfiguration('task_file')
    urdf_file = LaunchConfiguration('urdf_file')
    lib_folder = LaunchConfiguration('lib_folder')

    odom_topic = LaunchConfiguration('odom_topic')
    map_odom_topic = LaunchConfiguration('map_odom_topic')
    joint_state_topic = LaunchConfiguration('joint_state_topic')

    static_esdf_file = LaunchConfiguration('static_esdf_file')
    static_esdf_offset_x = LaunchConfiguration('static_esdf_offset_x')
    static_esdf_offset_y = LaunchConfiguration('static_esdf_offset_y')
    static_esdf_offset_z = LaunchConfiguration('static_esdf_offset_z')

    tracking_error_replan_enabled = LaunchConfiguration(
        'tracking_error_replan_enabled')
    manipulator_max_vel = LaunchConfiguration('manipulator_max_vel')
    manipulator_max_acc = LaunchConfiguration('manipulator_max_acc')
    freeze_manipulator = LaunchConfiguration('freeze_manipulator')
    mobile_base_max_wheel_omega = LaunchConfiguration(
        'mobile_base_max_wheel_omega')
    mobile_base_max_wheel_alpha = LaunchConfiguration(
        'mobile_base_max_wheel_alpha')
    mobile_base_non_singul_vel = LaunchConfiguration(
        'mobile_base_non_singul_vel')

    jaka_read_only = LaunchConfiguration('jaka_read_only')
    command_output_enabled = LaunchConfiguration('command_output_enabled')
    start_ocs2 = LaunchConfiguration('start_ocs2')
    start_remani = LaunchConfiguration('start_remani')
    start_bridge = LaunchConfiguration('start_bridge')
    arm_max_delta_per_step = LaunchConfiguration('arm_max_delta_per_step')
    arm_max_command_velocity = LaunchConfiguration(
        'arm_max_command_velocity')

    start_arm_pose = LaunchConfiguration('start_arm_pose')
    start_imu = LaunchConfiguration('start_imu')
    start_lidar = LaunchConfiguration('start_lidar')
    use_joy = LaunchConfiguration('use_joy')

    scan_topic = LaunchConfiguration('scan_topic')
    map_file = LaunchConfiguration('map_file')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')

    default_urdf = os.path.join(
        description_share, 'urdf', 'tracer_jaka_zu5.urdf')
    default_task = os.path.join(
        bringup_share, 'config', 'task_real_conservative.info')
    default_esdf = ''
    default_map = os.path.join(
        localization_share, 'maps', 'factory_map.yaml')

    declare_args = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('robot_ip', default_value='10.5.5.100'),
        DeclareLaunchArgument('local_ip', default_value='10.5.5.127'),
        DeclareLaunchArgument('lidar_host_ip', default_value='0.0.0.0'),
        DeclareLaunchArgument(
            'lidar_sensor_ip', default_value='192.168.198.2'),
        DeclareLaunchArgument('jaka_read_only', default_value='true'),
        DeclareLaunchArgument(
            'command_output_enabled', default_value='false'),
        DeclareLaunchArgument(
            'safety_release', default_value='false',
            description=(
                'Explicit third gate required before real command output. '
                'Keep false for L5/read-only validation.')),
        DeclareLaunchArgument('start_ocs2', default_value='true'),
        DeclareLaunchArgument('start_remani', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument(
            'arm_max_delta_per_step', default_value='0.05'),
        DeclareLaunchArgument(
            'arm_max_command_velocity', default_value='0.15'),
        DeclareLaunchArgument('task_file', default_value=default_task),
        DeclareLaunchArgument('urdf_file', default_value=default_urdf),
        DeclareLaunchArgument(
            'lib_folder',
            default_value='/tmp/ocs2_tracer_jaka_real/auto_generated'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument(
            'map_odom_topic', default_value='/odometry/filtered_map'),
        DeclareLaunchArgument(
            'joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument('static_esdf_file', default_value=default_esdf),
        DeclareLaunchArgument('static_esdf_offset_x', default_value='0.0'),
        DeclareLaunchArgument('static_esdf_offset_y', default_value='0.0'),
        DeclareLaunchArgument('static_esdf_offset_z', default_value='0.0'),
        DeclareLaunchArgument(
            'tracking_error_replan_enabled', default_value='false'),
        DeclareLaunchArgument('manipulator_max_vel', default_value='0.10'),
        DeclareLaunchArgument('manipulator_max_acc', default_value='0.20'),
        DeclareLaunchArgument('freeze_manipulator', default_value='true'),
        DeclareLaunchArgument(
            'mobile_base_max_wheel_omega', default_value='1.0'),
        DeclareLaunchArgument(
            'mobile_base_max_wheel_alpha', default_value='2.0'),
        DeclareLaunchArgument(
            'mobile_base_non_singul_vel', default_value='0.02'),
        DeclareLaunchArgument('start_arm_pose', default_value='false'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
    ]

    # 1. Hardware + EKF only. slam_toolbox is disabled because AMCL below
    #    owns map->odom.
    real_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'real_slam.launch.py')),
        launch_arguments={
            'start_base': 'true',
            'start_slam': 'false',
            'start_robot_state_publisher': 'true',
            'start_arm_pose': start_arm_pose,
            'start_imu': start_imu,
            'start_lidar': start_lidar,
            'rviz': 'false',
            'can_port': can_port,
            'serial_port': serial_port,
            'lidar_host_ip': lidar_host_ip,
            'lidar_sensor_ip': lidar_sensor_ip,
            'wheel_odom_topic': '/odom',
            'imu_topic': '/IMU_data',
            'scan_topic': scan_topic,
        }.items(),
    )

    # 2. AMCL + map_server -> map -> odom.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            localization_share, 'launch', 'localization_real.launch.py')),
        launch_arguments={
            'map_file': map_file,
            'scan_topic': scan_topic,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_yaw': initial_yaw,
            'use_sim_time': 'false',
            'start_esdf_visualization': 'false',
            'esdf_file': static_esdf_file,
            'esdf_offset_x': static_esdf_offset_x,
            'esdf_offset_y': static_esdf_offset_y,
            'esdf_offset_z': static_esdf_offset_z,
        }.items(),
    )

    # 3. Republish EKF odometry in map frame for REMANI.
    odom_to_map_relay = Node(
        package='tracer_jaka_bringup',
        executable='odom_to_map_relay.py',
        name='odom_to_map_relay',
        output='screen',
        parameters=[{
            'odom_topic': odom_topic,
            'output_topic': map_odom_topic,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'child_frame': 'base_footprint',
        }],
    )

    # 4. JAKA ros2_control + OCS2 MPC/MRT (continues in odom).
    ocs2_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'ocs2_real.launch.py')),
        launch_arguments={
            'use_rviz': use_rviz,
            'start_base': 'false',
            'start_robot_state_publisher': 'false',
            'mrt_odom_topic': odom_topic,
            'publish_odom_tf': 'false',
            'use_joy': use_joy,
            'can_port': can_port,
            'task_file': task_file,
            'urdf_file': urdf_file,
            'lib_folder': lib_folder,
            'robot_ip': robot_ip,
            'local_ip': local_ip,
            'jaka_read_only': jaka_read_only,
            'command_output_enabled': command_output_enabled,
            'safety_release': LaunchConfiguration('safety_release'),
            'start_ocs2': start_ocs2,
            'arm_max_delta_per_step': arm_max_delta_per_step,
            'arm_max_command_velocity': arm_max_command_velocity,
        }.items(),
    )

    # 5. REMANI plans in map; bridge transforms map trajectory to odom.
    remani_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            remani_share, 'launch', 'remani_mpc_tracking.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'start_planner': 'true',
            'start_bridge': start_bridge,
            'urdf_file': urdf_file,
            'static_esdf_file': static_esdf_file,
            'static_esdf_offset_x': static_esdf_offset_x,
            'static_esdf_offset_y': static_esdf_offset_y,
            'static_esdf_offset_z': static_esdf_offset_z,
            'odom_topic': map_odom_topic,
            'joint_state_topic': joint_state_topic,
            'planner_frame': 'map',
            'target_frame': 'odom',
            'use_tf_transform': 'true',
            'planner_to_ocs2_x': '0.0',
            'planner_to_ocs2_y': '0.0',
            'planner_to_ocs2_yaw': '0.0',
            'tracking_error_replan_enabled':
                tracking_error_replan_enabled,
            'manipulator_max_vel': manipulator_max_vel,
            'manipulator_max_acc': manipulator_max_acc,
            'freeze_manipulator': freeze_manipulator,
            'mobile_base_max_wheel_omega':
                mobile_base_max_wheel_omega,
            'mobile_base_max_wheel_alpha':
                mobile_base_max_wheel_alpha,
            'mobile_base_non_singul_vel':
                mobile_base_non_singul_vel,
        }.items(),
        condition=IfCondition(start_remani),
    )

    remani_delayed = TimerAction(
        period=15.0,
        actions=[remani_launch],
    )

    return LaunchDescription(
        declare_args +
        [
            OpaqueFunction(function=_enforce_safety_gate),
            real_slam_launch,
            localization_launch,
            odom_to_map_relay,
            ocs2_real_launch,
            remani_delayed,
        ]
    )
