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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tracer_mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    ocs2_share = get_package_share_directory('tracer_jaka_ocs2')
    remani_share = get_package_share_directory('remani_planner')
    localization_share = get_package_share_directory('tracer_jaka_localization')
    bringup_share = get_package_share_directory('tracer_jaka_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    can_port = LaunchConfiguration('can_port')
    serial_port = LaunchConfiguration('serial_port')
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
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
        tracer_mujoco_share, 'urdf', 'tracer_jaka_zu5_real.urdf')
    default_task = os.path.join(ocs2_share, 'config', 'task_real.info')
    default_esdf = '/home/a/workspaces/isaac_ros-dev/bag_export/site_remani.npz'
    default_map = os.path.join(bringup_share, 'maps', 'factory_map.yaml')

    declare_args = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('can_port', default_value='can0'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('robot_ip', default_value='10.5.5.100'),
        DeclareLaunchArgument('local_ip', default_value='10.5.5.127'),
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
        DeclareLaunchArgument('manipulator_max_vel', default_value='0.3'),
        DeclareLaunchArgument('manipulator_max_acc', default_value='0.5'),
        DeclareLaunchArgument('freeze_manipulator', default_value='false'),
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
            tracer_mujoco_share, 'launch', 'real_slam.launch.py')),
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
            ocs2_share, 'launch', 'ocs2_real.launch.py')),
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
        }.items(),
    )

    # 5. REMANI plans in map; bridge transforms map trajectory to odom.
    remani_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            remani_share, 'launch', 'remani_mpc_tracking.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'start_planner': 'true',
            'start_bridge': 'true',
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
        }.items(),
    )

    remani_delayed = TimerAction(
        period=15.0,
        actions=[remani_launch],
    )

    return LaunchDescription(
        declare_args +
        [
            real_slam_launch,
            localization_launch,
            odom_to_map_relay,
            ocs2_real_launch,
            remani_delayed,
        ]
    )
