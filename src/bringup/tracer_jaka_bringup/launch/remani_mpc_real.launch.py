#!/usr/bin/env python3
# =============================================================================
#  remani_mpc_real.launch.py
#
#  WBMM top-level real-robot REMANI + OCS2 MPC bringup.
#
#  This launch composes:
#    1. real_slam.launch.py          -> Tracer CAN, IMU, LiDAR, EKF, SLAM, RSP
#    2. ocs2_real.launch.py          -> JAKA ros2_control, MPC, MRT, optional RViz
#    3. remani_mpc_tracking.launch.py-> REMANI planner + REMANI->OCS2 bridge
#
#  The underlying ocs2_real.launch.py has been made composable:
#    * start_base := false (base is started by real_slam.launch.py)
#    * start_robot_state_publisher := false (RSP is started by real_slam.launch.py)
#    * mrt_odom_topic := /odometry/filtered (EKF output)
#    * publish_odom_tf := false (EKF owns odom -> base_footprint)
#
#  Only one OCS2 target publisher is allowed.  This launch therefore starts
#  ocs2_real with use_joy := false and lets the REMANI bridge own
#  /mobile_manipulator_mpc_target.
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tracer_mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    ocs2_share = get_package_share_directory('tracer_jaka_ocs2')
    remani_share = get_package_share_directory('remani_planner')
    grid_map_share = get_package_share_directory('grid_map')

    # ------------------------------------------------------------------
    # Top-level arguments
    # ------------------------------------------------------------------
    use_rviz = LaunchConfiguration('use_rviz')
    can_port = LaunchConfiguration('can_port')
    serial_port = LaunchConfiguration('serial_port')
    robot_ip = LaunchConfiguration('robot_ip')
    local_ip = LaunchConfiguration('local_ip')
    task_file = LaunchConfiguration('task_file')
    urdf_file = LaunchConfiguration('urdf_file')
    lib_folder = LaunchConfiguration('lib_folder')

    odom_topic = LaunchConfiguration('odom_topic')
    joint_state_topic = LaunchConfiguration('joint_state_topic')

    static_esdf_file = LaunchConfiguration('static_esdf_file')
    static_esdf_offset_x = LaunchConfiguration('static_esdf_offset_x')
    static_esdf_offset_y = LaunchConfiguration('static_esdf_offset_y')
    static_esdf_offset_z = LaunchConfiguration('static_esdf_offset_z')
    planner_to_ocs2_x = LaunchConfiguration('planner_to_ocs2_x')
    planner_to_ocs2_y = LaunchConfiguration('planner_to_ocs2_y')
    planner_to_ocs2_yaw = LaunchConfiguration('planner_to_ocs2_yaw')

    tracking_error_replan_enabled = LaunchConfiguration(
        'tracking_error_replan_enabled')
    manipulator_max_vel = LaunchConfiguration('manipulator_max_vel')
    manipulator_max_acc = LaunchConfiguration('manipulator_max_acc')
    freeze_manipulator = LaunchConfiguration('freeze_manipulator')

    start_arm_pose = LaunchConfiguration('start_arm_pose')
    start_imu = LaunchConfiguration('start_imu')
    start_lidar = LaunchConfiguration('start_lidar')
    use_joy = LaunchConfiguration('use_joy')

    default_urdf = os.path.join(
        tracer_mujoco_share, 'urdf', 'tracer_jaka_zu5_real.urdf')
    default_task = os.path.join(
        ocs2_share, 'config', 'task_real.info')
    default_esdf = os.path.join(
        grid_map_share, 'maps', 'tracer_jaka_zu5_scene_esdf.npz')

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
        DeclareLaunchArgument(
            'odom_topic', default_value='/odometry/filtered',
            description='Odometry used by both MRT and REMANI. '
                        'Usually /odometry/filtered with EKF.'),
        DeclareLaunchArgument(
            'joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument(
            'static_esdf_file', default_value=default_esdf,
            description='REMANI-format ESDF NPZ. For real use pass the '
                        'nvblox exported d455_bag_remani_esdf.npz.'),
        DeclareLaunchArgument('static_esdf_offset_x', default_value='0.0'),
        DeclareLaunchArgument('static_esdf_offset_y', default_value='0.0'),
        DeclareLaunchArgument('static_esdf_offset_z', default_value='0.0'),
        DeclareLaunchArgument('planner_to_ocs2_x', default_value='0.0'),
        DeclareLaunchArgument('planner_to_ocs2_y', default_value='0.0'),
        DeclareLaunchArgument('planner_to_ocs2_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'tracking_error_replan_enabled', default_value='false',
            description='Start with false during real-robot commissioning.'),
        DeclareLaunchArgument(
            'manipulator_max_vel', default_value='0.3'),
        DeclareLaunchArgument(
            'manipulator_max_acc', default_value='0.5'),
        DeclareLaunchArgument(
            'freeze_manipulator', default_value='false'),
        DeclareLaunchArgument(
            'start_arm_pose', default_value='false',
            description='Must be false when the real JAKA driver publishes '
                        '/joint_states.'),
        DeclareLaunchArgument('start_imu', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
    ]

    # ------------------------------------------------------------------
    # 1. Real localization/SLAM bringup
    #    - Starts Tracer base, RSP, IMU, LiDAR, EKF and slam_toolbox.
    #    - Does NOT start the fake arm_pose_publisher.
    #    - Does NOT start the OCS2 RViz (ocs2_real does that).
    # ------------------------------------------------------------------
    real_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            tracer_mujoco_share, 'launch', 'real_slam.launch.py')),
        launch_arguments={
            'start_base': 'true',
            'start_robot_state_publisher': 'true',
            'start_arm_pose': start_arm_pose,
            'start_imu': start_imu,
            'start_lidar': start_lidar,
            'rviz': 'false',
            'can_port': can_port,
            'serial_port': serial_port,
            'wheel_odom_topic': '/odom',
            'imu_topic': '/IMU_data',
            'scan_topic': '/scan',
        }.items(),
    )

    # ------------------------------------------------------------------
    # 2. Real JAKA ros2_control + OCS2 MPC/MRT
    #    - Base and RSP are owned by real_slam, so disable them here.
    #    - MRT reads the same odometry as REMANI.
    #    - use_joy is false so the REMANI bridge is the only target source.
    # ------------------------------------------------------------------
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

    # ------------------------------------------------------------------
    # 3. REMANI planner + REMANI->OCS2 bridge
    #    Delayed so EKF, SLAM, controllers and OCS2 have time to initialize.
    # ------------------------------------------------------------------
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
            'odom_topic': odom_topic,
            'joint_state_topic': joint_state_topic,
            'planner_to_ocs2_x': planner_to_ocs2_x,
            'planner_to_ocs2_y': planner_to_ocs2_y,
            'planner_to_ocs2_yaw': planner_to_ocs2_yaw,
            'tracking_error_replan_enabled':
                tracking_error_replan_enabled,
            'manipulator_max_vel': manipulator_max_vel,
            'manipulator_max_acc': manipulator_max_acc,
            'freeze_manipulator': freeze_manipulator,
        }.items(),
    )

    remani_delayed = TimerAction(
        period=12.0,
        actions=[remani_launch],
    )

    return LaunchDescription(
        declare_args +
        [
            real_slam_launch,
            ocs2_real_launch,
            remani_delayed,
        ]
    )
