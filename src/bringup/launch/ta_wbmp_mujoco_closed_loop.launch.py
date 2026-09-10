#!/usr/bin/env python3
"""Unified YAML -> TA-WBMP -> REMANI -> MPC(+optional force) -> MuJoCo."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    planner_share = get_package_share_directory('ta_wbmp')
    bringup_share = get_package_share_directory('tracer_jaka_bringup')
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    description_share = get_package_share_directory('tracer_jaka_description')
    grid_map_share = get_package_share_directory('grid_map')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_share, 'launch', 'ocs2_sim.launch.py')),
        launch_arguments={
            'viewer': LaunchConfiguration('viewer'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'start_slam': 'false',
            'start_remani': 'true',
            # During NAVIGATING the bridge owns the MPC reference. After the
            # measured 9D q_pre arrival, the coordinator calls
            # /remani_planner/set_task_execution and becomes the owner for the
            # approach/contact reference.
            'start_remani_bridge': 'true',
            'use_joy': 'false',
            'use_csv_target': 'false',
            'mujoco_model': os.path.join(
                mujoco_share, 'models', 'scene_task_table.xml'),
            'init_keyframe': 'home',
            'remani_static_esdf_file': os.path.join(
                grid_map_share, 'maps', 'task_table_esdf.npz'),
            'remani_static_esdf_offset_x': '0.0',
            'remani_static_esdf_offset_y': '0.0',
            'remani_static_esdf_offset_z': '0.0',
            'map_to_odom_x': '0.0',
            'remani_freeze_manipulator': 'false',
            'remani_manipulator_max_vel': '0.55',
            'remani_manipulator_max_acc': '1.00',
            'remani_manipulator_safe_margin': '0.02',
            'remani_general_safe_margin': '0.02',
            'remani_self_safe_margin': '0.02',
            'remani_tracking_error_replan_enabled': 'false',
            'remani_goal_replan_position_threshold': '0.40',
            'remani_goal_replan_yaw_threshold': '0.70',
            'remani_goal_replan_joint_threshold': '0.45',
            'remani_goal_replan_persistence': '1.0',
            'remani_max_consecutive_planning_failures': '20',
            'mrt_traj_horizon': '0.02',
            'arm_use_velocity_integrator': 'true',
            'arm_max_command_velocity': '0.25',
            'arm_max_delta_per_step': '0.10',
        }.items(),
    )

    coordinator = TimerAction(period=20.0, actions=[Node(
        package='ta_wbmp',
        executable='ta_wbmp_execution_coordinator_node',
        name='ta_wbmp_execution_coordinator',
        output='screen',
        parameters=[{
            # Do not call this top-level argument ``task_file``: the included
            # OCS2 launch reserves that name for its Boost .info controller
            # configuration.  The coordinator consumes YAML; OCS2 must keep
            # consuming tracer_jaka_ocs2/config/task.info.
            'task_file': LaunchConfiguration('task_yaml'),
            'urdf_file': os.path.join(
                description_share, 'urdf', 'tracer_jaka_zu5.urdf'),
            'ee_frame': 'tool0',
            'control_frame': 'odom',
            'execution_enabled': True,
            'auto_start': ParameterValue(
                LaunchConfiguration('auto_start'), value_type=bool),
            'force_control_enabled': ParameterValue(
                LaunchConfiguration('force_control_enabled'), value_type=bool),
        }],
    )])

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument(
            'force_control_enabled', default_value='false',
            description='Enable YAML-configured online force correction.'),
        DeclareLaunchArgument(
            'task_yaml', default_value=os.path.join(
                planner_share, 'config', 'mujoco_table_task.yaml'),
            description='Unified task YAML consumed directly by TA-WBMP.'),
        simulation,
        coordinator,
    ])
