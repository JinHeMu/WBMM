#!/usr/bin/env python3
"""Compose the real hardware interface with the shared OCS2 algorithms.

``hardware_write`` is the only real-motion gate:
  * true  -> JAKA writes and OCS2 command output are enabled;
  * false -> JAKA remains telemetry-only and OCS2 command output is disabled.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("tracer_jaka_bringup")
    description_share = FindPackageShare("tracer_jaka_description")
    ocs2_share = FindPackageShare("wbmm_ocs2_ros")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "mrt_odom_topic", default_value="/odom",
            description=(
                "Odometry consumed by MRT. Use /odometry/filtered when an "
                "EKF is running.")),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument(
            "publish_odom_tf", default_value="true",
            description=(
                "Publish tracer_base odom TF. Set false when "
                "robot_localization owns it.")),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument(
            "hardware_write", default_value="false",
            description=(
                "Only real-motion gate: true enables OCS2 command output, "
                "false keeps JAKA and OCS2 telemetry-only.")),
        DeclareLaunchArgument("start_ocs2", default_value="true"),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                ocs2_share, "config", "task_real.info",
            ])),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description_share, "urdf", "tracer_jaka_zu5.urdf",
            ])),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_real/auto_generated"),
        DeclareLaunchArgument(
            "ocs2_config", default_value=PathJoinSubstitution([
                ocs2_share, "config", "ocs2_real.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch",
                "hardware_interface.launch.py",
            ])),
            launch_arguments={
                "start_base": LaunchConfiguration("start_base"),
                "start_robot_state_publisher": LaunchConfiguration(
                    "start_robot_state_publisher"),
                "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
                "can_port": LaunchConfiguration("can_port"),
                "jaka_robot_ip": LaunchConfiguration("robot_ip"),
                "jaka_local_ip": LaunchConfiguration("local_ip"),
                "hardware_write": LaunchConfiguration("hardware_write"),
                "start_arm_pose": "false",
                "start_imu": "false",
                "start_lidar": "false",
                "start_jaka_hardware": "true",
                "start_jaka_fts": "true",
                "start_arm_controller": "true",
                "arm_controller_name": "arm_controller",
            }.items(),
        ),
        TimerAction(period=10.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "ocs2.launch.py",
            ])),
            condition=IfCondition(LaunchConfiguration("start_ocs2")),
            launch_arguments={
                "use_sim_time": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "config_file": LaunchConfiguration("ocs2_config"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "lib_folder": LaunchConfiguration("lib_folder"),
                "odom_topic": LaunchConfiguration("mrt_odom_topic"),
                "command_output_enabled": PythonExpression([
                    "'true' if '", LaunchConfiguration("hardware_write"),
                    "'.lower() == 'true' else 'false'",
                ]),
            }.items())]),
    ])
