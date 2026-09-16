#!/usr/bin/env python3
"""Real-robot REMANI + OCS2 MPC/MRT with slam_toolbox mapping.

Composes the real localization/controller launches with the shared REMANI
algorithm launch. REMANI tuning defaults to the conservative real profile.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _enforce_safety_gate(context):
    esdf_file = context.perform_substitution(
        LaunchConfiguration("static_esdf_file"))
    if not esdf_file or not os.path.isfile(esdf_file):
        raise RuntimeError(
            "Real REMANI requires an explicit existing static_esdf_file; "
            "a simulation/demo ESDF is never selected by default")
    return []


def generate_launch_description():
    bringup_share = FindPackageShare("tracer_jaka_bringup")
    description_share = FindPackageShare("tracer_jaka_description")
    ocs2_share = FindPackageShare("wbmm_ocs2_ros")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
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
            "odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "static_esdf_file", default_value="",
            description=(
                "Required REMANI-format ESDF NPZ from the current real site.")),
        DeclareLaunchArgument(
            "remani_config", default_value=PathJoinSubstitution([
                bringup_share, "config", "real", "remani_real.yaml"])),
        DeclareLaunchArgument("start_arm_pose", default_value="false"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        OpaqueFunction(function=_enforce_safety_gate),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "real_slam.launch.py",
            ])),
            launch_arguments={
                "rviz": "false",
                "start_base": "true",
                "start_robot_state_publisher": "true",
                "start_arm_pose": LaunchConfiguration("start_arm_pose"),
                "start_imu": LaunchConfiguration("start_imu"),
                "start_lidar": LaunchConfiguration("start_lidar"),
                "can_port": LaunchConfiguration("can_port"),
                "serial_port": LaunchConfiguration("serial_port"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "ocs2_real.launch.py"])),
            launch_arguments={
                "start_base": "false",
                "start_robot_state_publisher": "false",
                "publish_odom_tf": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "local_ip": LaunchConfiguration("local_ip"),
                "hardware_write": LaunchConfiguration("hardware_write"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "lib_folder": LaunchConfiguration("lib_folder"),
                "mrt_odom_topic": LaunchConfiguration("odom_topic"),
            }.items(),
        ),
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "remani.launch.py"])),
            launch_arguments={
                "use_sim_time": "false",
                "config_file": LaunchConfiguration("remani_config"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "static_esdf_file": LaunchConfiguration("static_esdf_file"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "planner_frame": "odom",
                "target_frame": "odom",
                "use_tf_transform": "false",
            }.items())]),
    ])
