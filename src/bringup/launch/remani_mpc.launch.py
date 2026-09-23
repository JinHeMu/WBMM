#!/usr/bin/env python3
"""REMANI + OCS2 algorithm entry point.

No real hardware or MuJoCo backend is started here. The caller must already
run a separate hardware backend.
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


def _enforce_safety_gate(context):
    esdf_file = context.perform_substitution(
        LaunchConfiguration("static_esdf_file"))
    if not esdf_file or not os.path.isfile(esdf_file):
        raise RuntimeError(
            "REMANI requires an explicit existing static_esdf_file")
    return []


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument(
            "static_esdf_file", default_value="",
            description=(
                "REMANI-format ESDF NPZ for the current site/model.")),
        DeclareLaunchArgument(
            "esdf_file", default_value="",
            description=(
                "Optional OCS2 environmentCollision ESDF NPZ; empty uses "
                "the task file.")),
        DeclareLaunchArgument(
            "world_frame", default_value="",
            description="Optional OCS2 world frame override."),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "wheel_odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument(
            "imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description, "urdf", "tracer_jaka_zu5.urdf"])),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "task.info"])),
        DeclareLaunchArgument(
            "ocs2_config", default_value="",
            description="Optional override; empty uses common defaults."),
        DeclareLaunchArgument(
            "remani_config",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "remani.yaml"])),
        DeclareLaunchArgument(
            "ekf_config",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "ekf.yaml"])),
        DeclareLaunchArgument(
            "slam_config", default_value="",
            description="Optional override; empty uses common defaults."),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_auto_generated"),
        DeclareLaunchArgument("planner_frame", default_value="odom"),
        DeclareLaunchArgument("target_frame", default_value="odom"),
        DeclareLaunchArgument("use_tf_transform", default_value="false"),
        OpaqueFunction(function=_enforce_safety_gate),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "localization.launch.py",
            ])),
            launch_arguments={
                "start_ekf": "true",
                "start_slam": LaunchConfiguration("start_slam"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ekf_config": LaunchConfiguration("ekf_config"),
                "slam_config": LaunchConfiguration("slam_config"),
                "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            }.items(),
        ),
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "ocs2.launch.py",
            ])),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "config_file": LaunchConfiguration("ocs2_config"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "lib_folder": LaunchConfiguration("lib_folder"),
                "command_output_enabled": LaunchConfiguration(
                    "hardware_write"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "esdf_file": LaunchConfiguration("esdf_file"),
                "world_frame": LaunchConfiguration("world_frame"),
            }.items())]),
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "remani.launch.py",
            ])),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_bridge": "true",
                "config_file": LaunchConfiguration("remani_config"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "static_esdf_file": LaunchConfiguration("static_esdf_file"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "planner_frame": LaunchConfiguration("planner_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "use_tf_transform": LaunchConfiguration("use_tf_transform"),
            }.items())]),
    ])
