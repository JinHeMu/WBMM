#!/usr/bin/env python3
"""REMANI + OCS2 with saved-map AMCL localization.

This launch starts algorithms only. Hardware or MuJoCo feedback must come from
a separate hardware backend.
"""

import os

import numpy as np
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _enforce_safety_gate(context):
    map_file = context.perform_substitution(LaunchConfiguration("map_file"))
    if not os.path.isfile(map_file):
        raise RuntimeError(f"Localization map does not exist: {map_file}")

    start_remani = _as_bool(context.perform_substitution(
        LaunchConfiguration("start_remani")))
    if start_remani:
        esdf_file = context.perform_substitution(
            LaunchConfiguration("static_esdf_file"))
        if not esdf_file or not os.path.isfile(esdf_file):
            raise RuntimeError(
                "start_remani:=true requires an existing static_esdf_file")
        try:
            with np.load(esdf_file, allow_pickle=False) as archive:
                if "frame_id" not in archive.files:
                    raise RuntimeError(
                        "Static ESDF is missing required frame_id metadata")
                frame_value = archive["frame_id"]
                if frame_value.shape != ():
                    raise RuntimeError(
                        "Static ESDF frame_id metadata must be scalar")
                esdf_frame = str(frame_value.item())
        except RuntimeError:
            raise
        except Exception as exception:
            raise RuntimeError(
                f"Cannot read static ESDF contract from {esdf_file}: "
                f"{exception}") from exception
        if esdf_frame != "map":
            raise RuntimeError(
                "Localized pipeline requires static ESDF frame_id=map, "
                f"but archive declares frame_id={esdf_frame!r}")
    return []


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")
    localization = FindPackageShare("tracer_jaka_localization")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("start_ocs2", default_value="true"),
        DeclareLaunchArgument("start_remani", default_value="true"),
        DeclareLaunchArgument("start_bridge", default_value="true"),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "task.info"])),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description, "urdf", "tracer_jaka_zu5.urdf"])),
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
            "lib_folder", default_value="/tmp/wbmm_ocs2_auto_generated"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "map_odom_topic", default_value="/odometry/filtered_map"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "wheel_odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "static_esdf_file",
            default_value="/home/a/WBMM/maps/map1/site_remani.npz",
            description=(
                "REMANI-format ESDF for map1; frame_id must be map.")),
        DeclareLaunchArgument(
            "esdf_file", default_value="",
            description=(
                "Optional OCS2 environmentCollision ESDF NPZ; empty uses "
                "the task file.")),
        DeclareLaunchArgument(
            "world_frame", default_value="",
            description="Optional OCS2 world frame override."),
        DeclareLaunchArgument(
            "map_file",
            default_value="/home/a/WBMM/maps/map1/site_2d.yaml",
            description="2D map matching the map1 ESDF."),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        OpaqueFunction(function=_enforce_safety_gate),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "localization.launch.py",
            ])),
            launch_arguments={
                "start_ekf": "true",
                "start_slam": "false",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "ekf_config": LaunchConfiguration("ekf_config"),
                "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                localization, "launch", "amcl_localization.launch.py",
            ])),
            launch_arguments={
                "map_file": LaunchConfiguration("map_file"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }.items(),
        ),
        Node(
            package="tracer_jaka_bringup",
            executable="odom_to_map_relay.py",
            name="odom_to_map_relay",
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "output_topic": LaunchConfiguration("map_odom_topic"),
                "map_frame": "map",
                "odom_frame": "odom",
                "child_frame": "base_footprint",
            }],
        ),
        IncludeLaunchDescription(
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
            }.items(),
        ),
        TimerAction(period=15.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "remani.launch.py",
            ])),
            condition=IfCondition(LaunchConfiguration("start_remani")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_bridge": LaunchConfiguration("start_bridge"),
                "config_file": LaunchConfiguration("remani_config"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "static_esdf_file": LaunchConfiguration("static_esdf_file"),
                "odom_topic": LaunchConfiguration("map_odom_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "planner_frame": "map",
                "target_frame": "odom",
                "use_tf_transform": "true",
            }.items())]),
    ])
