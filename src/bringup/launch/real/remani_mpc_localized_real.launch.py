#!/usr/bin/env python3
"""Real-robot REMANI + OCS2 MPC with saved-map AMCL localization.

Composes real feedback, AMCL localization and the shared algorithm launches.
The fail-closed safety gate and map/ESDF contract checks stay in this wrapper
because they are real-motion deployment requirements.
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
    """Fail closed before constructing any real command-producing nodes."""
    map_file = context.perform_substitution(LaunchConfiguration("map_file"))
    if not os.path.isfile(map_file):
        raise RuntimeError(
            f"Real localization map does not exist: {map_file}")

    start_remani = _as_bool(context.perform_substitution(
        LaunchConfiguration("start_remani")))
    if start_remani:
        esdf_file = context.perform_substitution(
            LaunchConfiguration("static_esdf_file"))
        if not esdf_file or not os.path.isfile(esdf_file):
            raise RuntimeError(
                "start_remani:=true requires an existing static_esdf_file. "
                "Pass the site-specific .npz explicitly; no unsafe default "
                "environment map is selected.")
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
                "Localized real pipeline requires static ESDF frame_id=map, "
                f"but archive declares frame_id={esdf_frame!r}. Re-export or "
                "transform the ESDF; do not relabel it.")
    return []


def generate_launch_description():
    bringup_share = FindPackageShare("tracer_jaka_bringup")
    description_share = FindPackageShare("tracer_jaka_description")
    localization_share = FindPackageShare("tracer_jaka_localization")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("lidar_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument(
            "lidar_sensor_ip", default_value="192.168.198.2"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("start_ocs2", default_value="true"),
        DeclareLaunchArgument("start_remani", default_value="true"),
        DeclareLaunchArgument("start_bridge", default_value="true"),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("wbmm_ocs2_ros"),
                "config", "task_real_conservative.info",
            ])),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description_share, "urdf", "tracer_jaka_zu5.urdf",
            ])),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_real/auto_generated"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "map_odom_topic", default_value="/odometry/filtered_map"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("static_esdf_file", default_value=""),
        DeclareLaunchArgument("start_arm_pose", default_value="false"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "map_file",
            default_value=PathJoinSubstitution([
                localization_share, "maps", "factory_map.yaml",
            ])),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "remani_config", default_value=PathJoinSubstitution([
                bringup_share, "config", "real", "remani_real.yaml"])),
        OpaqueFunction(function=_enforce_safety_gate),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "real_slam.launch.py",
            ])),
            launch_arguments={
                "start_slam": "false",
                "rviz": "false",
                "start_base": "true",
                "start_robot_state_publisher": "true",
                "start_arm_pose": LaunchConfiguration("start_arm_pose"),
                "start_imu": LaunchConfiguration("start_imu"),
                "start_lidar": LaunchConfiguration("start_lidar"),
                "can_port": LaunchConfiguration("can_port"),
                "serial_port": LaunchConfiguration("serial_port"),
                "lidar_host_ip": LaunchConfiguration("lidar_host_ip"),
                "lidar_sensor_ip": LaunchConfiguration("lidar_sensor_ip"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                localization_share, "launch", "localization_real.launch.py"])),
            launch_arguments={
                "map_file": LaunchConfiguration("map_file"),
                "initial_x": LaunchConfiguration("initial_x"),
                "initial_y": LaunchConfiguration("initial_y"),
                "initial_yaw": LaunchConfiguration("initial_yaw"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "use_sim_time": "false",
            }.items(),
        ),
        Node(
            package="tracer_jaka_bringup", executable="odom_to_map_relay.py",
            name="odom_to_map_relay", output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "output_topic": LaunchConfiguration("map_odom_topic"),
                "map_frame": "map", "odom_frame": "odom",
                "child_frame": "base_footprint",
            }]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "ocs2_real.launch.py"])),
            launch_arguments={
                "start_base": "false",
                "start_robot_state_publisher": "false",
                "publish_odom_tf": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_ocs2": LaunchConfiguration("start_ocs2"),
                "robot_ip": LaunchConfiguration("robot_ip"),
                "local_ip": LaunchConfiguration("local_ip"),
                "hardware_write": LaunchConfiguration("hardware_write"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "lib_folder": LaunchConfiguration("lib_folder"),
                "mrt_odom_topic": LaunchConfiguration("odom_topic"),
            }.items(),
        ),
        TimerAction(period=15.0, actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "remani.launch.py"])),
            condition=IfCondition(LaunchConfiguration("start_remani")),
            launch_arguments={
                "use_sim_time": "false",
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
