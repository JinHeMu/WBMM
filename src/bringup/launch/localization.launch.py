#!/usr/bin/env python3
"""Start only the shared EKF and optional SLAM algorithms.

Hardware and simulation sources are composed by the launch files in
``launch/real`` and ``launch/sim``. This file only receives their topics and
parameter files.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_nodes(context):
    start_ekf = _as_bool(_value(context, "start_ekf"))
    start_slam = _as_bool(_value(context, "start_slam"))
    ekf_config = _value(context, "ekf_config")
    slam_config = _value(context, "slam_config")

    if start_ekf and not ekf_config:
        raise RuntimeError(
            "ekf_config must be provided by the deployment launch")
    if start_slam and not slam_config:
        raise RuntimeError(
            "slam_config must be provided by the deployment launch")
    if start_ekf and not os.path.isfile(ekf_config):
        raise RuntimeError(f"EKF config does not exist: {ekf_config!r}")
    if start_slam and not os.path.isfile(slam_config):
        raise RuntimeError(f"SLAM config does not exist: {slam_config!r}")

    use_sim_time = _as_bool(_value(context, "use_sim_time"))
    wheel_odom_topic = _value(context, "wheel_odom_topic")
    imu_topic = _value(context, "imu_topic")
    scan_topic = _value(context, "scan_topic")

    # Keep the EKF independent of the producing backend while enforcing the
    # canonical WBMM hardware interface spellings.
    ekf_remappings = [
        ("/wheel/odometry", wheel_odom_topic),
        ("/imu/data", imu_topic),
    ]

    return [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=IfCondition(str(start_ekf).lower()),
            parameters=[ekf_config, {"use_sim_time": use_sim_time}],
            remappings=ekf_remappings,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=IfCondition(str(start_slam).lower()),
            parameters=[slam_config, {"use_sim_time": use_sim_time}],
            remappings=[("/scan", scan_topic)],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("ekf_config", default_value=""),
        DeclareLaunchArgument("slam_config", default_value=""),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        OpaqueFunction(function=_make_nodes),
    ])
