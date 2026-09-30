#!/usr/bin/env python3
"""Start the shared EKF and one selectable mapping/localization backend.

Hardware and simulation sources are composed by the launch files in
``launch/real`` and ``launch/sim``. This file only receives their topics and
parameter files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from wbmm_localization_launch.launch_support import BACKENDS, resolve_backend
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_nodes(context):
    start_ekf = _as_bool(_value(context, "start_ekf"))
    backend = resolve_backend(_value(context, "localization_backend"),
                              _as_bool(_value(context, "start_slam")))
    start_slam = backend == "slam_toolbox"
    ekf_base_config = _value(context, "ekf_base_config")
    slam_base_config = _value(context, "slam_base_config")
    ekf_config = _value(context, "ekf_config")
    slam_config = _value(context, "slam_config")

    if start_ekf and not os.path.isfile(ekf_base_config):
        raise RuntimeError(
            f"EKF base config does not exist: {ekf_base_config!r}")
    if start_slam and not os.path.isfile(slam_base_config):
        raise RuntimeError(
            f"SLAM base config does not exist: {slam_base_config!r}")
    if ekf_config and not os.path.isfile(ekf_config):
        raise RuntimeError(f"EKF config does not exist: {ekf_config!r}")
    if start_slam and slam_config and not os.path.isfile(slam_config):
        raise RuntimeError(f"SLAM config does not exist: {slam_config!r}")

    ekf_layers = [ekf_base_config]
    if ekf_config:
        ekf_layers.append(ekf_config)
    slam_layers = [slam_base_config]
    if slam_config:
        slam_layers.append(slam_config)

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

    actions = [
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            condition=IfCondition(str(start_ekf).lower()),
            parameters=[*ekf_layers, {"use_sim_time": use_sim_time}],
            remappings=ekf_remappings,
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            condition=IfCondition(str(start_slam).lower()),
            parameters=[*slam_layers, {"use_sim_time": use_sim_time}],
            remappings=[("/scan", scan_topic)],
        ),
    ]
    if backend in ("amcl", "cartographer_mapping", "cartographer_localization"):
        share = get_package_share_directory("wbmm_localization")
        arguments = {
            "use_sim_time": str(use_sim_time).lower(),
            "scan_topic": scan_topic,
            "odom_topic": _value(context, "odom_topic"),
            "readiness_config": _value(context, "readiness_config"),
        }
        if backend == "amcl":
            map_file = _value(context, "map_file")
            if not os.path.isfile(map_file):
                raise RuntimeError(f"Localization map does not exist: {map_file!r}")
            arguments.update(map_file=map_file,
                             initial_x=_value(context, "initial_x"),
                             initial_y=_value(context, "initial_y"),
                             initial_yaw=_value(context, "initial_yaw"))
            entry = "amcl_localization.launch.py"
        else:
            arguments.update(imu_topic=imu_topic,
                             state_file=_value(context, "state_file"),
                             save_state_file=_value(context, "save_state_file"),
                             cartographer_config=_value(context, "cartographer_config"))
            entry = backend + ".launch.py"
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", entry)),
            launch_arguments=arguments.items()))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("start_ekf", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("localization_backend", default_value="auto", choices=list(BACKENDS),
            description="auto preserves start_slam; explicit backends own map -> odom exclusively."),
        DeclareLaunchArgument("map_file", default_value=os.path.join(
            get_package_share_directory("wbmm_localization"), "maps", "factory_map.yaml")),
        DeclareLaunchArgument("state_file", default_value=""),
        DeclareLaunchArgument("save_state_file", default_value=""),
        DeclareLaunchArgument("cartographer_config", default_value=""),
        DeclareLaunchArgument("readiness_config", default_value=os.path.join(
            get_package_share_directory("wbmm_localization"), "config", "readiness.yaml")),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "ekf_base_config",
            default_value=os.path.join(
                get_package_share_directory("tracer_jaka_bringup"),
                "config", "common", "ekf.yaml")),
        DeclareLaunchArgument(
            "slam_base_config",
            default_value=os.path.join(
                get_package_share_directory("tracer_jaka_bringup"),
                "config", "common", "slam_toolbox.yaml")),
        DeclareLaunchArgument("ekf_config", default_value=""),
        DeclareLaunchArgument("slam_config", default_value=""),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        OpaqueFunction(function=_make_nodes),
    ])
