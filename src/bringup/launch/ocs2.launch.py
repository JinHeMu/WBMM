#!/usr/bin/env python3
"""Start the shared OCS2 MPC/MRT algorithm nodes.

This launch has no hardware or simulator knowledge. A deployment launch owns
the state sources, command consumers, safety gate and parameter profile.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_nodes(context):
    config_file = _value(context, "config_file")
    task_file = _value(context, "task_file")
    urdf_file = _value(context, "urdf_file")
    lib_folder = _value(context, "lib_folder")

    for label, path in (
        ("OCS2 config_file", config_file),
        ("OCS2 task_file", task_file),
        ("OCS2 urdf_file", urdf_file),
    ):
        if not path:
            raise RuntimeError(
                f"{label} must be provided by the deployment launch")
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path!r}")
    if not lib_folder:
        raise RuntimeError(
            "OCS2 lib_folder must be provided by the deployment launch")

    use_sim_time = _as_bool(_value(context, "use_sim_time"))
    use_target = _as_bool(_value(context, "use_target"))
    use_rviz = _as_bool(_value(context, "use_rviz"))
    command_output_enabled = _as_bool(
        _value(context, "command_output_enabled"))

    common_parameters = {
        "taskFile": task_file,
        "urdfFile": urdf_file,
        "use_sim_time": use_sim_time,
    }
    nodes = [
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mpc_node",
            name="wbmm_mpc_node",
            output="screen",
            parameters=[config_file, {
                **common_parameters,
                "libFolder": os.path.join(lib_folder, "mpc"),
            }],
        ),
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mrt_node",
            name="wbmm_mrt_node",
            output="screen",
            parameters=[config_file, {
                **common_parameters,
                "libFolder": os.path.join(lib_folder, "mrt"),
                "command_output_enabled": command_output_enabled,
                "odom_topic": _value(context, "odom_topic"),
            }],
        ),
    ]

    if use_target:
        nodes.append(Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_target_node",
            name="wbmm_target_node",
            output="screen",
            parameters=[config_file, {"use_sim_time": use_sim_time}],
        ))

    if use_rviz:
        nodes.append(Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", _value(context, "rviz_config")],
            parameters=[{"use_sim_time": use_sim_time}],
        ))

    return nodes


def generate_launch_description():
    ocs2_share = get_package_share_directory("wbmm_ocs2_ros")

    return LaunchDescription([
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("task_file", default_value=""),
        DeclareLaunchArgument("urdf_file", default_value=""),
        DeclareLaunchArgument("lib_folder", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_target", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "command_output_enabled", default_value="false"),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                ocs2_share, "rviz", "wbmm_ocs2_ros.rviz")),
        OpaqueFunction(function=_make_nodes),
    ])
