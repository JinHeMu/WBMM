#!/usr/bin/env python3
"""Start the shared REMANI planner and its OCS2 reference bridge.

The static ESDF is already expressed in ``planner_frame``. When the planner
and controller frames differ, the bridge must use TF; fixed translation/yaw
offset launch arguments are intentionally not part of this interface.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _stringify(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def _load_config(path):
    if not path:
        return {}
    if not os.path.isfile(path):
        raise RuntimeError(f"REMANI config does not exist: {path}")
    with open(path, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"REMANI config must be a flat mapping: {path}")
    return {str(key): _stringify(value) for key, value in data.items()}


def _make_include(context):
    planner_frame = _value(context, "planner_frame")
    target_frame = _value(context, "target_frame")
    use_tf_transform = _as_bool(_value(context, "use_tf_transform"))
    if planner_frame != target_frame and not use_tf_transform:
        raise RuntimeError(
            "planner_frame and target_frame differ; use_tf_transform must "
            "be true instead of applying a fixed launch-file offset")

    urdf_file = _value(context, "urdf_file")
    static_esdf_file = _value(context, "static_esdf_file")
    if not urdf_file:
        raise RuntimeError(
            "urdf_file must be provided by the deployment launch")
    if not os.path.isfile(urdf_file):
        raise RuntimeError(f"REMANI urdf_file does not exist: {urdf_file!r}")
    if not static_esdf_file:
        raise RuntimeError(
            "static_esdf_file must be provided by the deployment launch")
    if not os.path.isfile(static_esdf_file):
        raise RuntimeError(
            f"REMANI static_esdf_file does not exist: {static_esdf_file!r}")

    runtime_args = {
        "use_sim_time": _value(context, "use_sim_time"),
        "start_planner": _value(context, "start_planner"),
        "start_bridge": _value(context, "start_bridge"),
        "urdf_file": urdf_file,
        "static_esdf_file": static_esdf_file,
        "odom_topic": _value(context, "odom_topic"),
        "joint_state_topic": _value(context, "joint_state_topic"),
        "planner_frame": planner_frame,
        "target_frame": target_frame,
        "use_tf_transform": str(use_tf_transform).lower(),
    }
    base_config = _value(context, "base_config_file")
    if not os.path.isfile(base_config):
        raise RuntimeError(f"REMANI base config does not exist: {base_config!r}")
    tuning_args = {}
    tuning_args.update(_load_config(base_config))
    tuning_args.update(_load_config(_value(context, "config_file")))
    remani_share = get_package_share_directory("remani_planner")

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            remani_share, "launch", "remani_mpc_tracking.launch.py")),
        launch_arguments={**tuning_args, **runtime_args}.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_planner", default_value="true"),
        DeclareLaunchArgument("start_bridge", default_value="true"),
        DeclareLaunchArgument("urdf_file", default_value=""),
        DeclareLaunchArgument("static_esdf_file", default_value=""),
        DeclareLaunchArgument(
            "odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("planner_frame", default_value="odom"),
        DeclareLaunchArgument("target_frame", default_value="odom"),
        DeclareLaunchArgument("use_tf_transform", default_value="false"),
        DeclareLaunchArgument(
            "base_config_file",
            default_value=os.path.join(
                get_package_share_directory("tracer_jaka_bringup"),
                "config", "common", "remani.yaml")),
        DeclareLaunchArgument("config_file", default_value=""),
        OpaqueFunction(function=_make_include),
    ])
