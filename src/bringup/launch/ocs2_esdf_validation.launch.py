#!/usr/bin/env python3
"""End-to-end OCS2 ESDF collision validation entry point.

This launches MuJoCo (room scene), OCS2 MPC/MRT and the static ESDF collision
backend.  The ESDF file must be expressed in the OCS2 world frame (odom by
default); the C++ interface refuses to query an ESDF in a different frame.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_actions(context):
    bringup = get_package_share_directory("tracer_jaka_bringup")
    grid_map = get_package_share_directory("grid_map")

    task_file = _value(context, "task_file")
    if not task_file:
        task_file = os.path.join(
            bringup, "config", "sim", "task_esdf.info")

    esdf_file = _value(context, "esdf_file")
    if not esdf_file:
        esdf_file = os.path.join(
            grid_map, "maps", "tracer_jaka_zu5_scene_esdf.npz")

    for label, path in (
        ("task_esdf file", task_file),
        ("ESDF file", esdf_file),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path!r}")

    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("tracer_jaka_bringup"),
            "launch", "wbmm.launch.py",
        ])),
        launch_arguments={
            "hardware_backend": "mujoco",
            "config_profile": "sim",
            "scene": _value(context, "scene"),
            "viewer": _value(context, "viewer"),
            "use_rviz": _value(context, "use_rviz"),
            "hardware_write": _value(context, "hardware_write"),
            "start_ocs2": "true",
            "start_remani": "false",
            "start_force_control": "false",
            "start_moveit": "false",
            "task_file": task_file,
            "esdf_file": esdf_file,
            "world_frame": _value(context, "world_frame"),
            "lib_folder": _value(context, "lib_folder"),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="room",
            description=(
                "MuJoCo scene; 'room' matches the installed "
                "tracer_jaka_zu5_scene_esdf.npz map.")),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument(
            "world_frame", default_value="odom",
            description="OCS2 state/world frame; must match the ESDF NPZ frame_id."),
        DeclareLaunchArgument(
            "task_file", default_value="",
            description="Defaults to installed config/sim/task_esdf.info."),
        DeclareLaunchArgument(
            "esdf_file", default_value="",
            description=(
                "Defaults to installed grid_map/maps/tracer_jaka_zu5_scene_esdf.npz.")),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_esdf_validation"),
        OpaqueFunction(function=_make_actions),
    ])
