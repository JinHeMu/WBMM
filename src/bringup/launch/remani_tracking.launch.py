#!/usr/bin/env python3
"""REMANI whole-body navigation + OCS2 end-effector tracking demo.

Composition:
  MuJoCo empty scene (odom aliased to map by an identity map->odom TF)
    -> map1 static ESDF for REMANI and OCS2
    -> REMANI whole-body navigation from RViz 2D Goal
    -> WbmmTargetNode interactive 3D marker for EE pose
    -> phase bridge: new goal => Navigation, finish stays Navigation,
       fresh EE target => Execution directly
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_actions(context):
    bringup = get_package_share_directory("tracer_jaka_bringup")

    task_file = _value(context, "task_file")
    if not task_file:
        task_file = os.path.join(
            bringup, "config", "sim", "task_esdf_tracking.info")

    static_esdf_file = _value(context, "static_esdf_file")
    if not static_esdf_file:
        static_esdf_file = "/home/a/WBMM/maps/map1/site_remani.npz"

    esdf_file = _value(context, "esdf_file")
    if not esdf_file:
        esdf_file = static_esdf_file

    remani_config = _value(context, "remani_config")
    if not remani_config:
        remani_config = os.path.join(
            bringup, "config", "sim", "remani_tracking.yaml")

    for label, path in (
        ("task_esdf_tracking file", task_file),
        ("REMANI static ESDF file", static_esdf_file),
        ("OCS2 ESDF file", esdf_file),
        ("REMANI tracking config", remani_config),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path!r}")

    world_frame = _value(context, "world_frame")
    rviz_config = _value(context, "rviz_config")
    if rviz_config and not os.path.isfile(rviz_config):
        raise RuntimeError(f"RViz config does not exist: {rviz_config!r}")

    wbmm_args = {
        "hardware_backend": "mujoco",
        "config_profile": "sim",
        "scene": _value(context, "scene"),
        "viewer": _value(context, "viewer"),
        "publish_odom_tf": "true",
        "use_rviz": _value(context, "use_rviz"),
        "hardware_write": _value(context, "hardware_write"),
        "start_localization": "false",
        "start_ocs2": "true",
        "start_remani": "true",
        "start_force_control": "false",
        "start_moveit": "false",
        "use_target": "true",
        "task_file": task_file,
        "static_esdf_file": static_esdf_file,
        "esdf_file": esdf_file,
        "world_frame": world_frame,
        "remani_config": remani_config,
        "remani_planner_frame": "map",
        "remani_target_frame": "map",
        "remani_use_tf_transform": "false",
        "odom_topic": _value(context, "odom_topic"),
        "joint_state_topic": _value(context, "joint_state_topic"),
        "lib_folder": _value(context, "lib_folder"),
    }
    if rviz_config:
        wbmm_args["rviz_config"] = rviz_config

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tracer_jaka_bringup"),
                "launch", "wbmm.launch.py",
            ])),
            launch_arguments=wbmm_args.items(),
        ),
        Node(
            package="grid_map",
            executable="esdf_rviz_publisher",
            name="esdf_rviz_publisher",
            output="screen",
            parameters=[{
                "esdf_file": esdf_file,
                "frame_id": world_frame,
                "z_slice": 0.5,
                "max_distance": 0.8,
                "stride": 2,
                "publish_period": 0.5,
            }],
        ),
        Node(
            package="tracer_jaka_bringup",
            executable="remani_phase_bridge.py",
            name="remani_phase_bridge",
            output="screen",
            parameters=[{
                "phase_service": "/mobile_manipulator_set_task_phase",
                "goal_topic": "/goal_pose",
                "finish_topic": "/planning/finish",
                "odom_topic": _value(context, "odom_topic"),
                "goal_position_tolerance": 0.15,
                "goal_yaw_tolerance": 0.30,
                "goal_hold_time": 1.0,
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(
                LaunchConfiguration("publish_map_odom_tf")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="esdf_validation",
            description=(
                "MuJoCo scene. Default is robot+floor only; ESDF supplies "
                "the virtual obstacles.")),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "hardware_write", default_value="true",
            description=(
                "MuJoCo command output. Keep true for full navigation; set "
                "false for a dry-run.")),
        DeclareLaunchArgument(
            "world_frame", default_value="map",
            description="Must match the ESDF NPZ frame_id."),
        DeclareLaunchArgument(
            "publish_map_odom_tf", default_value="true",
            description=(
                "Publish an identity map->odom TF for this odom-aliased "
                "MuJoCo demo.")),
        DeclareLaunchArgument(
            "task_file", default_value="",
            description=(
                "Defaults to installed config/sim/task_esdf_tracking.info.")),
        DeclareLaunchArgument(
            "static_esdf_file",
            default_value="/home/a/WBMM/maps/map1/site_remani.npz",
            description="REMANI ESDF; defaults to map1."),
        DeclareLaunchArgument(
            "remani_config", default_value="",
            description=(
                "REMANI profile; defaults to installed "
                "config/sim/remani_tracking.yaml.")),
        DeclareLaunchArgument(
            "esdf_file", default_value="",
            description="OCS2 ESDF; defaults to static_esdf_file."),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("tracer_jaka_bringup"),
                "rviz", "remani_tracking_map1.rviz"]),
            description=(
                "RViz config for the map1 REMANI tracking demo.")),
        DeclareLaunchArgument(
            "odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_esdf_tracking"),
        OpaqueFunction(function=_make_actions),
    ])
