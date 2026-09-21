#!/usr/bin/env python3
"""MuJoCo hardware interface for WBMM.

This is the simulation counterpart of ``wbmm_hardware_interface.launch.py``.
It starts the MuJoCo bridge and robot_state_publisher and exposes the canonical
WBMM ROS interface defined in ``config/common/interface.yaml``. It does not
start OCS2, REMANI, MoveIt, localization or force-control algorithms.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Named MuJoCo scenes installed under tracer_jaka_mujoco/models.
_SCENES = {
    "empty": "scene_empty.xml",
    "room": "scene.xml",
    "task_table": "scene_task_table.xml",
    "force_follow_infinite": "scene_force_follow_infinite.xml",
    "force_follow_5m": "scene_force_follow_5m.xml",
    "nvblox_remani_demo": "scene_nvblox_remani_demo.xml",
    "esdf_validation": "scene_esdf_validation.xml",
}


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name))


def _make_nodes(context):
    mujoco_share = get_package_share_directory("tracer_jaka_mujoco")
    description_share = get_package_share_directory("tracer_jaka_description")

    model = _value(context, "model")
    if not model:
        scene_name = _value(context, "scene")
        try:
            scene_file = _SCENES[scene_name]
        except KeyError as exc:
            raise RuntimeError(
                f"Unknown MuJoCo scene {scene_name!r}; "
                f"available scenes: {', '.join(sorted(_SCENES))}") from exc
        model = os.path.join(mujoco_share, "models", scene_file)
    sensors_yaml = os.path.join(mujoco_share, "config", "sensors.yaml")
    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")

    start_rsp = _as_bool(_value(context, "start_robot_state_publisher"))
    publish_odom_tf = _as_bool(_value(context, "publish_odom_tf"))
    start_imu = _as_bool(_value(context, "start_imu"))
    start_lidar = _as_bool(_value(context, "start_lidar"))
    start_camera = _as_bool(_value(context, "start_camera"))
    start_fts = _as_bool(_value(context, "start_fts"))
    viewer = _as_bool(_value(context, "viewer"))
    init_keyframe = _value(context, "init_keyframe").strip()
    if not init_keyframe:
        init_keyframe = _value(context, "initial_pose").strip()

    bridge_parameters = {
        "model_path": model,
        "use_sim_time": False,
        "use_viewer": viewer,
        "init_keyframe": init_keyframe,
        "odom_topic": _value(context, "wheel_odom_topic"),
        "publish_odom_tf": publish_odom_tf,
        "imu.enable": start_imu,
        "imu.topic": _value(context, "imu_topic"),
        "lidar.enable": start_lidar,
        "lidar.topic": _value(context, "scan_topic"),
        "fts.enable": start_fts,
        "fts.topic": _value(context, "fts_topic"),
        "camera.enable": start_camera,
        "camera.color_topic": _value(context, "color_image_topic"),
        "camera.color_info_topic": _value(
            context, "color_camera_info_topic"),
        "camera.depth_topic": _value(context, "depth_image_topic"),
        "camera.depth_info_topic": _value(
            context, "depth_camera_info_topic"),
    }

    nodes = [
        Node(
            package="tracer_jaka_mujoco",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            output="screen",
            parameters=[sensors_yaml, bridge_parameters],
        ),
    ]

    if start_rsp:
        with open(urdf_file, "r", encoding="utf-8") as urdf_stream:
            robot_description = urdf_stream.read()
        nodes.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    robot_description, value_type=str),
                "use_sim_time": True,
            }],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="empty",
            choices=sorted(_SCENES),
            description=(
                "Named MuJoCo scene. Default is an empty ground scene.")),
        DeclareLaunchArgument(
            "model",
            default_value="",
            description=(
                "Explicit MuJoCo scene XML path; overrides scene when set.")),
        DeclareLaunchArgument(
            "initial_pose",
            default_value="low",
            choices=["low", "home", "task_contact"],
            description=(
                "Initial MuJoCo keyframe. 'low' avoids the singular "
                "straight-up home pose; 'home' is the legacy arm-up pose.")),
        DeclareLaunchArgument(
            "init_keyframe",
            default_value="",
            description=(
                "Explicit keyframe override. When empty, initial_pose is used.")),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument(
            "publish_odom_tf", default_value="false",
            description=(
                "Publish odom -> base_footprint from mujoco_bridge. "
                "Keep false when robot_localization owns the transform.")),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("start_fts", default_value="true"),
        DeclareLaunchArgument(
            "wheel_odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument(
            "fts_topic", default_value="/fts_broadcaster/wrench"),
        DeclareLaunchArgument(
            "color_image_topic",
            default_value="/camera/d455/color/image_raw"),
        DeclareLaunchArgument(
            "color_camera_info_topic",
            default_value="/camera/d455/color/camera_info"),
        DeclareLaunchArgument(
            "depth_image_topic",
            default_value="/camera/d455/depth/image_raw"),
        DeclareLaunchArgument(
            "depth_camera_info_topic",
            default_value="/camera/d455/depth/camera_info"),
        OpaqueFunction(function=_make_nodes),
    ])
