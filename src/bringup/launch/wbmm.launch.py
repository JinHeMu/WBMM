#!/usr/bin/env python3

import os
"""Optional WBMM top-level composition.

Defaults are fail-closed and do not start any hardware, MuJoCo backend or
algorithm. Select a backend and/or algorithm stacks explicitly.
"""

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


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _source(bringup_share, name):
    return PythonLaunchDescriptionSource(PathJoinSubstitution([
        bringup_share, "launch", name]))


def _make_actions(context):
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")

    backend = _value(context, "hardware_backend")
    if backend not in ("none", "real", "mujoco"):
        raise RuntimeError(
            "hardware_backend must be none, real or mujoco")

    use_sim_time = _value(context, "use_sim_time")
    if use_sim_time == "auto":
        use_sim_time = "true" if backend == "mujoco" else "false"

    profile = _value(context, "config_profile")
    if profile not in ("real", "sim"):
        raise RuntimeError("config_profile must be real or sim")
    bringup_share = get_package_share_directory("tracer_jaka_bringup")

    def config_value(name, real_path, sim_path):
        value = _value(context, name)
        if value:
            return value
        return sim_path if profile == "sim" else real_path

    task_file = config_value(
        "task_file",
        os.path.join(bringup_share, "config", "real", "task.info"),
        os.path.join(bringup_share, "config", "sim", "task.info"))
    ocs2_config = config_value(
        "ocs2_config",
        "",  # Real uses common OCS2 defaults without an override file.
        os.path.join(bringup_share, "config", "sim", "ocs2.yaml"))
    remani_config = config_value(
        "remani_config",
        os.path.join(bringup_share, "config", "real", "remani.yaml"),
        os.path.join(bringup_share, "config", "sim", "remani.yaml"))
    ekf_config = config_value(
        "ekf_config",
        os.path.join(bringup_share, "config", "real", "ekf.yaml"),
        os.path.join(bringup_share, "config", "sim", "ekf.yaml"))
    slam_config = config_value(
        "slam_config",
        "",  # Real uses common SLAM defaults without an override file.
        os.path.join(bringup_share, "config", "sim", "slam_toolbox.yaml"))
    force_params_file = config_value(
        "force_params_file",
        os.path.join(bringup_share, "config", "real", "force_control.yaml"),
        os.path.join(bringup_share, "config", "sim", "force_control.yaml"))
    lib_folder = _value(context, "lib_folder")
    if not lib_folder:
        lib_folder = (
            "/tmp/wbmm_ocs2_sim/auto_generated" if profile == "sim"
            else "/tmp/wbmm_ocs2_real/auto_generated")

    # The planner needs a concrete ESDF path. esdf_file is the OCS2 override and
    # may be empty; fall back to static_esdf_file, which is what the planner used
    # before the two were split.
    planner_esdf_file = (
        _value(context, "esdf_file") or _value(context, "static_esdf_file"))

    start_ocs2 = _as_bool(_value(context, "start_ocs2"))
    start_force_control = _as_bool(_value(context, "start_force_control"))
    start_moveit = _as_bool(_value(context, "start_moveit"))
    if start_ocs2 and start_force_control:
        raise RuntimeError(
            "start_force_control already starts OCS2; do not also set "
            "start_ocs2:=true")

    if start_moveit and not (start_ocs2 or start_force_control):
        arm_controller_name = "arm_trajectory_controller"
    else:
        arm_controller_name = "arm_controller"

    actions = []

    if backend == "real":
        actions.append(IncludeLaunchDescription(
            _source(bringup, "wbmm_hardware_interface.launch.py"),
            launch_arguments={
                "hardware_write": _value(context, "hardware_write"),
                "can_port": _value(context, "can_port"),
                "jaka_robot_ip": _value(context, "robot_ip"),
                "jaka_local_ip": _value(context, "local_ip"),
                "start_arm_controller": "true",
                "arm_controller_name": arm_controller_name,
            }.items(),
        ))
    elif backend == "mujoco":
        actions.append(IncludeLaunchDescription(
            _source(bringup, "mujoco_hardware_interface.launch.py"),
            launch_arguments={
                "viewer": _value(context, "viewer"),
                "scene": _value(context, "scene"),
                "model": _value(context, "mujoco_model"),
                "initial_pose": _value(context, "initial_pose"),
                "init_keyframe": _value(context, "init_keyframe"),
                "arm_bias_compensation": _value(context, "sim_arm_bias_compensation"),
                "start_camera": _value(context, "start_camera"),
                "publish_odom_tf": _value(context, "publish_odom_tf"),
            }.items(),
        ))

    if _as_bool(_value(context, "start_localization")):
        actions.append(IncludeLaunchDescription(
            _source(bringup, "localization.launch.py"),
            launch_arguments={
                "start_ekf": "true",
                "start_slam": _value(context, "start_slam"),
                "use_sim_time": use_sim_time,
                "ekf_config": ekf_config,
                "slam_config": slam_config,
            }.items(),
        ))

    if start_ocs2:
        ocs2_launch_arguments = {
            "use_sim_time": use_sim_time,
            "use_rviz": _value(context, "use_rviz"),
            "config_file": ocs2_config,
            "task_file": task_file,
            "urdf_file": _value(context, "urdf_file"),
            "lib_folder": lib_folder,
            "command_output_enabled": _value(
                context, "hardware_write"),
            "odom_topic": _value(context, "odom_topic"),
            "esdf_file": _value(context, "esdf_file"),
            "world_frame": _value(context, "world_frame"),
            "use_target": _value(context, "use_target"),
        }
        rviz_config = _value(context, "rviz_config")
        if rviz_config:
            ocs2_launch_arguments["rviz_config"] = rviz_config
        actions.append(IncludeLaunchDescription(
            _source(bringup, "ocs2.launch.py"),
            launch_arguments=ocs2_launch_arguments.items(),
        ))

    # WBMM-native planner. `start_remani` is kept as a deprecated alias so
    # existing commands keep working, but both spellings now start the WBMM
    # planner and its reference bridge; the REMANI upstream is no longer used.
    if _as_bool(_value(context, "start_planning")) or _as_bool(
        _value(context, "start_remani")
    ):
        planning_arguments = {
            "use_sim_time": use_sim_time,
            "urdf_file": _value(context, "urdf_file"),
            "esdf_file": planner_esdf_file,
            "world_frame": _value(context, "remani_planner_frame"),
            "bridge_world_frame": _value(context, "world_frame") or "odom",
            "odom_topic": _value(context, "odom_topic"),
            "joint_state_topic": _value(context, "joint_state_topic"),
            "goal_topic": _value(context, "goal_topic"),
            "enable_optimization": _value(context, "planner_enable_optimization"),
            "cruise_speed": _value(context, "planner_cruise_speed"),
            "max_linear_velocity": _value(context, "planner_max_linear_velocity"),
            "max_yaw_rate": _value(context, "planner_max_yaw_rate"),
            "max_joint_velocity": _value(context, "planner_max_joint_velocity"),
            "max_base_speed": _value(context, "planner_max_base_speed"),
            "max_base_yaw_rate": _value(context, "planner_max_base_yaw_rate"),
            "collision_safety_margin": _value(
                context, "planner_collision_safety_margin"),
            "treat_unknown_as_occupied": _value(
                context, "planner_treat_unknown_as_occupied"),
        }
        actions.append(IncludeLaunchDescription(
            _source(bringup, "wbmm_planning.launch.py"),
            launch_arguments=planning_arguments.items(),
        ))

    if start_force_control:
        actions.append(IncludeLaunchDescription(
            _source(bringup, "whole_body_force_control.launch.py"),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": _value(context, "use_rviz"),
                "hardware_write": _value(context, "hardware_write"),
                "force_params_file": force_params_file,
                "ocs2_config": ocs2_config,
                "task_file": task_file,
                "urdf_file": _value(context, "urdf_file"),
                "lib_folder": lib_folder,
                "odom_topic": _value(context, "odom_topic"),
                "esdf_file": _value(context, "esdf_file"),
                "world_frame": _value(context, "world_frame"),
            }.items(),
        ))

    if start_moveit:
        actions.append(IncludeLaunchDescription(
            _source(bringup, "moveit.launch.py"),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": _value(context, "use_rviz"),
                "hardware_write": _value(context, "hardware_write"),
            }.items(),
        ))

    return actions


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_backend", default_value="none",
            choices=["none", "real", "mujoco"],
            description="Hardware backend started by this entry point."),
        DeclareLaunchArgument(
            "use_sim_time", default_value="auto",
            choices=["auto", "true", "false"]),
        DeclareLaunchArgument(
            "config_profile", default_value="real",
            choices=["real", "sim"],
            description="Selects default config paths; explicit paths win."),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("start_camera", default_value="false"),
        DeclareLaunchArgument("publish_odom_tf", default_value="false"),
        DeclareLaunchArgument(
            "scene", default_value="empty",
            choices=[
                "empty", "room", "task_table", "force_follow_infinite",
                "force_follow_5m", "nvblox_remani_demo",
                "esdf_validation"]),
        DeclareLaunchArgument(
            "mujoco_model", default_value="",
            description="Explicit scene XML path; overrides scene."),
        DeclareLaunchArgument(
            "initial_pose", default_value="low",
            choices=["low", "home", "task_contact"]),
        DeclareLaunchArgument("init_keyframe", default_value=""),
        DeclareLaunchArgument("sim_arm_bias_compensation", default_value="false"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("start_localization", default_value="false"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_ocs2", default_value="false"),
        DeclareLaunchArgument("start_remani", default_value="false"),
        DeclareLaunchArgument("start_force_control", default_value="false"),
        DeclareLaunchArgument("start_moveit", default_value="false"),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description, "urdf", "tracer_jaka_zu5.urdf"])),
        DeclareLaunchArgument("task_file", default_value=""),
        DeclareLaunchArgument("ocs2_config", default_value=""),
        DeclareLaunchArgument("remani_config", default_value=""),
        DeclareLaunchArgument("ekf_config", default_value=""),
        DeclareLaunchArgument("slam_config", default_value=""),
        DeclareLaunchArgument("force_params_file", default_value=""),
        DeclareLaunchArgument("lib_folder", default_value=""),
        DeclareLaunchArgument(
            "odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("static_esdf_file", default_value=""),
        DeclareLaunchArgument(
            "esdf_file", default_value="",
            description=(
                "Optional ESDF NPZ override for environmentCollision; "
                "empty uses the task file/default.")),
        DeclareLaunchArgument(
            "world_frame", default_value="",
            description=(
                "Optional OCS2 world frame override; empty uses the "
                "loaded parameter profile.")),
        DeclareLaunchArgument(
            "start_planning", default_value="false",
            description=(
                "Start the WBMM planner and its OCS2 reference bridge.")),
        DeclareLaunchArgument(
            "goal_topic", default_value="/goal_pose",
            description="2D navigation goal consumed by the WBMM planner."),
        DeclareLaunchArgument("planner_enable_optimization", default_value="true"),
        DeclareLaunchArgument("planner_cruise_speed", default_value="0.35"),
        DeclareLaunchArgument(
            "planner_max_linear_velocity", default_value="0.5",
            description=(
                "Controller envelope the planner must stay inside. Must match "
                "jointVelocityLimits in the task file.")),
        DeclareLaunchArgument(
            "planner_max_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "planner_max_joint_velocity", default_value="2.0"),
        DeclareLaunchArgument("planner_max_base_speed", default_value="0.5"),
        DeclareLaunchArgument(
            "planner_max_base_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "planner_collision_safety_margin", default_value="0.0",
            description="Extra clearance added to every collision sphere."),
        DeclareLaunchArgument(
            "planner_treat_unknown_as_occupied", default_value="false",
            description=(
                "Reject ESDF queries touching unobserved space. map1 records "
                "unknown_is_occupied=false, so the default trusts it.")),
        DeclareLaunchArgument(
            "remani_planner_frame", default_value="",
            description="Planning frame; empty adopts the ESDF frame."),
        DeclareLaunchArgument(
            "remani_target_frame", default_value="odom",
            description="Deprecated; the WBMM planner uses world_frame only."),
        DeclareLaunchArgument(
            "remani_use_tf_transform", default_value="false",
            description="Deprecated; the WBMM planner uses world_frame only."),
        DeclareLaunchArgument("use_target", default_value="false"),
        DeclareLaunchArgument(
            "rviz_config", default_value="",
            description=(
                "Optional RViz config path for the OCS2 stack.")),
        OpaqueFunction(function=_make_actions),
    ])
