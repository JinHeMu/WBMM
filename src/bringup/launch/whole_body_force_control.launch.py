#!/usr/bin/env python3
"""Whole-body admittance plus OCS2 algorithm entry point.

This launch starts only algorithms. It never starts real hardware or MuJoCo.
A separate hardware backend must provide the canonical topics.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _enforce_force_motion_gate(context):
    write_enabled = _as_bool(
        LaunchConfiguration("hardware_write").perform(context))
    reference_output = _as_bool(
        LaunchConfiguration("admittance.output").perform(context))
    if reference_output and not write_enabled:
        raise RuntimeError(
            "Force reference output requires hardware_write:=true")
    return []


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")

    urdf = LaunchConfiguration("urdf_file")
    force_base_params_file = LaunchConfiguration("force_base_params_file")
    force_params_file = LaunchConfiguration("force_params_file")
    hardware_write = LaunchConfiguration("hardware_write")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument(
            "initial_task_phase", default_value="-1",
            description=(
                "TaskPhase override for MPC/MRT; -1 uses "
                "modeSwitch.initialPhase from task_file. Force-control "
                "Execution should be commanded via the phase service or by "
                "setting this to 2 explicitly.")),
        DeclareLaunchArgument("admittance.enable", default_value="false"),
        DeclareLaunchArgument("admittance.output", default_value="false"),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description, "urdf", "tracer_jaka_zu5.urdf"])),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "task.info"])),
        DeclareLaunchArgument(
            "ocs2_config", default_value="",
            description="Optional override; empty uses common defaults."),
        DeclareLaunchArgument(
            "force_base_params_file",
            default_value=PathJoinSubstitution([
                bringup, "config", "common", "force_control.yaml"])),
        DeclareLaunchArgument(
            "force_params_file",
            default_value=PathJoinSubstitution([
                bringup, "config", "real", "force_control.yaml"])),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_auto_generated"),
        DeclareLaunchArgument(
            "odom_topic", default_value="/wheel/odometry"),
        OpaqueFunction(function=_enforce_force_motion_gate),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "ocs2.launch.py",
            ])),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "use_rviz": LaunchConfiguration("use_rviz"),
                "config_file": LaunchConfiguration("ocs2_config"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": urdf,
                "lib_folder": LaunchConfiguration("lib_folder"),
                "command_output_enabled": hardware_write,
                "odom_topic": LaunchConfiguration("odom_topic"),
                "initial_task_phase": LaunchConfiguration(
                    "initial_task_phase"),
            }.items(),
        ),
        TimerAction(period=12.0, actions=[
            Node(
                package="whole_body_force_control",
                executable="force_sensor_processor_node",
                name="force_sensor_processor",
                output="screen",
                parameters=[
                    force_base_params_file,
                    force_params_file,
                    {
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool),
                    },
                ],
            ),
            Node(
                package="whole_body_force_control",
                executable="whole_body_force_control_node",
                name="whole_body_force_control",
                output="screen",
                parameters=[
                    force_base_params_file,
                    force_params_file,
                    {
                        "urdf_file": urdf,
                        "admittance.enable": ParameterValue(
                            LaunchConfiguration("admittance.enable"),
                            value_type=bool),
                        "admittance.output": ParameterValue(
                            LaunchConfiguration("admittance.output"),
                            value_type=bool),
                        "use_sim_time": ParameterValue(
                            use_sim_time, value_type=bool),
                    },
                ],
            ),
        ]),
    ])
