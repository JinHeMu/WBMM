#!/usr/bin/env python3
"""Independent real-hardware OCS2 + whole-body force-follow bringup.

No WipePlanner or REMANI target bridge is started. This launch uses the OCS2
real-motion gates plus the force controller's own arm switch.
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
    return value.lower() in ("1", "true", "yes", "on")


def _enforce_force_motion_gate(context):
    release = _as_bool(LaunchConfiguration("safety_release").perform(context))
    read_only = _as_bool(LaunchConfiguration("jaka_read_only").perform(context))
    command_output = _as_bool(
        LaunchConfiguration("command_output_enabled").perform(context))
    reference_output = _as_bool(
        LaunchConfiguration(
            "force_reference_output_enabled").perform(context))

    if reference_output and not command_output:
        raise RuntimeError(
            "force_reference_output_enabled:=true requires "
            "command_output_enabled:=true")
    if reference_output and (read_only or not release):
        raise RuntimeError(
            "Real force reference output requires jaka_read_only:=false and "
            "safety_release:=true")
    return []


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")
    force_control = FindPackageShare("whole_body_force_control")

    urdf = LaunchConfiguration("urdf_file")
    force_params_file = LaunchConfiguration("force_params_file")
    jaka_read_only = LaunchConfiguration("jaka_read_only")
    command_output = LaunchConfiguration("command_output_enabled")
    safety_release = LaunchConfiguration("safety_release")
    force_armed = LaunchConfiguration("force_control_armed")
    force_reference_output = LaunchConfiguration(
        "force_reference_output_enabled")

    hardware_and_ocs2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "ocs2_real.launch.py"])
        ),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_base": "true",
            "start_robot_state_publisher": "true",
            "mrt_odom_topic": "/odom",
            "publish_odom_tf": "true",
            "use_joy": "false",
            "can_port": LaunchConfiguration("can_port"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "local_ip": LaunchConfiguration("local_ip"),
            "jaka_read_only": jaka_read_only,
            "command_output_enabled": command_output,
            "safety_release": safety_release,
            "task_file": LaunchConfiguration("task_file"),
            "urdf_file": urdf,
            "arm_max_delta_per_step": "0.03",
            "arm_max_command_velocity": "0.08",
        }.items(),
    )

    controller = Node(
        package="whole_body_force_control",
        executable="whole_body_force_control_node",
        name="whole_body_force_control",
        output="screen",
        parameters=[
            force_params_file,
            {
                "urdf_file": urdf,
                "ee_frame": "tool0",
                "robot_name": "mobile_manipulator",
                "wrench_topic": "/fts_broadcaster/wrench",
                "armed": ParameterValue(force_armed, value_type=bool),
                "reference_output_enabled": ParameterValue(
                    force_reference_output, value_type=bool),
                "use_sim_time": False,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("jaka_read_only", default_value="true"),
        DeclareLaunchArgument("command_output_enabled", default_value="false"),
        DeclareLaunchArgument("safety_release", default_value="false"),
        DeclareLaunchArgument("force_control_armed", default_value="false"),
        DeclareLaunchArgument(
            "force_reference_output_enabled", default_value="false"),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                description, "urdf", "tracer_jaka_zu5.urdf"])),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("tracer_jaka_ocs2"),
                "config", "task_real.info"])),
        DeclareLaunchArgument(
            "force_params_file",
            default_value=PathJoinSubstitution([
                force_control, "config", "force_follow_real.yaml"])),
        OpaqueFunction(function=_enforce_force_motion_gate),
        hardware_and_ocs2,
        TimerAction(period=12.0, actions=[controller]),
    ])

