#!/usr/bin/env python3
"""Automated fake-wrench validation of the independent force-follow stack."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    run_test = LaunchConfiguration("run_test")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup, "launch", "whole_body_force_control_sim.launch.py"])),
        launch_arguments={
            "viewer": LaunchConfiguration("viewer"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "force_control_armed": "true",
            "force_reference_output_enabled": "true",
        }.items(),
    )

    tester = Node(
        package="whole_body_force_control",
        executable="whole_body_force_control_test.py",
        output="screen",
        parameters=[{
            "report_file": LaunchConfiguration("report_file"),
            "wrench_topic": "/whole_body_force_control/fake_wrench",
            "wrench_frame": "tool0",
            "low_force": ParameterValue(
                LaunchConfiguration("low_force"), value_type=float),
            "high_force": ParameterValue(
                LaunchConfiguration("high_force"), value_type=float),
            "pull_force": ParameterValue(
                LaunchConfiguration("pull_force"), value_type=float),
            "baseline_duration": ParameterValue(
                LaunchConfiguration("baseline_duration"), value_type=float),
            "low_force_duration": ParameterValue(
                LaunchConfiguration("low_force_duration"), value_type=float),
            "high_force_duration": ParameterValue(
                LaunchConfiguration("high_force_duration"), value_type=float),
            "release_duration": ParameterValue(
                LaunchConfiguration("release_duration"), value_type=float),
            "pull_duration": ParameterValue(
                LaunchConfiguration("pull_duration"), value_type=float),
            "final_release_duration": ParameterValue(
                LaunchConfiguration("final_release_duration"), value_type=float),
        }],
        condition=IfCondition(run_test),
    )
    delayed_tester = TimerAction(period=14.0, actions=[tester])
    stop_after_test = RegisterEventHandler(
        OnProcessExit(
            target_action=tester,
            on_exit=[EmitEvent(event=Shutdown(
                reason="force-control fake-wrench test complete"))],
        ),
        condition=IfCondition(run_test),
    )

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("run_test", default_value="true"),
        DeclareLaunchArgument("low_force", default_value="5.0"),
        DeclareLaunchArgument("high_force", default_value="12.0"),
        DeclareLaunchArgument("pull_force", default_value="-8.0"),
        DeclareLaunchArgument("baseline_duration", default_value="4.0"),
        DeclareLaunchArgument("low_force_duration", default_value="10.0"),
        DeclareLaunchArgument("high_force_duration", default_value="15.0"),
        DeclareLaunchArgument("release_duration", default_value="10.0"),
        DeclareLaunchArgument("pull_duration", default_value="12.0"),
        DeclareLaunchArgument("final_release_duration", default_value="10.0"),
        DeclareLaunchArgument(
            "report_file",
            default_value="/tmp/whole_body_force_control_test_report.json"),
        simulation,
        delayed_tester,
        stop_after_test,
    ])
