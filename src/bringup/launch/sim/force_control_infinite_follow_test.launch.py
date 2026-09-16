#!/usr/bin/env python3
"""Run a sustained whole-body force-follow validation in an open infinite scene."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            bringup, "launch", "whole_body_force_control_sim.launch.py"])),
        launch_arguments={
            "viewer": LaunchConfiguration("viewer"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "profile": "infinite",
        }.items(),
    )

    tester = Node(
        package="whole_body_force_control",
        executable="whole_body_force_control_test.py",
        name="whole_body_force_control_infinite_test",
        output="screen",
        parameters=[{
            "test_profile": "continuous_infinite",
            "report_file": LaunchConfiguration("report_file"),
            "wrench_topic": "/whole_body_force_control/fake_wrench",
            "wrench_frame": "tool0",
            "continuous_force": ParameterValue(
                LaunchConfiguration("force"), value_type=float),
            "continuous_duration": ParameterValue(
                LaunchConfiguration("duration"), value_type=float),
            "baseline_duration": 4.0,
        }],
    )
    delayed_tester = TimerAction(period=14.0, actions=[tester])
    stop_after_test = RegisterEventHandler(
        OnProcessExit(
            target_action=tester,
            on_exit=[EmitEvent(event=Shutdown(
                reason="infinite force-follow test complete"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("force", default_value="7.0"),
        DeclareLaunchArgument("duration", default_value="30.0"),
        DeclareLaunchArgument(
            "report_file",
            default_value="/tmp/whole_body_force_control_infinite_report.json"),
        simulation,
        delayed_tester,
        stop_after_test,
    ])
