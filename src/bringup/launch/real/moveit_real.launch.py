#!/usr/bin/env python3
"""Compose the real hardware interface with shared MoveIt algorithms.

``hardware_write`` is the only real-motion gate:
  * true  -> MoveIt trajectory execution or MoveIt Servo may command JAKA;
  * false -> JAKA is telemetry-only and MoveIt/Servo cannot command it.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("tracer_jaka_bringup")
    use_servo = LaunchConfiguration("use_servo")
    hardware_write = LaunchConfiguration("hardware_write")

    arm_controller_name = PythonExpression([
        "'arm_controller' if '", use_servo,
        "'.lower() == 'true' else 'arm_trajectory_controller'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_servo", default_value="false"),
        DeclareLaunchArgument("use_joy", default_value="false"),
        DeclareLaunchArgument(
            "hardware_write", default_value="false",
            description=(
                "Only real-motion gate: true allows MoveIt/Servo to command "
                "JAKA; false is telemetry-only.")),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("controller_manager_timeout", default_value="30.0"),
        DeclareLaunchArgument("use_gripper", default_value="false"),
        DeclareLaunchArgument("gripper_device_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
        DeclareLaunchArgument("gripper_id", default_value="1"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "hardware_interface.launch.py",
            ])),
            launch_arguments={
                "start_base": "false",
                "start_robot_state_publisher": "true",
                "start_arm_pose": "false",
                "start_imu": "false",
                "start_lidar": "false",
                "start_jaka_hardware": "true",
                "start_jaka_fts": "true",
                "start_arm_controller": "true",
                "arm_controller_name": arm_controller_name,
                "controller_manager_timeout": LaunchConfiguration(
                    "controller_manager_timeout"),
                "publish_odom_tf": "false",
                "jaka_robot_ip": LaunchConfiguration("robot_ip"),
                "jaka_local_ip": LaunchConfiguration("local_ip"),
                "hardware_write": hardware_write,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "moveit.launch.py",
            ])),
            launch_arguments={
                "use_sim_time": "false",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_servo": use_servo,
                "use_joy": LaunchConfiguration("use_joy"),
                "hardware_write": hardware_write,
            }.items(),
        ),
        Node(
            package="dh_gripper_driver",
            executable="dh_ag95_driver",
            name="dh_ag95_driver",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_gripper")),
            parameters=[
                PathJoinSubstitution([
                    bringup_share, "config", "common", "moveit_bringup.yaml",
                ]),
                {
                    "device_port": ParameterValue(
                        LaunchConfiguration("gripper_device_port"),
                        value_type=str),
                    "baudrate": ParameterValue(
                        LaunchConfiguration("gripper_baudrate"),
                        value_type=int),
                    "gripper_id": ParameterValue(
                        LaunchConfiguration("gripper_id"), value_type=int),
                },
            ],
        ),
    ])
