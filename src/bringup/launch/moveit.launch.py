#!/usr/bin/env python3
"""Start only MoveIt planning and visualization.

State sources and command consumers belong to the hardware/simulation backend
and deployment launch. MoveIt Servo and joystick control are intentionally not
part of this launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("tracer_jaka_moveit_config")

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    hardware_write = LaunchConfiguration("hardware_write")
    allow_trajectory_execution = ParameterValue(
        hardware_write, value_type=bool)

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka_zu5",
            package_name="tracer_jaka_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="chomp", pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), {
            "use_sim_time": use_sim_time,
            "allow_trajectory_execution": allow_trajectory_execution,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "hardware_write", default_value="false",
            description=(
                "Deployment-provided motion gate: false disables MoveIt "
                "trajectory execution, true permits the configured owner.")),
        move_group,
        rviz,
    ])
