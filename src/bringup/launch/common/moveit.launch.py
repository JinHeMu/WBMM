#!/usr/bin/env python3
"""Start only MoveIt planning, visualization and optional Servo nodes.

State sources and command consumers belong to the deployment launch.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    moveit_share = get_package_share_directory("tracer_jaka_moveit_config")

    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    use_servo = LaunchConfiguration("use_servo")
    use_joy = LaunchConfiguration("use_joy")
    hardware_write = LaunchConfiguration("hardware_write")
    allow_trajectory_execution = ParameterValue(PythonExpression([
        "'", hardware_write, "'.lower() == 'true' and '",
        use_servo, "'.lower() != 'true'",
    ]), value_type=bool)
    servo_enabled = PythonExpression([
        "'", use_servo, "'.lower() == 'true' and '",
        hardware_write, "'.lower() == 'true'",
    ])
    servo_and_joy = PythonExpression([
        "'", use_servo, "'.lower() == 'true' and '",
        use_joy, "'.lower() == 'true' and '",
        hardware_write, "'.lower() == 'true'",
    ])
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka_zu5",
            package_name="tracer_jaka_moveit_config",
        )
        .planning_pipelines(
            default_planning_pipeline="chomp", pipelines=["ompl", "chomp"])
        .to_moveit_configs()
    )
    with open(os.path.join(
            bringup_share, "config", "common", "moveit_servo.yaml"),
            "r", encoding="utf-8") as stream:
        servo_parameters = {"moveit_servo": yaml.safe_load(stream)}

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
    servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        condition=IfCondition(servo_enabled),
        parameters=[
            servo_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
    )
    joy_to_servo = ComposableNodeContainer(
        name="moveit_servo_joy_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        condition=IfCondition(servo_and_joy),
        composable_node_descriptions=[
            ComposableNode(
                package="jaka_driver", plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node"),
            ComposableNode(
                package="joy", plugin="joy::Joy", name="joy_node"),
        ],
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
        DeclareLaunchArgument("use_servo", default_value="false"),
        DeclareLaunchArgument("use_joy", default_value="false"),
        DeclareLaunchArgument(
            "hardware_write", default_value="false",
            description=(
                "Deployment-provided motion gate: false disables MoveIt/Servo "
                "command output, true permits the configured owner.")),
        move_group, servo, joy_to_servo, rviz,
    ])
